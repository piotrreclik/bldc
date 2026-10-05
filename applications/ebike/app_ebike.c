/*
	Copyright 2019 - 2020 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	*/

#include "app.h"

#include "mc_interface.h"
//#include "mcpwm_foc.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"
#include "app_ebike.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// --- Configuration Constants ---
#define MIN_SPEED_RATIO   500.0f
#define MAX_SPEED_RATIO   3000.0f

// --- Velocity Logic Thresholds ---
// 6.0 km/h = 1.67 m/s. The inversion point for the asymmetrical filter behavior.
#define SPEED_THRESHOLD_MS    1.67f    
#define ERPM_STOP_THRESHOLD   5.0f 

// --- Filter Smoothing Rates (Alpha Coefficients) ---
#define ALPHA_FAST            0.5f    // High-responsiveness rate 
#define ALPHA_SLOW            0.15f    // dampened rate 

#define MOTOR_ACTIVE_THRESHOLD_ERPM 500

#define PAS_SPEED_MS      		   6.94f // 25 km/h
#define CLASS12_SPEED_MS           8.89f // 32 km/h
#define CLASS3_PAS_SPEED_MS        12.52f // 45 km/h
#define THROTTLE_SPEED_MS 		   1.11f // 4 km/h
#define CUTOFF_THROTTLE_SPEED_MS   1.67f // 6 km/h

#define APP_UPDATE_RATE_HZ 		  50
#define APP_SLEEP_MS 			  (1000 / APP_UPDATE_RATE_HZ)
#define DISABLE_APP_OUTPUT_MS 	  (APP_SLEEP_MS * 2)
#define APP_UPDATE_LOOP_DT        (1.0f / (float)APP_UPDATE_RATE_HZ)
#define PID_SPIN_UP_TIME_MS		  500
#define PID_SPIN_UP_CYCLES		  (PID_SPIN_UP_TIME_MS / APP_SLEEP_MS)
#define WALK_PID_RAMP_UP_TIME_MS  1000
#define WALK_PID_RAMP_UP_CYCLES	  (WALK_PID_RAMP_UP_TIME_MS / APP_SLEEP_MS)
#define SPIN_UP_BLANKING_TIME_MS  200
#define SPIN_UP_BLANKING_CYCLES	  (SPIN_UP_BLANKING_TIME_MS / APP_SLEEP_MS)

#define CUSTOM_SPEED_LIMIT_EEPROM_ADDR    42

//typedef enum {
//    BELOW_ERPM_THRESHOLD,
//    SPINNING_UP,
//    SPINNING_DOWN_UNLOADED,
//    SPINNING_AT_TARGET_ERPM,
//	UNDER_LOAD
//} MotorState;

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private functions
static void ebike_test(int argc, const char **argv);
static void terminal_set_custom_speed(int argc, const char **argv);
float load_my_custom_speed(void);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile bool was_pid = false;
static volatile bool release_motor = false;
static volatile bool walk_pid_speed_set_or_ramping = false;
static volatile bool override_adc1 = false;
static volatile bool adc1_detached = false;
static volatile bool pas_detached = false;
static volatile float last_app_pwr;
static volatile float last_erpm;
static volatile float last_current = 0.0f;
static volatile uint8_t pid_ramp_up_step = WALK_PID_RAMP_UP_CYCLES;
static volatile float pid_ramp_start_erpm = 900.0f;
static volatile uint8_t spin_up_step = PID_SPIN_UP_CYCLES;
static float class12_speed_ms = 0.0f;
static float filtered_pas_max_erpm = -1.0f; 

static volatile float max_erpm = 100000.0f;

static float filtered_speed_ratio = (MIN_SPEED_RATIO + MAX_SPEED_RATIO) / 2;
static float smoothed_speed_ratio = (MIN_SPEED_RATIO + MAX_SPEED_RATIO) / 2;
static float last_hardware_ratio = (MIN_SPEED_RATIO + MAX_SPEED_RATIO) / 2;

static volatile EBIKE_MODE_T mode = EBIKE_MODE_COMPLIANT;
static volatile EBIKE_MODE_T prev_mode = EBIKE_MODE_UNDEFINED;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"ebike",
			"ebike",
			0,
			ebike_test);
	terminal_register_command_callback(
        "set_custom_speed",
        "Saves custom speed limit directly to a dedicated EEPROM address. Usage: set_custom_speed [value]",
        "value",
        terminal_set_custom_speed
    );
	class12_speed_ms = load_my_custom_speed();
	app_adc_start(false);
	app_pas_start(false);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	terminal_unregister_callback(ebike_test);
	terminal_unregister_callback(terminal_set_custom_speed);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

void app_ebike_set_mode(EBIKE_MODE_T new_mode) {
	mode = new_mode;
}

//static float get_current_speed_erpm(void) {
//	const float current_speed = mc_interface_get_speed();
//	return current_speed * smoothed_speed_ratio;
//}

void app_custom_release_motor(void) {
	release_motor = true;
}

/*static void schedule_spin_up(void) {
	spin_up_step = 0;
}

static void cancel_spin_up(void) {
	spin_up_step = PID_SPIN_UP_CYCLES;
}

static void spin_up_if_scheduled(float target_erpm) {
	if (spin_up_step < PID_SPIN_UP_CYCLES) {
		spin_up_step++;
		if (mc_interface_get_tot_current_filtered() > 5.0) {
			cancel_spin_up();
			return;
		}
		target_erpm *= 0.9f;
		app_disable_output(DISABLE_APP_OUTPUT_MS);
		float current_target_erpm = utils_map((float)spin_up_step, 1.0f, (float)PID_SPIN_UP_CYCLES, 1000.0f, target_erpm);
		utils_truncate_number(&current_target_erpm, 1000.0f, target_erpm);
		mc_interface_set_pid_speed(current_target_erpm);
	}					
}

static bool is_spinning_up(void) {
	return spin_up_step > 0 && spin_up_step < PID_SPIN_UP_CYCLES;
}*/

static void detach_adc1(void) {
	adc1_detached = true;
	app_adc_adc1_override(0.0);
	app_adc_detach_adc(1);
}

static void reattach_adc1_if_detached(void) {
	if (adc1_detached) {
		adc1_detached = false;
		app_adc_detach_adc(0);
	}
}

static void detach_pas(void) {
	pas_detached = true;
	app_pas_pas_override(0.0);
	app_pas_detach_pas(1);
}

static void reattach_pas_if_detached(void) {
	if (pas_detached) {
		pas_detached = false;
		app_pas_detach_pas(0);
	}
}

static float update_dynamic_alpha(float base_alpha, float target_speed_ms, float current_speed, float speed_diff, bool first_speed_redout) {
    float alpha = base_alpha;
    
    const float speed_error_pct = fabsf(target_speed_ms - current_speed) / target_speed_ms;
    const float dynamic_trigger = speed_diff + speed_error_pct;
    
    if (dynamic_trigger > 0.2f && !first_speed_redout) {
        const float exponent = 0.2f / dynamic_trigger;
        alpha = powf(alpha, exponent);
        
        // Clamp between the provided base and the absolute maximum safety ceiling (0.8f)
        utils_truncate_number(&alpha, base_alpha, 0.8f);
    }
    return alpha;
}

/**
 * Pipeline dedicated ONLY to ultra-low speed maneuvers like Walk Assist.
 * Uses strict current-load scaling to prevent overshoots from destroying the ratio at no-load.
 */
static void process_walk_assist_pipeline(float target_speed_ms, float current_speed, float current_erpm, 
                                         float current_current, float estimated_speed, bool sensor_pulse_arrived, 
                                         int spin_up_blanking_cycles, bool motor_coasting, float speed_diff, bool first_speed_redout) {
    static float erpm_sum = 0.0f;
    static int erpm_sample_count = 0;

    // Dynamically calculate alpha for Walk Assist using ALPHA_SLOW as the foundation
    float active_alpha = update_dynamic_alpha(ALPHA_SLOW, target_speed_ms, current_speed, speed_diff, first_speed_redout);

    // Boundaries tuned for Walk Assist context
    const bool overshot_target = (estimated_speed > (target_speed_ms * 1.15f)) || (current_speed > (target_speed_ms * 1.05f));
    const bool undershot_target = (estimated_speed < target_speed_ms * 0.90f) && (current_speed < (target_speed_ms * 0.90f));
    
    const float last_target_erpm = target_speed_ms * smoothed_speed_ratio;
    const bool motor_at_no_load_target = (fabsf(current_erpm - last_target_erpm) < (last_target_erpm * 0.05f)) && (current_current < 2.7f);

    const bool can_update_filter = sensor_pulse_arrived && (spin_up_blanking_cycles == 0) && !motor_coasting && !motor_at_no_load_target; //&& !is_spinning_up();
    const bool is_flushing = (spin_up_blanking_cycles > 0) || motor_coasting || (motor_at_no_load_target && !undershot_target && !overshot_target);

    if (!is_flushing) {
        erpm_sum += current_erpm;
        erpm_sample_count++;
    }

    if (can_update_filter) {
        const float exact_mean_erpm = (erpm_sample_count > 2) ? (erpm_sum / (float)erpm_sample_count) : current_erpm;
        float raw_speed_ratio = exact_mean_erpm / current_speed;
        
        utils_truncate_number(&raw_speed_ratio, MIN_SPEED_RATIO, MAX_SPEED_RATIO);
        UTILS_LP_FAST(filtered_speed_ratio, raw_speed_ratio, active_alpha);
        last_hardware_ratio = filtered_speed_ratio;
        
        commands_printf("RU WALK SR:%0.2f, CC:%0.2f, Alpha:%0.2f", (double)filtered_speed_ratio, (double)current_current, (double)active_alpha);
        erpm_sum = 0.0f;
        erpm_sample_count = 0;
    } 
    else if (is_flushing) {
        erpm_sum = 0.0f;
        erpm_sample_count = 0;
    } 
    else {
        if (overshot_target && (current_current > 4.0f) && !motor_coasting) {
            // Proportional current scaling model for the low-speed domain (Walk Assist)
            float current_load_factor = current_current / (5.0f + current_current);
            float step_down = target_speed_ms * 0.008f * current_load_factor;
            utils_truncate_number(&step_down, 0.0001f, 0.02f);

            filtered_speed_ratio -= (filtered_speed_ratio * step_down);
            //commands_printf("RT WALK [-%0.2f%%]: %0.2f, CC: %0.2f", (double)(step_down * 100.0f), (double)filtered_speed_ratio, (double)current_current);
        } 
        else if (undershot_target && motor_at_no_load_target && !motor_coasting) { // && !is_spinning_up()) {
            float step_up = target_speed_ms * 0.008f;
            utils_truncate_number(&step_up, 0.002f, 0.02f);

            filtered_speed_ratio += (filtered_speed_ratio * step_up);
            //commands_printf("RT WALK [+%0.2f%%]: %0.2f", (double)(step_up * 100.0f), (double)filtered_speed_ratio);
        }
    }
}

/**
 * Pipeline dedicated ONLY to standard higher speeds (e.g., 25 km/h).
 * Provides rapid, assertive corrections without current limits to firmly enforce legal limits.
 */
static void process_high_speed_pipeline(float target_speed_ms, float current_speed, float current_erpm, 
                                        float current_current, float estimated_speed, bool sensor_pulse_arrived, 
                                        int spin_up_blanking_cycles, bool motor_coasting, float speed_diff, bool first_speed_redout) {
    static float erpm_sum = 0.0f;
    static int erpm_sample_count = 0;

    // Dynamically calculate alpha for High Speed using ALPHA_FAST as the foundation
    float active_alpha = update_dynamic_alpha(ALPHA_FAST, target_speed_ms, current_speed, speed_diff, first_speed_redout);

    const bool overshot_target = (estimated_speed > (target_speed_ms * 1.15f)) || (current_speed > (target_speed_ms * 1.05f));
    const bool undershot_target = (estimated_speed < target_speed_ms) && current_speed < (target_speed_ms * 0.99f);
    
    const float last_target_erpm = target_speed_ms * smoothed_speed_ratio;
    const bool motor_at_no_load_target = (fabsf(current_erpm - last_target_erpm) < (last_target_erpm * 0.06f)) && (current_current < 3.0f);
	const bool motor_has_load = (current_current > 4.0f) && !motor_coasting; // && !is_spinning_up();

    const bool can_update_filter = sensor_pulse_arrived && (spin_up_blanking_cycles == 0) && !motor_coasting && !motor_at_no_load_target;// && !is_spinning_up();
    const bool is_flushing = (spin_up_blanking_cycles > 0) || motor_coasting || (motor_at_no_load_target && !undershot_target && !overshot_target);

    if (!is_flushing) {
        erpm_sum += current_erpm;
        erpm_sample_count++;
    }

    if (can_update_filter) {
        const float exact_mean_erpm = (erpm_sample_count > 2) ? (erpm_sum / (float)erpm_sample_count) : current_erpm;
        float raw_speed_ratio = exact_mean_erpm / current_speed;
        
        utils_truncate_number(&raw_speed_ratio, MIN_SPEED_RATIO, MAX_SPEED_RATIO);
        UTILS_LP_FAST(filtered_speed_ratio, raw_speed_ratio, active_alpha);
        last_hardware_ratio = filtered_speed_ratio;
        
        //commands_printf("RU HIGH SR:%0.2f, CC:%0.2f, Alpha:%0.2f", (double)filtered_speed_ratio, (double)current_current, (double)active_alpha);
        erpm_sum = 0.0f;
        erpm_sample_count = 0;
    } 
    else if (is_flushing) {
        erpm_sum = 0.0f;
        erpm_sample_count = 0;
    } 
    else {
        if (overshot_target && motor_has_load && !motor_at_no_load_target) {
            // High speed context uses 1.0f (full response strength) as per your design
            float step_down = target_speed_ms * 0.008f * 1.0f;
            utils_truncate_number(&step_down, 0.0001f, 0.02f);

            filtered_speed_ratio -= (filtered_speed_ratio * step_down);
            //commands_printf("RT HIGH [-%0.2f%%]: %0.2f", (double)(step_down * 100.0f), (double)filtered_speed_ratio);
        } 
        else if (undershot_target && motor_at_no_load_target && !motor_coasting) { // && !is_spinning_up()) {
            float step_up = target_speed_ms * 0.008f;
            utils_truncate_number(&step_up, 0.002f, 0.02f);

            filtered_speed_ratio += (filtered_speed_ratio * step_up);
            //commands_printf("RT HIGH [+%0.2f%%]: %0.2f", (double)(step_up * 100.0f), (double)filtered_speed_ratio);
        }
    }
}

/**
 * --- MAIN ENTRY POINT (Executed at 50Hz) ---
 * Manages core data acquisition, routes telemetry to specific speed sub-methods, 
 * handles cutoffs, and formats final ERPM loop output.
 */
static float calculate_target_max_erpm_with_cutoff(float target_speed_ms, float cutoff_speed_ms) {
    static float prev_speed_ms = 0.0f;
    static int spin_up_blanking_cycles = 0; 
	static bool overshot_cutoff = false;
    

    // --- 1. DATA ACQUISITION ---
    const float current_erpm    = mc_interface_get_rpm();
    const float current_speed   = mc_interface_get_speed();
    const float current_current = mc_interface_get_tot_current_filtered();
    const float speed_diff      = fabsf(current_speed - prev_speed_ms);

    if (spin_up_blanking_cycles > 0) {
        spin_up_blanking_cycles--;
    }

    // --- 2. COMMON ANALYSIS ---
    const float estimated_speed    = current_erpm / last_hardware_ratio;
    const bool first_speed_redout   = (prev_speed_ms == 0.0f);
    const bool motor_coasting       = ((last_erpm - current_erpm) > 10.0f) && (fabsf(current_current) < 0.1f);
    const bool sensor_pulse_arrived = speed_diff > 0.0001f;
	
	if (overshot_cutoff) {
		overshot_cutoff = current_speed > target_speed_ms;
	} else {
		overshot_cutoff = current_speed > cutoff_speed_ms;
	}

    // --- 3. STARTUP FLUSHES ---
    if (fabsf(last_erpm) < MOTOR_ACTIVE_THRESHOLD_ERPM && fabsf(current_erpm) >= MOTOR_ACTIVE_THRESHOLD_ERPM) {
        spin_up_blanking_cycles = SPIN_UP_BLANKING_CYCLES;
    }

    // --- 4. ROUTING BASED ON SPEED DOMAIN ---
    if (current_speed > 0.05f && fabsf(current_erpm) > MOTOR_ACTIVE_THRESHOLD_ERPM) {
        
        if (target_speed_ms < SPEED_THRESHOLD_MS) {
            process_walk_assist_pipeline(target_speed_ms, current_speed, current_erpm, current_current, 
                                         estimated_speed, sensor_pulse_arrived, spin_up_blanking_cycles, 
                                         motor_coasting, speed_diff, first_speed_redout);
        } else {
            process_high_speed_pipeline(target_speed_ms, current_speed, current_erpm, current_current, 
                                        estimated_speed, sensor_pulse_arrived, spin_up_blanking_cycles, 
                                        motor_coasting, speed_diff, first_speed_redout);
        }
    } 
    // --- 5. CRITICAL ASYNC SAFETY BOUNDARIES ---
    else if (overshot_cutoff && last_current > 3.5f) {
        filtered_speed_ratio -= (filtered_speed_ratio * 0.1f); 
        //commands_printf("OST [-10.0%%]: %0.2f", (double)filtered_speed_ratio);
    } 

    // --- 6. SIGNAL SMOOTHING & OUTPUT GENERATION ---
    utils_truncate_number(&filtered_speed_ratio, MIN_SPEED_RATIO, MAX_SPEED_RATIO);
    UTILS_LP_FAST(smoothed_speed_ratio, filtered_speed_ratio, 0.05f);

    prev_speed_ms = current_speed;
    last_erpm     = current_erpm;
    last_current  = current_current;

    return overshot_cutoff ? 0.0f : (target_speed_ms * smoothed_speed_ratio);
}

static float calculate_target_max_erpm(float target_speed_ms) {	
	return calculate_target_max_erpm_with_cutoff(target_speed_ms, target_speed_ms * 1.06f);
}

static float get_max_pas_erpm(void) {
	const volatile app_configuration *app_conf = app_get_configuration();
	const volatile mc_configuration *mc_conf = mc_interface_get_configuration();

	if (!app_conf || !mc_conf) {
		return 0.0f; 
	}

	float gear_ratio = mc_conf->si_gear_ratio;
	float pole_pairs = (float)mc_conf->si_motor_poles / 2.0f;
	float max_pedal_cadence = app_conf->app_pas_conf.pedal_rpm_end;
	float min_pedal_cadence = app_conf->app_pas_conf.pedal_rpm_start;
	if (max_pedal_cadence < min_pedal_cadence) {
		return mc_interface_get_configuration()->l_max_erpm;
	}
	float max_pas_erpm = max_pedal_cadence * gear_ratio * pole_pairs;
	// in order to start taper the power only after the max cadence rpm is reached
	// we need to extend the limit by the complement of erpm current limit start
	float erpm_start_ratio = mc_conf->l_erpm_start; 
	utils_truncate_number(&erpm_start_ratio, 0.0f, 1.0f);
	float mirrored_ratio = 1.0f - erpm_start_ratio;
	return max_pas_erpm + (2 * mirrored_ratio * max_pas_erpm);
}

static void set_max_erpm(float lo_max_erpm) {
	utils_truncate_number(&lo_max_erpm, 1500, mc_interface_get_configuration()->l_max_erpm);
	max_erpm = lo_max_erpm;
}

static void calculate_and_set_pas_max_erpm(float app_adc_pwr) {
	float target_speed_max_erpm = 10000;
	
	if (mode == EBIKE_MODE_COMPLIANT) {
		target_speed_max_erpm = calculate_target_max_erpm(PAS_SPEED_MS);
	} else if (mode == EBIKE_MODE_CLASS1 || mode == EBIKE_MODE_CLASS2) {
		target_speed_max_erpm = calculate_target_max_erpm(class12_speed_ms);
	} else if (mode == EBIKE_MODE_CLASS3) {
		target_speed_max_erpm = calculate_target_max_erpm(CLASS3_PAS_SPEED_MS);
	}
	if (target_speed_max_erpm < ERPM_STOP_THRESHOLD) {
		//app_disable_output(DISABLE_APP_OUTPUT_MS);
		//mc_interface_release_motor();
		//schedule_spin_up();
		detach_pas();
		detach_adc1();
		//commands_printf("PAS Overshot");
		return;
	} 

	// in pedal modes limit the erpms to equal roughly 120 crank cadence
	// if throttle is pressed ramp the limit rpm
	float pas_max_erpm_limit_start = get_max_pas_erpm();
	float pas_max_erpm = utils_map(app_adc_pwr, 0.03f, 1.0f, pas_max_erpm_limit_start, target_speed_max_erpm);
	utils_truncate_number(&pas_max_erpm, pas_max_erpm_limit_start, target_speed_max_erpm);

	if (mode != EBIKE_MODE_COMPLIANT && mode != EBIKE_MODE_CLASS1) {
		// for the classes where throttle is allowed if the rpm is already above the pas limit and throttle is pushed just ignore it
		pas_max_erpm = (app_adc_pwr > 0.03 && mc_interface_get_rpm() > (pas_max_erpm + 400)) ? target_speed_max_erpm : pas_max_erpm;
	}
	
	if (filtered_pas_max_erpm < 0.0f) {
    	filtered_pas_max_erpm = pas_max_erpm;
	}
	utils_step_towards(&filtered_pas_max_erpm, pas_max_erpm, pas_max_erpm > filtered_pas_max_erpm? 100.0f : 40.0f);
	utils_truncate_number(&target_speed_max_erpm, 1500, filtered_pas_max_erpm);
	set_max_erpm(target_speed_max_erpm);
	//spin_up_if_scheduled(target_speed_max_erpm);
	reattach_pas_if_detached();
	reattach_adc1_if_detached();
}

void reset_pas_max_erpm_filter(void) {
    filtered_pas_max_erpm = -1.0f; 
}

static void reset_pid_speed_ramp(void) {
	pid_ramp_up_step = 0;
	float current_rpm = mc_interface_get_rpm();
	pid_ramp_start_erpm = current_rpm > 900.0f ? current_rpm : 900.0f;
}

static void ramp_and_set_pid_speed(float erpm) {
	if (!walk_pid_speed_set_or_ramping) {
		walk_pid_speed_set_or_ramping = true;
		reset_pid_speed_ramp();
	}
	if (pid_ramp_up_step < WALK_PID_RAMP_UP_CYCLES) {
		pid_ramp_up_step++;
		float ramping_target_erpm = utils_map((float)pid_ramp_up_step, 1.0f, (float)WALK_PID_RAMP_UP_CYCLES, pid_ramp_start_erpm, erpm);
		mc_interface_set_pid_speed(ramping_target_erpm);
	} else {
		mc_interface_set_pid_speed(erpm);
	}
}

static void override_adc_for_class3(bool is_pas_engaged) {
	if (mode == EBIKE_MODE_CLASS3 && is_pas_engaged) {
		float l_erpm_start = mc_interface_get_configuration()->l_erpm_start;
		float speed_start_fade = class12_speed_ms * l_erpm_start;
		float current_speed_ms = mc_interface_get_speed();
		if (current_speed_ms > speed_start_fade) {
			float fade_scaler = utils_map(current_speed_ms, speed_start_fade, class12_speed_ms, 1.0, 0.0);
			utils_truncate_number(&fade_scaler, 0.0, 1.0);
			adc_config config = app_get_configuration()->app_adc_conf;
			float adc_raw_volts = ADC_VOLTS(ADC_IND_EXT);
			float adc_raw_pwr = utils_map(adc_raw_volts, config.voltage_start, config.voltage_end, 0.0, 1.0);
			utils_truncate_number(&adc_raw_pwr, 0.0, 1.0);
			float scaled_adc_pwr = adc_raw_pwr * fade_scaler;
			app_adc_adc1_override(utils_map(scaled_adc_pwr, 0.0, 1.0, config.voltage_start, config.voltage_end));
			if (!override_adc1) {
				override_adc1 = true;
				app_adc_detach_adc(1);
			}
		} else {
			if (override_adc1) {
				override_adc1 = false;
				app_adc_detach_adc(0);
			}
		}
	} else {
		if (override_adc1) {
			override_adc1 = false;
			app_adc_detach_adc(0);
		}
	} 
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;

	chRegSetThreadName("App e-bike");

	is_running = true;

	// Wait for motor config initialization
	chThdSleepMilliseconds(800);

	for(;;) {
		chThdSleepMilliseconds(APP_SLEEP_MS);
		if (stop_now) {
			is_running = false;
			mc_interface_release_motor();
			return;
		}
		timeout_reset();
		if (mode != prev_mode) {
			// reset the limiter on each mode change
			max_erpm = mc_interface_get_configuration()->l_max_erpm;
			reattach_pas_if_detached(); //?
			reattach_adc1_if_detached(); //?
			prev_mode = mode;
		} 

		float app_pas_pwr = app_pas_get_current_target_rel();
		bool pas_engaged = app_pas_is_engaged();
		override_adc_for_class3(pas_engaged);
		float app_adc_pwr = app_adc_get_decoded_level();
		float app_adc_direct_pwr = app_adc_get_direct_decoded_level();
		if (mode == EBIKE_MODE_COMPLIANT || mode == EBIKE_MODE_CLASS1 
			|| mode == EBIKE_MODE_CLASS2 || mode == EBIKE_MODE_CLASS3) {
			bool motor_active = app_pas_pwr > 0.0 || walk_pid_speed_set_or_ramping || (mc_interface_get_rpm() > 100.0f) 
				|| (fabsf(mc_interface_get_tot_current()) > 0.2f); // || is_spinning_up();
			if (pas_engaged && !walk_pid_speed_set_or_ramping) {
				calculate_and_set_pas_max_erpm(app_adc_pwr);
			} else if (app_adc_direct_pwr > 0.03 && (mode == EBIKE_MODE_COMPLIANT || mode == EBIKE_MODE_CLASS1)) {
				float erpm = calculate_target_max_erpm_with_cutoff(THROTTLE_SPEED_MS, CUTOFF_THROTTLE_SPEED_MS);
				if (erpm < ERPM_STOP_THRESHOLD) {
					if (walk_pid_speed_set_or_ramping) {
						// for walk assist just disable motor and inputs
						app_disable_output(DISABLE_APP_OUTPUT_MS);  
						reset_pid_speed_ramp();
						mc_interface_set_current(0.0);
					} else {
						// if user stopped pedaling while holding the throttle engaged detach adc1
						// so when the pas input comes back the adc1 ramps up back again
						detach_adc1();
						// reset the filter so that after the user starts pedaling again the maximum erpm is
						// calculated from base maximum cadence erpm and adc input
						reset_pas_max_erpm_filter();
					}
				} else {
					// engage the walk assist
					app_disable_output(DISABLE_APP_OUTPUT_MS);  
					utils_truncate_number(&erpm, 900, 4000);
					ramp_and_set_pid_speed(erpm);
				}
			} else if (app_adc_direct_pwr > 0.03 && (mc_interface_get_speed() == 0.0 || walk_pid_speed_set_or_ramping) 
						&& (mode == EBIKE_MODE_CLASS2 || mode == EBIKE_MODE_CLASS3)) {
				app_disable_output(DISABLE_APP_OUTPUT_MS);
				ramp_and_set_pid_speed(2500);
			} else if (((app_adc_direct_pwr > 0.03 && motor_active) || (app_adc_direct_pwr > 0.0 && !motor_active)) 
						&& (mode == EBIKE_MODE_CLASS2 || mode == EBIKE_MODE_CLASS3)) {
				float erpm = calculate_target_max_erpm(class12_speed_ms);
				if (erpm < ERPM_STOP_THRESHOLD) {
					//app_disable_output(DISABLE_APP_OUTPUT_MS);
					//mc_interface_release_motor();
					//schedule_spin_up();
					detach_adc1();
					reset_pas_max_erpm_filter();
				} else {
					//spin_up_if_scheduled(erpm);
					set_max_erpm(erpm);
					reattach_adc1_if_detached();
				}
			} else if (was_pid || walk_pid_speed_set_or_ramping) { // || is_spinning_up()) {
				mc_interface_release_motor();
				walk_pid_speed_set_or_ramping = false;
				//cancel_spin_up();
			} else if (adc1_detached) {
				reattach_adc1_if_detached();
			} else {
				filtered_pas_max_erpm = -1.0f;
			}
		} else if (mode == EBIKE_MODE_UNRESTRICTED_PLUS) {
 			if (app_pas_pwr > 0.05 || app_adc_pwr > 0.05) {
				if (was_pid) {
					mc_interface_set_current(last_current);
					was_pid = false;
				}
				last_app_pwr = utils_max_abs(app_pas_pwr, app_adc_pwr);
				last_erpm = mc_interface_get_rpm();
			} else if (last_app_pwr > 0.0) {
				app_disable_output(DISABLE_APP_OUTPUT_MS);
				mc_interface_set_pid_speed(last_erpm + 200);
				was_pid = true;
				release_motor = false;
				last_app_pwr = 0;
			} else if (was_pid) {
				if (release_motor) {
					mc_interface_release_motor();
					was_pid = false;
					release_motor = false;
				} else {
					float curr_erpm = mc_interface_get_rpm();
					last_current = mc_interface_get_tot_current();
					if ((curr_erpm + 400) < last_erpm) {
						if (curr_erpm > 2000.0) {
							last_erpm = curr_erpm;
							app_disable_output(DISABLE_APP_OUTPUT_MS);
							mc_interface_set_pid_speed(curr_erpm + 150);
							was_pid = true;
						} else {
							// stop the CC when erpms drop below 2000
							mc_interface_release_motor();
							was_pid = false;
						}
					} else {
						app_disable_output(DISABLE_APP_OUTPUT_MS);
						mc_interface_set_pid_speed(last_erpm + 150);
						was_pid = true;
					}
				}
			}
		} 
	}
}

float app_custom_max_erpm(void) {
	return max_erpm;
}

// Callback function for the terminal command with arguments.
static void ebike_test(int argc, const char **argv) {
	(void)argc; (void)argv;
	commands_printf("ADC1: %.2f V FSR: %.2f MODE %d ERPM: %.2f PWR: %.2f PID: %d PAS: %.2f",
		(double)ADC_VOLTS(ADC_IND_EXT), (double)filtered_speed_ratio, mode, 
		(double) mc_interface_get_rpm(), (double) last_app_pwr, (double) walk_pid_speed_set_or_ramping,
		(double) app_pas_get_current_target_rel());
}

static void terminal_set_custom_speed(int argc, const char **argv) {
    if (argc == 2) {
		float speed_val_kmh = 0.0;
		if (sscanf(argv[1], "%f", &speed_val_kmh) == 1) {
        
			float speed_val = speed_val_kmh / 3.6f;
			class12_speed_ms = speed_val;

        	eeprom_var v;
        	v.as_float = speed_val;

        	bool success = conf_general_store_eeprom_var_custom(&v, CUSTOM_SPEED_LIMIT_EEPROM_ADDR);

        	if (success) {
            	commands_printf("Success: Custom limit %.2f m/s saved to EEPROM address %d.\n", 
                            	(double) speed_val, CUSTOM_SPEED_LIMIT_EEPROM_ADDR);
        	} else {
            	commands_printf("Error: Failed to write to emulated EEPROM.\n");
        	}
		} else {
            commands_printf("Error: Invalid number format.\n");
        }
    } else {
        commands_printf("Usage: set_custom_speed [value in km/h]\n");
    }
}

float load_my_custom_speed(void) {
    eeprom_var v;
    if (conf_general_read_eeprom_var_custom(&v, CUSTOM_SPEED_LIMIT_EEPROM_ADDR)) {
        return v.as_float;
    }
    return CLASS12_SPEED_MS; // Default fallback if address has never been written to
}