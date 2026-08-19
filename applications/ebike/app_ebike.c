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

#define PAS_SPEED_MS      		 6.94f // 25 km/h
#define CUTOFF_PAS_SPEED_MS   	 PAS_SPEED_MS * 1.06f  // 6% more than the limit
#define CLASS12_SPEED_MS      8.89f // 32 km/h
#define CUTOFF_CLASS12_SPEED_MS   	 CLASS12_SPEED_MS * 1.06f  // 6% more than the limit
#define CLASS3_PAS_SPEED_MS      12.52f // 45 km/h
#define CUTOFF_CLASS3_PAS_SPEED_MS   	 CLASS3_PAS_SPEED_MS * 1.06f  // 6% more than the limit
#define THROTTLE_SPEED_MS 		 1.11f // 4 km/h
#define CUTOFF_THROTTLE_SPEED_MS 1.67f // 6 km/h

#define APP_UPDATE_RATE_HZ 		50
#define APP_SLEEP_MS 			1000 / APP_UPDATE_RATE_HZ
#define DIABLE_APP_OUTPUT_MS 	APP_SLEEP_MS * 2
#define APP_UPDATE_LOOP_DT      (1.0f / (float)APP_UPDATE_RATE_HZ)

// Soft-start tuning parameters for the Bafang BBSHD motor
#define WALK_START_CURRENT_TARGET  15.0f   // Gentle current (A) to silently eliminate gear backlash
#define WALK_CURRENT_RAMP_RATE     5.0f  // Current ramp rate (Amperes per second)

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private functions
static void ebike_test(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile bool was_pid = false;
static volatile bool release_motor = false;
static volatile float last_app_pwr;
static volatile float last_erpm;
static volatile float last_current = 0.0f;
static volatile float walk_current_ramp = 0.0f;
static volatile bool pid_speed_set_or_ramping = false;
static volatile float max_erpm = 100000.0f;

static float filtered_speed_ratio = (MIN_SPEED_RATIO + MAX_SPEED_RATIO) / 2;
static float smoothed_speed_ratio = (MIN_SPEED_RATIO + MAX_SPEED_RATIO) / 2;

static volatile EBIKE_MODE_T mode = EBIKE_MODE_COMPLIANT;
static volatile EBIKE_MODE_T prev_mode = EBIKE_MODE_UNDEFINED;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

	app_adc_start(false);
	app_pas_start(false);
	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"ebike",
			"ebike",
			0,
			ebike_test);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	terminal_unregister_callback(ebike_test);

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

void app_custom_release_motor(void) {
	release_motor = true;
}

/**
 * @brief Unified Speed Ratio Control Loop (50Hz)
 * Optimizes motor target ERPM by tracking dimensionless speed ratios
 * while mitigating sensor dead-time lag and avoiding pedal-past-target drift.
 */
float calculate_target_erpm(float target_speed_ms, float cutoff_speed_ms) {
    // Persistent state tracking across 50Hz execution frames
    static float prev_speed_ms = 0.0f;
    static int spin_up_blanking_cycles = 0; 
    static float erpm_sum = 0.0f;       
    static int erpm_sample_count = 0;   

    // --- 1. DATA ACQUISITION ---
    const float current_erpm    = mc_interface_get_rpm();
    const float current_speed   = mc_interface_get_speed();
    const float current_current = mc_interface_get_tot_current();
    const float speed_diff      = fabsf(current_speed - prev_speed_ms);

    if (spin_up_blanking_cycles > 0) {
        spin_up_blanking_cycles--;
    }

    // --- 2. ENVIRONMENT STATE ANALYSIS ---
    const float estimated_speed    = current_erpm / filtered_speed_ratio;
    const bool overshot_target      = (estimated_speed > (target_speed_ms * 1.08f)) || (current_speed > (target_speed_ms * 1.08f));
    const bool undershot_target     = (estimated_speed < (target_speed_ms * 0.98f)) && (current_speed < (target_speed_ms * 0.98f));
    const bool overshot_cutoff      = current_speed > cutoff_speed_ms;
    const bool first_speed_redout   = (prev_speed_ms == 0.0f);
    const bool motor_slowing_down   = (fabsf(last_erpm) + 5.0f > fabsf(current_erpm)) && (current_current < 0.05f);
    const bool sensor_pulse_arrived = speed_diff > 0.0001f;
    
    // Dynamic free-wheeling / no-load tracking based on the previous target ERPM loop output
    const float last_target_erpm = target_speed_ms * smoothed_speed_ratio;
    const bool motor_at_no_load_target = (fabsf(current_erpm - last_target_erpm) < (last_target_erpm * 0.05f)) 
                                         && (current_current < 0.8f);

    // --- 3. DYNAMIC FILTER COEFFICIENTS (ALPHA) ---
    float active_alpha = (target_speed_ms < SPEED_THRESHOLD_MS) ? ALPHA_SLOW : ALPHA_FAST;

    // Proportional Alpha Advance Model
    const float speed_error_pct = fabsf(target_speed_ms - current_speed) / target_speed_ms;
    const float dynamic_trigger = speed_diff + speed_error_pct;
    
    if (dynamic_trigger > 0.2f && !first_speed_redout) {
        const float exponent = 0.2f / dynamic_trigger;
        active_alpha = powf(active_alpha, exponent);
        utils_truncate_number(&active_alpha, ALPHA_SLOW, 0.8f); // Secure clamp replacing unstable inline fminf
    }

    // --- 4. HARDWARE EVENT TRIGGERS ---
    // Flush historical data on exact motor startup frame to prevent dirty mean estimations
    if (fabsf(last_erpm) < 500.0f && fabsf(current_erpm) >= 500.0f) {
        erpm_sum = 0.0f;
        erpm_sample_count = 0;
        spin_up_blanking_cycles = 8;
    }

    // --- 5. RATIO PROCESSING PIPELINE ---
    if (current_speed > 0.05f && fabsf(current_erpm) > 500.0f) {
        
        const bool low_speed_domain  = (target_speed_ms < SPEED_THRESHOLD_MS);
        const bool can_update_filter = sensor_pulse_arrived && (spin_up_blanking_cycles == 0) 
                                       && !motor_slowing_down && (!motor_at_no_load_target || low_speed_domain);
        
        if (can_update_filter) {
            // Compute true mathematical average of ERPM during this single wheel rotation window
            const float exact_mean_erpm = (erpm_sample_count > 0) ? (erpm_sum / (float)erpm_sample_count) : current_erpm;
            float raw_speed_ratio = exact_mean_erpm / current_speed;
            
            utils_truncate_number(&raw_speed_ratio, MIN_SPEED_RATIO, MAX_SPEED_RATIO);
            UTILS_LP_FAST(filtered_speed_ratio, raw_speed_ratio, active_alpha);
            
            //commands_printf("Ratio Update! Filtered: %0.2f, CC: %0.2f, Mean: %0.1f, Alpha: %0.2f", 
                            //(double)filtered_speed_ratio, (double)current_current, (double)exact_mean_erpm, (double)active_alpha);

            // Flush integration pipeline for the next hardware window
            erpm_sum = 0.0f;
            erpm_sample_count = 0;
            
        } 
        // Flush pipeline while system states are transitioning or free-wheeling without a downshift event
        else if (spin_up_blanking_cycles > 0 || motor_slowing_down || (motor_at_no_load_target && !undershot_target)) {
            erpm_sum = 0.0f;
            erpm_sample_count = 0;
            
        } else {
            // Safely buffer dead-time samples and run localized micro-adjustments inside the sensor blind zone
            erpm_sum += current_erpm;
            erpm_sample_count++;
            
            // Localized micro adjustment downwards
            if (overshot_target && current_current > 1.0f && (!motor_at_no_load_target || low_speed_domain)) {
                filtered_speed_ratio -= target_speed_ms;
                //commands_printf("Ratio Auto-Trim [-%0.1f]: %0.2f, CC: %0.2f", (double)target_speed_ms, (double)filtered_speed_ratio, (double)current_current);
            } 
            // Localized high-speed emergency micro adjustment upwards (Locked out in low speed domain by application design)
            else if (undershot_target && motor_at_no_load_target && target_speed_ms > SPEED_THRESHOLD_MS) {
                filtered_speed_ratio += (target_speed_ms * 5.0f);
                //commands_printf("Ratio Auto-Trim [+%0.1f]: %0.2f, CC: %0.2f", (double)(target_speed_ms * 5.0f), (double)filtered_speed_ratio, (double)current_current);
            }
        }
    } 
    // --- 6. CRITICAL ASYNC SAFETY BOUNDARIES ---
    else if (overshot_cutoff && last_current > 1.5f) {
        // Soft launch preparation: smoothly drop ratio during power-off coasting past cut-off speed
        filtered_speed_ratio -= 80.0f; 
        //commands_printf("Soft Launch Prep [-80.0]: %0.2f", (double)filtered_speed_ratio);
        erpm_sum = 0.0f;
        erpm_sample_count = 0;    
    } 
    else if (current_speed < 0.05f && fabsf(current_erpm) > 500.0f) {
        // Hard standstill anchor to handle zero speed transitions cleanly
        filtered_speed_ratio = 2.0f * MIN_SPEED_RATIO; 
        smoothed_speed_ratio = MIN_SPEED_RATIO; 
        erpm_sum = 0.0f;
        erpm_sample_count = 0;    
    }
    
    // --- 7. SIGNAL SMOOTHING & OUTPUT GENERATION ---
    utils_truncate_number(&filtered_speed_ratio, MIN_SPEED_RATIO, MAX_SPEED_RATIO);
    UTILS_LP_FAST(smoothed_speed_ratio, filtered_speed_ratio, 0.05f);

    // Commit telemetry states for the next 50Hz frame calculation
    prev_speed_ms = current_speed;
    last_erpm     = current_erpm;
    last_current  = current_current;

    // Output final target ERPM to the built-in VESC speed loop structure
    return overshot_cutoff ? 0.0f : (target_speed_ms * smoothed_speed_ratio);
}

void set_max_erpm(float lo_max_erpm) {
	utils_truncate_number(&lo_max_erpm, 1500, mc_interface_get_configuration()->l_max_erpm);
	max_erpm = lo_max_erpm;
}

void calculate_and_set_pas_max_erpm(void) {
	float lo_max_erpm = 10000;
	if (mode == EBIKE_MODE_COMPLIANT) {
		lo_max_erpm = calculate_target_erpm(PAS_SPEED_MS, CUTOFF_PAS_SPEED_MS);
	} else if (mode == EBIKE_MODE_CLASS1 || mode == EBIKE_MODE_CLASS2) {
		lo_max_erpm = calculate_target_erpm(CLASS12_SPEED_MS, CUTOFF_CLASS12_SPEED_MS);
	} else if (mode == EBIKE_MODE_CLASS3) {
		lo_max_erpm = calculate_target_erpm(CLASS3_PAS_SPEED_MS, CUTOFF_CLASS3_PAS_SPEED_MS);
	}
	if (lo_max_erpm < ERPM_STOP_THRESHOLD) {
		mc_interface_release_motor();
		app_disable_output(DIABLE_APP_OUTPUT_MS);
		return;
	}
	// in pedal modes limit the erpms to equal roughly 150 crank cadence
	utils_truncate_number(&lo_max_erpm, 1500, 15000);
	set_max_erpm(lo_max_erpm);
}

static void ramp_current_and_set_pid_speed(float erpm) {
	pid_speed_set_or_ramping = true;
	float current_erpm = mc_interface_get_rpm();
	if (!was_pid && (current_erpm < (erpm * 0.8))) {
		if (walk_current_ramp < WALK_START_CURRENT_TARGET) {
			walk_current_ramp += WALK_CURRENT_RAMP_RATE * APP_UPDATE_LOOP_DT;
		} else {
			walk_current_ramp = WALK_START_CURRENT_TARGET;
		}
		mc_interface_set_current(walk_current_ramp);
		was_pid = false; 
	} else {
		mc_interface_set_pid_speed(erpm);
		was_pid = true;
	}
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;

	chRegSetThreadName("App e-bike");

	is_running = true;

	// Wait for motor config initialization
	chThdSleepMilliseconds(500);

	for(;;) {
		chThdSleepMilliseconds(APP_SLEEP_MS);
		if (stop_now) {
			is_running = false;
			return;
		}

		timeout_reset();

		if (mode != prev_mode) {
			// reset the limiter on each mode change
			max_erpm = mc_interface_get_configuration()->l_max_erpm;
			prev_mode = mode;
		} 

		float app_adc_pwr = app_adc_get_decoded_level();
		float app_pas_pwr = app_pas_get_current_target_rel();
		if (mode == EBIKE_MODE_COMPLIANT || mode == EBIKE_MODE_CLASS1 
			|| mode == EBIKE_MODE_CLASS2 || mode == EBIKE_MODE_CLASS3) {
			if (app_pas_pwr > 0.0 && !pid_speed_set_or_ramping) {
				calculate_and_set_pas_max_erpm();
			} else if (app_adc_pwr > 0.1) {
				float erpm = 2200;
				if (mode == EBIKE_MODE_COMPLIANT || mode == EBIKE_MODE_CLASS1) {
					app_disable_output(DIABLE_APP_OUTPUT_MS);
					erpm = calculate_target_erpm(THROTTLE_SPEED_MS, CUTOFF_THROTTLE_SPEED_MS);
					if (erpm < ERPM_STOP_THRESHOLD) {
						mc_interface_release_motor();
						continue;
					}
					utils_truncate_number(&erpm, 800, 4000);
					ramp_current_and_set_pid_speed(erpm);
				} else if (mode == EBIKE_MODE_CLASS2 || mode == EBIKE_MODE_CLASS3) {
					erpm = calculate_target_erpm(CLASS12_SPEED_MS, CUTOFF_CLASS12_SPEED_MS);
					float current_speed = mc_interface_get_speed();
					if (current_speed == 0.0 || pid_speed_set_or_ramping) {
						app_disable_output(DIABLE_APP_OUTPUT_MS);
						ramp_current_and_set_pid_speed(2200);
					} else {
						if (erpm < ERPM_STOP_THRESHOLD) {
							app_disable_output(DIABLE_APP_OUTPUT_MS);
							mc_interface_release_motor();
						}
						set_max_erpm(erpm);
					}
				}
			} else if (was_pid || pid_speed_set_or_ramping) {
				mc_interface_release_motor();
				walk_current_ramp = 0.0f;
				pid_speed_set_or_ramping = false;
				was_pid = false;
			} else if (app_adc_pwr > 0.0) {
				// kind of a safety feature. On my bikes after motor engages the ADC voltage 
				// tends to go up and not release the motor completly when the throttle is released
				mc_interface_set_current_rel(0.0);
				app_disable_output(DIABLE_APP_OUTPUT_MS);
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
				app_disable_output(DIABLE_APP_OUTPUT_MS);
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
							app_disable_output(DIABLE_APP_OUTPUT_MS);
							mc_interface_set_pid_speed(curr_erpm + 150);
							was_pid = true;
						} else {
							// stop the CC when erpms drop below 2000
							mc_interface_release_motor();
							was_pid = false;
						}
					} else {
						app_disable_output(DIABLE_APP_OUTPUT_MS);
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
	commands_printf("ADC1: %.2f V SR: %.2f MODE %d ERPM: %.2f PWR: %.2f",
		(double)ADC_VOLTS(ADC_IND_EXT), (double)filtered_speed_ratio, mode, (double) mc_interface_get_rpm(), (double) last_app_pwr);
}