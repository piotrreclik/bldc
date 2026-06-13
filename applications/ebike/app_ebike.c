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
#define MIN_GEAR_RATIO        5.00f    // Smallest mechanical gear ratio (high gear)
#define MAX_GEAR_RATIO        28.00f    // Largest mechanical gear ratio (low/climbing gear)

// --- Velocity Logic Thresholds ---
// 6.0 km/h = 1.67 m/s. The inversion point for the asymmetrical filter behavior.
#define SPEED_THRESHOLD_MS    1.67f    

// --- Filter Smoothing Rates (Alpha Coefficients) ---
#define ALPHA_FAST            0.15f    // High-responsiveness rate (~4 loop iterations to react)
#define ALPHA_SLOW            0.01f    // Heavily dampened rate (~50 loop iterations to smoothly transition)

#define MAX_PAS_SPEED_MS      7.4f
#define MAX_THROTTLE_SPEED_MS 1.25f // 4.5 km/h

#define APP_UPDATE_RATE_HZ 		50
#define APP_SLEEP_MS 			1000 / APP_UPDATE_RATE_HZ
#define DIABLE_APP_OUTPUT_MS 	APP_SLEEP_MS * 2


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
static float conf_max_rpm;
static float last_app_pwr;
static float last_erpm;
static float prev_speed_ms = 0.0f;
static float filtered_gear_ratio = (MIN_GEAR_RATIO + MAX_GEAR_RATIO) / 2;

static float wheel_circumference;
static uint8_t motor_pole_pairs;

static volatile EBIKE_MODE_T mode = EBIKE_MODE_LEGAL;
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
 * Calculates the required Motor ERPM directly from target speed.
 * Uses an Exponential Moving Average (EMA) filter for seamless gear transitions.
 * 
 * @param target_speed_ms   The desired bicycle speed in meters per second (m/s).
 * @return                  The target ERPM to pass directly to mc_interface_set_pid_speed().
 */
float calculate_target_erpm(float target_speed_ms) {
    
    float current_speed_ms = mc_interface_get_speed();
    float current_erpm = mc_interface_get_rpm();
    float current_motor_rpm = current_erpm / motor_pole_pairs;
    float current_wheel_rpm = (current_speed_ms * 60.0f) / wheel_circumference;
    float active_alpha = ALPHA_SLOW; 

    if (target_speed_ms < SPEED_THRESHOLD_MS) {
        // --- LOW SPEED STRATEGY (< 6 km/h) ---
        if (current_speed_ms > target_speed_ms) {
			active_alpha = ALPHA_FAST;
		} else {
            active_alpha = ALPHA_SLOW;  
        }
    } 
    else {
        // --- HIGH SPEED STRATEGY (>= 6 km/h) ---
        if (current_speed_ms > target_speed_ms) {
            active_alpha = ALPHA_SLOW;  
        } else {
            active_alpha = ALPHA_FAST;  
        }
    }

	//advance the alpha after the wheel speed was refreshed
	float speed_diff = fabsf(current_speed_ms - prev_speed_ms);
	if (speed_diff > 0.1f) {
		float exponent = 0.1 / speed_diff;
		active_alpha = powf(active_alpha, exponent);
	}
	prev_speed_ms = current_speed_ms;

    if (current_wheel_rpm > 5.0f && current_motor_rpm > 10.0f) { 
        float raw_gear_ratio = current_motor_rpm / current_wheel_rpm;
        utils_truncate_number(&raw_gear_ratio, MIN_GEAR_RATIO, MAX_GEAR_RATIO);

        filtered_gear_ratio = (active_alpha * raw_gear_ratio) + 
                              ((1.0f - active_alpha) * filtered_gear_ratio);
    }
    utils_truncate_number(&filtered_gear_ratio, MIN_GEAR_RATIO, MAX_GEAR_RATIO);

    float target_wheel_rpm = (target_speed_ms * 60.0f) / wheel_circumference;
    float target_motor_rpm = target_wheel_rpm * filtered_gear_ratio;
    
    return target_motor_rpm * motor_pole_pairs;
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;

	chRegSetThreadName("App Custom");

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
			volatile mc_configuration *mcconf = (volatile mc_configuration*) mc_interface_get_configuration();
			if (prev_mode == EBIKE_MODE_UNDEFINED && mode == EBIKE_MODE_LEGAL) {
				// store the configured max rpm once at startup
				conf_max_rpm = mcconf->l_max_erpm;
				// and calculate some parameters
				wheel_circumference = mcconf->si_wheel_diameter * M_PI;
				if (wheel_circumference <= 0.1f) { 
					wheel_circumference = 2.182f; // Fallback to 27" wheel
				}
				motor_pole_pairs = mcconf->si_motor_poles / 2;
			} else {
				mcconf->l_max_erpm = conf_max_rpm;	
			}
			prev_mode = mode;
		}

		float app_adc_pwr = app_adc_get_decoded_level();
		float app_pas_pwr = app_pas_get_current_target_rel();
		if (mode == EBIKE_MODE_LEGAL || mode == EBIKE_MODE_RPM_LIMIT) {
			if (app_adc_pwr > 0.1) {
				app_disable_output(DIABLE_APP_OUTPUT_MS);
				float erpm = 1400;
				if (mode == EBIKE_MODE_LEGAL) {
					erpm = calculate_target_erpm(MAX_THROTTLE_SPEED_MS);
					utils_truncate_number(&erpm, 1000, 4000);
				}
				mc_interface_set_pid_speed(erpm);
				was_pid = true;
			} else if (app_pas_pwr > 0.0) {
				float max_rpm = 10000;
				if (mode == EBIKE_MODE_LEGAL) {
					max_rpm = calculate_target_erpm(MAX_PAS_SPEED_MS);
					utils_truncate_number(&max_rpm, 2000, 15000);
				}
				volatile mc_configuration *mcconf = (volatile mc_configuration*) mc_interface_get_configuration();
				mcconf->l_max_erpm = max_rpm;
			} else if (was_pid) {
				mc_interface_release_motor();
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
					mc_interface_release_motor();
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

// Callback function for the terminal command with arguments.
static void ebike_test(int argc, const char **argv) {
	(void)argc; (void)argv;
	commands_printf("ADC1: %.2f V GR: %.2f V MODE %d ERPM: %.2f PWR: %.2f",
		(double)ADC_VOLTS(ADC_IND_EXT), (double)filtered_gear_ratio, mode, (double) last_erpm, (double) last_app_pwr);
}