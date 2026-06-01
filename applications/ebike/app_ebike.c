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

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private functions
static void ebike_test(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static float conf_max_rpm;

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
	conf_max_rpm = mc_interface_get_configuration()->l_max_erpm;
}

void app_ebike_set_mode(EBIKE_MODE_T new_mode) {
	mode = new_mode;
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;

	chRegSetThreadName("App Custom");

	is_running = true;

	// Example of using the experiment plot
//	chThdSleepMilliseconds(8000);
//	commands_init_plot("Sample", "Voltage");
//	commands_plot_add_graph("Temp Fet");
//	commands_plot_add_graph("Input Voltage");
//	float samp = 0.0;
//
//	for(;;) {
//		commands_plot_set_graph(0);
//		commands_send_plot_points(samp, mc_interface_temp_fet_filtered());
//		commands_plot_set_graph(1);
//		commands_send_plot_points(samp, GET_INPUT_VOLTAGE());
//		samp++;
//		chThdSleepMilliseconds(10);
//	}

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.

		// Run your logic here. A lot of functionality is available in mc_interface.h.
		float app_adc_pwr = 0.0;
		app_adc_pwr = app_adc_get_decoded_level();
		if (mode == EBIKE_MODE_LEGAL && app_adc_pwr > 0.1) {
			app_disable_output(50);
			mc_interface_set_pid_speed(2000);
			timeout_reset();
		}
		if (mode != prev_mode) {
			volatile mc_configuration *mcconf = (volatile mc_configuration*) mc_interface_get_configuration();
			if (mode == EBIKE_MODE_LEGAL) {
				mcconf->l_max_erpm = 9000;
			} else {
				mcconf->l_max_erpm = conf_max_rpm;
			}
			//mc_interface_set_configuration(mcconf);
		}
		prev_mode = mode;

		chThdSleepMilliseconds(20);
	}
}

// Callback function for the terminal command with arguments.
static void ebike_test(int argc, const char **argv) {
	(void)argc; (void)argv;
	//if (argc == 2) {
		//int d = -1;
		//sscanf(argv[1], "%d", &d);

		//commands_printf("You have entered %d", d);

		// For example, read the ADC inputs on the COMM header.
		commands_printf("ADC1: %.2f V ADC2: %.2f V MODE %d",
				(double)ADC_VOLTS(ADC_IND_EXT), (double)ADC_VOLTS(ADC_IND_EXT2), mode);
	//} else {
		//commands_printf("This command requires one argument.\n");
	//}
}