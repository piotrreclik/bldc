/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se
	Copyright 2020 Marcos Chaparro	mchaparro@powerdesigns.ca

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

#pragma GCC optimize ("Os")

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_math.h"
#include "comm_can.h"
#include "hw.h"
#include <math.h>
#include "terminal.h"
#include "commands.h"

// Settings
#define MAX_MS_WITHOUT_CADENCE_OR_TORQUE	5000
#define MAX_MS_WITHOUT_CADENCE			1000
#define MIN_MS_WITHOUT_POWER			500
#define PULSES_TIME_SAMPLES_COUNT		5
#define ENGAGEMENT_COUNTER_TIMEOUT_SECONDS 0.5F

// Threads
static THD_FUNCTION(pas_thread, arg);
__attribute__((section(".ram4"))) static THD_WORKING_AREA(pas_thread_wa, 512);

// Private variables
static float pedal_rpm_start = 0;
static int32_t engagement_pulses = 0;
static uint32_t rotation_pulses = 0;
static volatile pas_config config;
static volatile float sub_scaling = 1.0;
static volatile float output_current_rel = 0.0;
static volatile float ms_without_power = 0.0;
static volatile float max_pulse_period = 0.0;
static volatile float direction_conf = 0.0;
static volatile float pedal_rpm = 0;
static volatile bool primary_output = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile float torque_ratio = 0.0;

static volatile int32_t pulses_counter = 0;
static volatile systime_t pulse_time_samples[PULSES_TIME_SAMPLES_COUNT] = {0,0,0,0,0};
static volatile uint32_t pulse_time_sample_index = 0;

static void read_isr_count(int argc, const char **argv);

/**
 * Configure and initialize PAS application
 *
 * @param conf
 * App config
 */
void app_pas_configure(pas_config *conf) {
	config = *conf;
	ms_without_power = 0.0;
	output_current_rel = 0.0;

	// a period longer than this should immediately reduce power to zero
	max_pulse_period = 1.0 / ((config.pedal_rpm_start / 60.0) * config.magnets * 4) * 1.2;

	(config.invert_pedal_direction) ? (direction_conf = -1.0) : (direction_conf = 1.0);
	rotation_pulses = config.magnets * 4;

  	pedal_rpm_start = (config.pedal_rpm_start < 1 ? 1 : floor(config.pedal_rpm_start));
  	float engagement_multiplier = config.pedal_rpm_start - pedal_rpm_start;
	int calculated_engagement_pulses = engagement_multiplier * 2 * config.magnets;
	engagement_pulses = calculated_engagement_pulses < 5 ? 5 : calculated_engagement_pulses;
}

/**
 * Start PAS thread
 *
 * @param is_primary_output
 * True when PAS app takes direct control of the current target,
 * false when PAS app shares control with the ADC app for current command
 */
void app_pas_start(bool is_primary_output) {
#ifdef HW_PAS1_PORT
	palSetPadMode(HW_PAS1_PORT, HW_PAS1_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_PAS2_PORT, HW_PAS2_PIN, PAL_MODE_INPUT_PULLUP);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(HW_PAS1_EXTI_PORTSRC, HW_PAS1_EXTI_PINSRC);
	SYSCFG_EXTILineConfig(HW_PAS2_EXTI_PORTSRC, HW_PAS2_EXTI_PINSRC);

	// Configure EXTI Line
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = HW_PAS1_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_InitTypeDef EXTI_InitStructure2;
	EXTI_InitStructure2.EXTI_Line = HW_PAS2_EXTI_LINE;
	EXTI_InitStructure2.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure2.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure2.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure2);

	// Enable and set EXTI Line Interrupt to the highest priority
	nvicEnableVector(EXTI9_5_IRQn, 0);

terminal_register_command_callback(
			"pas",
			"pas",
			0,
			read_isr_count);
#endif
	stop_now = false;
	chThdCreateStatic(pas_thread_wa, sizeof(pas_thread_wa), NORMALPRIO, pas_thread, NULL);

	primary_output = is_primary_output;
}

bool app_pas_is_running(void) {
	return is_running;
}

void app_pas_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}

	if (primary_output == true) {
		mc_interface_set_current_rel(0.0);
	}
	else {
		output_current_rel = 0.0;
	}
}

void app_pas_set_current_sub_scaling(float current_sub_scaling) {
	sub_scaling = current_sub_scaling;
}

float app_pas_get_current_target_rel(void) {
	return output_current_rel;
}

float app_pas_get_pedal_rpm(void) {
	return pedal_rpm;
}

void pas_pin_isr(void) {
	const int8_t counter_lut[32] = {
		//Direction = 0 (backwards)
        0, // 00 to 00
       -1, // 00 to 01
       +1, // 00 to 10
       -2, // 00 to 11

       +1, // 01 to 00
        0, // 01 to 01
       -2, // 01 to 10
       -1, // 01 to 11

       -1, // 10 to 00
       -2, // 10 to 01
        0, // 10 to 10
       +1, // 10 to 11

       -2, // 11 to 00
       +1, // 11 to 01
       -1, // 11 to 10
        0, // 11 to 11

		//Direction = 1 (forwards)
        0, // 00 to 00
       -1, // 00 to 01
       +1, // 00 to 10
       +2, // 00 to 11

       +1, // 01 to 00
        0, // 01 to 01
       +2, // 01 to 10
       -1, // 01 to 11

       -1, // 10 to 00
       +2, // 10 to 01
        0, // 10 to 10
       +1, // 10 to 11

       +2, // 11 to 00
       +1, // 11 to 01
       -1, // 11 to 10
        0, // 11 to 11
	};
	static uint8_t direction_isr = 0;
	static uint8_t lut_index = 16;
    lut_index |= palReadPad(HW_PAS1_PORT, HW_PAS1_PIN)<<1 | palReadPad(HW_PAS2_PORT, HW_PAS2_PIN);
	int8_t lut_value = counter_lut[lut_index];
    pulses_counter += direction_conf * lut_value;

    if (lut_value != 0) {
        direction_isr = (lut_value > 0) ? 1 : 0;
		direction_isr *= direction_conf;
		if (direction_isr > 0) {
			pulse_time_samples[pulse_time_sample_index++ % PULSES_TIME_SAMPLES_COUNT] = chVTGetSystemTimeX();
		}
    }
    //Prepare for next iteration by shifting current state
    //bits to old state bits and also the direction bit
    lut_index = ((lut_index << 2) & 0b1100) | (direction_isr<<4);
}

static void read_isr_count(int argc, const char **argv) {
	(void)argc; (void)argv;
	commands_printf("Pulses %d, rpm %f, cranks %d", pulses_counter, (double)pedal_rpm, 
		pulse_time_sample_index / rotation_pulses / 2);
}

void caclulate_pas_rpm(void) {
#ifdef HW_PAS1_PORT
	static float inactivity_time = 0;
	static float engagement_inactivity_time = 0;
	static int32_t engagement_counter = 0;
	static int32_t old_pulses_counter = 0;
	float tracked_pedal_rpm = 0;

	int32_t counter_diff = pulses_counter - old_pulses_counter;
	old_pulses_counter = pulses_counter;
    engagement_counter = engagement_counter + counter_diff > 0 ? engagement_counter + counter_diff : 0;
	if (counter_diff == 0) {
		engagement_inactivity_time += 1.0 / (float)config.update_rate_hz;

		//if no pedal activity, reset engagement counter
		if(engagement_inactivity_time > ENGAGEMENT_COUNTER_TIMEOUT_SECONDS) {
			engagement_counter = 0;
		}
	} else {
		engagement_inactivity_time = 0.0;
	}

	if (counter_diff > 0) {
		uint32_t pulses_ticks_sum = 0; 
		for (uint32_t pi = pulse_time_sample_index - 1; pi > pulse_time_sample_index - PULSES_TIME_SAMPLES_COUNT; pi--) {
			pulses_ticks_sum += pulse_time_samples[pi % PULSES_TIME_SAMPLES_COUNT] - pulse_time_samples[(pi - 1) % PULSES_TIME_SAMPLES_COUNT];
		}
		float average_pulse_seconds = (float)(pulses_ticks_sum / (float)(PULSES_TIME_SAMPLES_COUNT - 1)) /(float)CH_CFG_ST_FREQUENCY;
		tracked_pedal_rpm =  60.0 / (float)(rotation_pulses * average_pulse_seconds);
	}
	
	if ((pedal_rpm > pedal_rpm_start && counter_diff > 0)
		|| (tracked_pedal_rpm > pedal_rpm_start && engagement_counter > engagement_pulses)) {
		UTILS_LP_FAST(pedal_rpm, tracked_pedal_rpm, pedal_rpm > pedal_rpm_start ? 0.6 : 1.0);
		engagement_counter = 0;
		inactivity_time = 0;
	} else {
		inactivity_time += 1.0 / (float)config.update_rate_hz;

		//if no pedal activity, set RPM as zero
		if(inactivity_time > max_pulse_period) {
			pedal_rpm = 0.0;
		}
	}
#endif
}

static THD_FUNCTION(pas_thread, arg) {
	(void)arg;

	float output = 0;
	chRegSetThreadName("APP_PAS");

#ifdef HW_PAS1_PORT
	palSetPadMode(HW_PAS1_PORT, HW_PAS1_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_PAS2_PORT, HW_PAS2_PIN, PAL_MODE_INPUT_PULLUP);
#endif

	is_running = true;

	for(;;) {
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now) {
			is_running = false;
			return;
		}

		caclulate_pas_rpm();

		// For safe start when fault codes occur
		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			ms_without_power = 0;
		}

		if (app_is_output_disabled()) {
			continue;
		}

		switch (config.ctrl_type) {
			case PAS_CTRL_TYPE_NONE:
				output = 0.0;
				break;
			case PAS_CTRL_TYPE_CADENCE:
				// Map pedal rpm to assist level

				// NOTE: If the limits are the same a numerical instability is approached, so in that case
				// just use on/off control (which is what setting the limits to the same value essentially means).
				if (config.pedal_rpm_end > (pedal_rpm_start + 1.0)) {
					float total_current_scaling = config.current_scaling * sub_scaling;
					output = pedal_rpm > pedal_rpm_start ? 
						utils_map(pedal_rpm, pedal_rpm_start, config.pedal_rpm_end, total_current_scaling / 3.0, total_current_scaling) : 
						0.0;
					utils_truncate_number(&output, 0.0, total_current_scaling);
				} else {
					if (pedal_rpm > config.pedal_rpm_end) {
						output = config.current_scaling * sub_scaling;
					} else {
						output = 0.0;
					}
				}
				break;

#ifdef HW_HAS_PAS_TORQUE_SENSOR
			case PAS_CTRL_TYPE_TORQUE:
			{
				torque_ratio = hw_get_PAS_torque();
				output = torque_ratio * config.current_scaling * sub_scaling;
				utils_truncate_number(&output, 0.0, config.current_scaling * sub_scaling);
			}
			/* fall through */
			case PAS_CTRL_TYPE_TORQUE_WITH_CADENCE_TIMEOUT:
			{
				// disable assistance if torque has been sensed for >5sec without any pedal movement. Prevents
				// motor overtemps when the rider is just resting on the pedals
				static float ms_without_cadence_or_torque = 0.0;
				if(output == 0.0 || pedal_rpm > 0) {
					ms_without_cadence_or_torque = 0.0;
				} else {
					ms_without_cadence_or_torque += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
					if(ms_without_cadence_or_torque > MAX_MS_WITHOUT_CADENCE_OR_TORQUE) {
						output = 0.0;
					}
				}
				// if cranks are not moving, there should not be any output. This covers the case of a torque sensor
				// stuck with a non-zero signal.
				static float ms_without_cadence = 0.0;
				if(pedal_rpm < 0.01) {
					ms_without_cadence += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
					if(ms_without_cadence > MAX_MS_WITHOUT_CADENCE) {
						output = 0.0;
					}
				} else {
					ms_without_cadence = 0.0;
				}
			}
#endif
			default:
				break;
		}

		// Apply ramping
		static systime_t last_time = 0;
		static float output_ramp = 0.0;
		float ramp_time = fabsf(output) > fabsf(output_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		if (ramp_time > 0.01) {
			const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
			utils_step_towards(&output_ramp, output, ramp_step);
			utils_truncate_number(&output_ramp, 0.0, config.current_scaling * sub_scaling);

			last_time = chVTGetSystemTimeX();
			output = output_ramp;
		}

		if (output < 0.001) {
			ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
		}

		// Safe start is enabled if the output has not been zero for long enough
		if (ms_without_power < MIN_MS_WITHOUT_POWER) {
			static int pulses_without_power_before = 0;
			if (ms_without_power == pulses_without_power_before) {
				ms_without_power = 0;
			}
			pulses_without_power_before = ms_without_power;
			output_current_rel = 0.0;
			continue;
		}

		// Reset timeout
		timeout_reset();

		if (primary_output == true) {
			mc_interface_set_current_rel(output);
		}
		else {
			output_current_rel = output;
		}
	}
}
