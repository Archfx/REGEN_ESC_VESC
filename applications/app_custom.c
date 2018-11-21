/*
	Copyright 2017 Arvid Brodin	arvidb@kth.se

	This file is meant to be compiled as part of Benjamin Vedder's
	VESC firmware.

	This file is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This file is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <math.h>
#include "mc_interface.h"
#include "timeout.h"


// Target generator rpm (applies in both directions, always positive)
#define GEN_ERPM		5000.0

// Generator current (amperes) at target rpm (always positive)
#define GEN_CURRENT		  0.2

// At what ratio of GEN_RPM to start generation.
// GEN_RPM = 2000 and GEN_START = 0.90 would start regenerative braking at
// 0.90 * 2000 = 1800 rpm, and will linearly increase current so that
// GEN_CURRENT is reached at GEN_RPM.
// (VESC_Tool limits, i.e. max motor currents & max battery current, will be
// respected.)
#define GEN_START		   0.90

#define GEN_UPDATE_RATE_HZ	1000

static volatile bool stop_now = true;
static volatile bool is_running = false;

// Threads
static THD_FUNCTION(gen_thread, arg);
static THD_WORKING_AREA(gen_thread_wa, 1024);


void app_custom_start(void) {
	commands_printf("  Generator Started    ");
	stop_now = false;
	chThdCreateStatic(gen_thread_wa, sizeof(gen_thread_wa), NORMALPRIO, gen_thread, NULL);
}

void app_custom_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void) conf;
}

static THD_FUNCTION(gen_thread, arg) {
	(void)arg;

	is_running = true;

	for(;;) {
		const float rpm_now = mc_interface_get_rpm();

		// Get speed normalized to set rpm
		const float rpm_rel = fabsf(rpm_now)/GEN_ERPM;
		// Start generation at GEN_START * set rpm
		float current = rpm_rel - GEN_START;
		if (current < 0.0)
			current = 0.0;

		// Reach 100 % of set current at set rpm
		current /= 1.00 - GEN_START;

		current *= GEN_CURRENT;

		if (rpm_now < 0.0) {
			mc_interface_set_current(current);
		} else {
			mc_interface_set_current(-current);
		}


		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / GEN_UPDATE_RATE_HZ;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now) {
			is_running = false;
			return;
		}

		// Reset timeout
		timeout_reset();
	}
}