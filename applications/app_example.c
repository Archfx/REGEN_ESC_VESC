//#include "ch.h" // ChibiOS
//#include "hal.h" // ChibiOS HAL
//#include "mc_interface.h" // Motor control functions
//#include "hw.h" // Pin mapping on this hardware
//#include "timeout.h" // To reset the timeout


//Gen code

#include <math.h>
#include "mc_interface.h"
#include "timeout.h"
#include "commands.h" // Terminal print


// Target generator rpm (applies in both directions, always positive)
#define GEN_ERPM		38000.0

// Generator current (amperes) at target rpm (always positive)
#define GEN_CURRENT		  3.0

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


// Example thread
//static THD_FUNCTION(example_thread, arg);
//static THD_WORKING_AREA(example_thread_wa, 2048); // 2kb stack for this thread


//Generator thread
static THD_FUNCTION(gen_thread, arg);
static THD_WORKING_AREA(gen_thread_wa, 1024);

void app_custom_start(void) {
//void app_example_init(void) {
	// Set the UART TX pin as an input with pulldown
	
	//palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLDOWN);
 
	// Start the example thread
	//chThdCreateStatic(example_thread_wa, sizeof(example_thread_wa),
	//	NORMALPRIO, example_thread, NULL);

	//Generator start
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
	commands_printf("Generator app initiated");
	chRegSetThreadName("APP_GEN");
 
	for(;;) {
		// Read the pot value and scale it to a number between 0 and 1 (see hw_46.h)
		float pot = (float)ADC_Value[ADC_IND_EXT];
		float pot2 = (float)ADC_Value[ADC_IND_EXT2];
		pot /= 800.0;
		pot2/= 4095;

		//commands_printf("%.2f", pot);
		//commands_printf("%.2f", pot2);
		//pot /= 4095.0;
 		//mc_interface_set_pid_speed(pot * 10000.0);
		//if (palReadPad(HW_UART_TX_PORT, HW_UART_TX_PIN)) {
			// If the button is pressed, run the motor with speed control
			// proportional to the POT position with a speed between 0 ERPM
			// and 10000 ERPM
		//	mc_interface_set_pid_speed(pot * 10000.0);
		//} else {
			// If the button is not pressed, release the motor
		//	mc_interface_release_motor();
		//}
 
		// Run this loop at 500Hz
		//chThdSleepMilliseconds(2);
 
		// Reset the timeout
		//timeout_reset();

		const float rpm_now = mc_interface_get_rpm();
		const float rpm_rel = fabsf(rpm_now)/GEN_ERPM;
		// Get speed normalized to set rpm
		if (fabsf(rpm_now)>10000)
		{
			const float rpm_rel = 1;
		}
			
		// Start generation at GEN_START * set rpm
		float current = rpm_rel - pot2;   //GEN_START;

		//float current = 1 - GEN_START;
		if (current < 0.0)
			current = 0.0;

		// Reach 100 % of set current at set rpm
		current /= 1.00 - pot2;   //GEN_START;

		current *= pot;//GEN_CURRENT;

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