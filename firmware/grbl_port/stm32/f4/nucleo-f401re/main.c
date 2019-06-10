/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of grbl_port_opencm3 project, derived from the Grbl work.

  Copyright (c) 2017 Angelo Di Chello
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  
  Grbl_port_opencm3 is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl_port_opencm3 is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl_port_opencm3.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"


// Declare system global variable structure
system_t sys; 


int main(void)
{
#ifdef NUCLEO_F401
#ifndef BASIC_CPU_SPEED
	const struct rcc_clock_scale rcc_hse_8mhz_3v3_no_prescalers =
	{ /* 48MHz */
		.pllm = 8,
		.plln = 96,
		.pllp = 2,
		.pllq = 2,
		.hpre = RCC_CFGR_HPRE_DIV_NONE,
		.ppre1 = RCC_CFGR_PPRE_DIV_NONE,
		.ppre2 = RCC_CFGR_PPRE_DIV_NONE,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_ICEN | FLASH_ACR_DCEN |
				FLASH_ACR_LATENCY_3WS,
		.ahb_frequency  = 48000000,
		.apb1_frequency = 48000000,
		.apb2_frequency = 48000000
	};
//  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
	rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3_no_prescalers);
#endif
#endif
  // Initialize system upon power-up.
  serial_init();   // Setup serial baud rate and interrupts
  settings_init(); // Load Grbl settings from EEPROM
  stepper_init();  // Configure stepper pins and interrupt timers
  system_init();   // Configure pinout pins and pin-change interrupt

  memset(&sys, 0, sizeof(system_t));  // Clear all system variables
  sys.abort = true;   // Set abort to complete initialization
#ifdef NUCLEO_F401
  __enable_irq(); // Global enable interrupts
#else
  sei(); // Enable interrupts
#endif

  // Check for power-up and set system alarm if homing is enabled to force homing cycle
  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
  // startup scripts, but allows access to settings and internal commands. Only a homing
  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
  // NOTE: The startup script will run after successful completion of the homing cycle, but
  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
  // things uncontrollably. Very bad.
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif
  
  // Force Grbl into an ALARM state upon a power-cycle or hard reset.
  #ifdef FORCE_INITIALIZATION_ALARM
    sys.state = STATE_ALARM;
  #endif
  
  // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
  // will return to this loop to be cleanly re-initialized.
  for(;;) {

    // TODO: Separate configure task that require interrupts to be disabled, especially upon
    // a system abort and ensuring any active interrupts are cleanly reset.
  
    // Reset Grbl primary systems.
    serial_reset_read_buffer(); // Clear serial read buffer
    gc_init(); // Set g-code parser to default state
    spindle_init();
    coolant_init();
    limits_init(); 
    probe_init();
    plan_reset(); // Clear block buffer and planner variables
    st_reset(); // Clear stepper subsystem variables.

#ifdef VARIABLE_SPINDLE
    /* Check if a minimal pwm needs to be given to the spindle at startup
       As an example, this is useful when using something like an ESC to
       control the spindle motor and the ESC needs a small input PWM to be
       activated. */
    if(settings.spindle_pwm_enable_at_start)
    {
    	// Enable Clockwise spindle with minimal pwm
    	gc_state.modal.spindle = SPINDLE_ENABLE_CW;
    	spindle_set_state(SPINDLE_ENABLE_CW, 0);
    }
#endif

    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();

    // Reset system variables.
    sys.abort = false;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys.suspend = false;
    sys.soft_limit = false;
              
    // Start Grbl main loop. Processes program inputs and executes them.
    protocol_main_loop();
    
  }
  return 0;   /* Never reached */
}
