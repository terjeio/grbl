/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2017 Terje Io
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"
// #include "noeeprom.h" // uncomment to enable EEPROM emulation

// Declare system global variable structure
system_t sys;
int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.

HAL hal;

int grbl_enter (void)
{

	bool looping = true;

	memset(&hal, 0, sizeof(HAL));  // Clear...

	hal.version = 2; // Update when signatures and/or contract is changed - hal_init() should fail

	driver_init();

#ifdef __noeeprom_h__
	if(!hal.hasEEPROM && (hal.hasEEPROM = noeeprom_init())) {
	    hal.eeprom_get_char = &noeeprom_get_char;
	    hal.eeprom_put_char = &noeeprom_put_char;
	    hal.memcpy_to_eeprom_with_checksum = &memcpy_to_noeeprom_with_checksum;
	    hal.memcpy_from_eeprom_with_checksum = &memcpy_from_noeeprom_with_checksum;
	    settings_restore(SETTINGS_RESTORE_ALL);
	}
#endif

	hal.limit_interrupt_callback = &limit_interrupt_handler;
	hal.control_interrupt_callback = &control_interrupt_handler;
	hal.stepper_interrupt_callback = &stepper_driver_interrupt_handler;
	hal.protocol_process_realtime = &protocol_process_realtime;
	hal.protocol_enqueue_gcode = &protocol_enqueue_gcode;

	settings_init(); // Load Grbl settings from EEPROM

	memset(sys_position,0,sizeof(sys_position)); // Clear machine position.

	hal.initMCU();

	if(hal.get_position)
	    hal.get_position(&sys_position); // TODO:  restore on abort when returns true?

    // Initialize system state.
    #ifdef FORCE_INITIALIZATION_ALARM
      // Force Grbl into an ALARM state upon a power-cycle or hard reset.
      sys.state = STATE_ALARM;
    #else
      sys.state = STATE_IDLE;
    #endif

    // Check for power-up and set system alarm if homing is enabled to force homing cycle
    // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
    // startup scripts, but allows access to settings and internal commands. Only a homing
    // cycle '$H' or kill alarm locks '$X' will disable the alarm.
    // NOTE: The startup script will run after successful completion of the homing cycle, but
    // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
    // things uncontrollably. Very bad.
    #ifdef HOMING_INIT_LOCK
      if (settings.flags.homing_enable) { sys.state = STATE_ALARM; }
    #endif

    // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
    // will return to this loop to be cleanly re-initialized.
    while(looping) {

      // Reset system variables.
      uint8_t prior_state = sys.state;
      memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
      sys.state = prior_state;
      sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
      sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
      sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%
  		memset(sys_probe_position,0,sizeof(sys_probe_position)); // Clear probe position.
      sys_probe_state = 0;
      sys_rt_exec_state = 0;
      sys_rt_exec_alarm = 0;
      flush_override_buffers();

      // Reset Grbl primary systems.
      serial_reset_read_buffer(); // Clear serial read buffer
      gc_init(); // Set g-code parser to default state
      limits_init();
      plan_reset(); // Clear block buffer and planner variables
      st_reset(); // Clear stepper subsystem variables.

      // Sync cleared gcode and planner positions to current system position.
      plan_sync_position();
      gc_sync_position();

      // Print welcome message. Indicates an initialization has occured at power-up or with a reset.
      report_init_message();

      // Start Grbl main loop. Processes program inputs and executes them.
      if(!(looping = protocol_main_loop()))
          looping = hal.releaseMCU == 0 || hal.releaseMCU();

    }

    return 0;
}
