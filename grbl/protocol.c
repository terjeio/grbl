/*
  protocol.c - controls Grbl execution protocol and procedures
  Part of Grbl

  Copyright (c) 2017 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
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

// Define line flags. Includes comment type tracking and line overflow detection.
typedef union {
    uint8_t value;
    struct {
        uint8_t overflow            :1,
                comment_parentheses :1,
                comment_semicolon   :1,
                block_delete        :1;
    };
} line_flags_t;

static uint8_t char_counter = 0;
static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.
static char xcommand[LINE_BUFFER_SIZE];

static void protocol_exec_rt_suspend();

// add gcode to execute not originating from serial stream
bool protocol_enqueue_gcode (char *gcode)
{

	bool ok = xcommand[0] == '\0' && (sys.state == STATE_IDLE || sys.state == STATE_JOG) && bit_isfalse(sys_rt_exec_state, EXEC_MOTION_CANCEL);

	if(ok)
		strcpy(xcommand, gcode);

	return ok;
}

/*
  GRBL PRIMARY LOOP:
*/
bool protocol_main_loop()
{
    // Perform some machine checks to make sure everything is good to go.
  #ifdef CHECK_LIMITS_AT_INIT
    if (settings.flags.hard_limit_enable && limits_get_state().value) {
        sys.state = STATE_ALARM; // Ensure alarm state is active.
        report_feedback_message(Message_CheckLimits);
    }
  #endif

    // Check for and report alarm state after a reset, error, or an initial power up.
    // NOTE: Sleep mode disables the stepper drivers and position can't be guaranteed.
    // Re-initialize the sleep state as an ALARM mode to ensure user homes or acknowledges.
    if (sys.state & (STATE_ALARM | STATE_SLEEP)) {
        report_feedback_message(Message_AlarmLock);
        sys.state = STATE_ALARM; // Ensure alarm state is set.
    } else {
        // Check if the safety door is open.
        sys.state = STATE_IDLE;
        if (system_check_safety_door_ajar()) {
            bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
            protocol_execute_realtime(); // Enter safety door mode. Should return as IDLE state.
        }
        // All systems go!
        system_execute_startup(line); // Execute startup script.
    }

    // ---------------------------------------------------------------------------------
    // Primary loop! Upon a system abort, this exits back to main() to reset the system.
    // This is also where Grbl idles while waiting for something to do.
    // ---------------------------------------------------------------------------------

    int32_t c;
    line_flags_t line_flags = {0};
    status_code_t rstatus;

    xcommand[0] = '\0';

    for (;;) {

        // Process one line of incoming serial data, as the data becomes available. Performs an
        // initial filtering by removing spaces and comments and capitalizing all letters.
        while((c = serial_read()) != SERIAL_NO_DATA) {

            if(c == CMD_RESET) {

                char_counter = 0;
                xcommand[0] = '\0';
                if (sys.state == STATE_JOG)// Block all other states from invoking motion cancel.
                    system_set_exec_state_flag(EXEC_MOTION_CANCEL);

            } else if ((c == '\n') || (c == '\r')) { // End of line reached

                if(!protocol_execute_realtime()) // Runtime command check point.
                    return !sys.exit;            // Bail to calling function upon system abort

                line[char_counter] = '\0'; // Set string termination character.

              #ifdef REPORT_ECHO_LINE_RECEIVED
                report_echo_line_received(line);
              #endif

                // Direct and execute one line of formatted input, and report status of execution.
                if (line_flags.overflow) // Report line overflow error.
                    rstatus = Status_Overflow;
                else if (line[0] == '\0' || char_counter == 0) // Empty or comment line. For syncing purposes.
                    rstatus = Status_OK;
                else if (line[0] == '$') // Grbl '$' system command
                    rstatus = system_execute_line(line);
                else if (sys.state & (STATE_ALARM | STATE_JOG)) // Everything else is gcode. Block if in alarm or jog mode.
                    rstatus = Status_SystemGClock;
                else  // Parse and execute g-code block.
                    rstatus = gc_execute_line(line);

                report_status_message(rstatus);

                // Reset tracking data for next line.
                line_flags.value = 0;
                char_counter = 0;

            } else if (c <= ' ' || line_flags.value) {
                // Throw away all whitepace, control characters, comment characters and overflow characters.
                if (c == ')' && line_flags.comment_parentheses)
                    // End of '()' comment. Resume line.
                    line_flags.comment_parentheses = false;
            } else if (c == '/') {
                // Block delete. Ignore character.
                // NOTE: If supported, would simply need to check the system if block delete is enabled.
                line_flags.block_delete = char_counter == 0 && sys.block_delete_enabled;
            } else if (c == '(') {
                // Enable comments flag and ignore all characters until ')' or EOL.
                // NOTE: This doesn't follow the NIST definition exactly, but is good enough for now.
                // In the future, we could simply remove the items within the comments, but retain the
                // comment control characters, so that the g-code parser can error-check it.
                line_flags.comment_parentheses = !line_flags.comment_semicolon;
            } else if (c == ';') {
                // NOTE: ';' comment to EOL is a LinuxCNC definition. Not NIST.
                line_flags.comment_semicolon = !line_flags.comment_parentheses;
            // TODO: Install '%' feature
            // } else if (c == '%') {
            // Program start-end percent sign NOT SUPPORTED.
            // NOTE: This maybe installed to tell Grbl when a program is running vs manual input,
            // where, during a program, the system auto-cycle start will continue to execute
            // everything until the next '%' sign. This will help fix resuming issues with certain
            // functions that empty the planner buffer to execute its task on-time.
            } else if (char_counter >= (LINE_BUFFER_SIZE - 1)) {
                // Detect line buffer overflow and set flag.
                line_flags.overflow = true;
            } else
                line[char_counter++] = (c >= 'a' && c <= 'z') ? c & 0x5F : c; // Upcase lowercase
        }

        // Handle extra command (internal stream)
        if(xcommand[0] != '\0') {

            if (xcommand[0] == '$') // Grbl '$' system command
                system_execute_line(xcommand);
            else if (sys.state & (STATE_ALARM | STATE_JOG)) // Everything else is gcode. Block if in alarm or jog mode.
                report_status_message(Status_SystemGClock);
            else // Parse and execute g-code block.
                gc_execute_line(xcommand);

            xcommand[0] = '\0';
        }

        // If there are no more characters in the serial read buffer to be processed and executed,
        // this indicates that g-code streaming has either filled the planner buffer or has
        // completed. In either case, auto-cycle start, if enabled, any queued moves.
        protocol_auto_cycle_start();

        if(!protocol_execute_realtime()) // Runtime command check point.
            return !sys.exit;            // Bail to main() program loop to reset system.
    }
}


// Block until all buffered steps are executed or in a cycle state. Works with feed hold
// during a synchronize call, if it should happen. Also, waits for clean cycle end.
void protocol_buffer_synchronize ()
{
    // If system is queued, ensure cycle resumes if the auto start flag is present.
    protocol_auto_cycle_start();
    while (protocol_execute_realtime() && (plan_get_current_block() || sys.state == STATE_CYCLE));
}


// Auto-cycle start triggers when there is a motion ready to execute and if the main program is not
// actively parsing commands.
// NOTE: This function is called from the main loop, buffer sync, and mc_line() only and executes
// when one of these conditions exist respectively: There are no more blocks sent (i.e. streaming
// is finished, single commands), a command that needs to wait for the motions in the buffer to
// execute calls a buffer sync, or the planner buffer is full and ready to go.
void protocol_auto_cycle_start ()
{
    if (plan_get_current_block() != NULL) // Check if there are any blocks in the buffer.
        system_set_exec_state_flag(EXEC_CYCLE_START); // If so, execute them!
}


// This function is the general interface to Grbl's real-time command execution system. It is called
// from various check points in the main program, primarily where there may be a while loop waiting
// for a buffer to clear space or any point where the execution time from the last check point may
// be more than a fraction of a second. This is a way to execute realtime commands asynchronously
// (aka multitasking) with grbl's g-code parsing and planning functions. This function also serves
// as an interface for the interrupts to set the system realtime flags, where only the main program
// handles them, removing the need to define more computationally-expensive volatile variables. This
// also provides a controlled way to execute certain tasks without having two or more instances of
// the same task, such as the planner recalculating the buffer upon a feedhold or overrides.
// NOTE: The sys_rt_exec_state variable flags are set by any process, step or serial interrupts, pinouts,
// limit switches, or the main program.
// Returns false if aborted
bool protocol_execute_realtime ()
{
    protocol_exec_rt_system();

    if(hal.execute_realtime)
      hal.execute_realtime(sys.state);

    if (sys.suspend.value)
      protocol_exec_rt_suspend();

  #ifdef EMULATE_EEPROM
    if(sys.state == STATE_IDLE && settings_dirty.is_dirty)
        eeprom_emu_sync_physical();
  #endif

    return !sys.abort;
}


// Executes run-time commands, when required. This function primarily operates as Grbl's state
// machine and controls the various real-time features Grbl has to offer.
// NOTE: Do not alter this unless you know exactly what you are doing!
void protocol_exec_rt_system ()
{
    uint8_t rt_exec; // Temp variable to avoid calling volatile multiple times.

    if (sys_rt_exec_alarm && (rt_exec = system_clear_exec_alarm())) { // Enter only if any bit flag is true
        // System alarm. Everything has shutdown by something that has gone severely wrong. Report
        // the source of the error to the user. If critical, Grbl disables by entering an infinite
        // loop until system reset/abort.
        sys.state = STATE_ALARM; // Set system alarm state
        report_alarm_message((alarm_code_t)rt_exec);
        // Halt everything upon a critical event flag. Currently hard and soft limits flag this.
        if (((alarm_code_t)rt_exec == Alarm_HardLimit) || ((alarm_code_t)rt_exec == Alarm_SoftLimit)) {
            report_feedback_message(Message_CriticalEvent);
            system_clear_exec_state_flag(EXEC_RESET); // Disable any existing reset
            // Block everything, except reset and status reports, until user issues reset or power
            // cycles. Hard limits typically occur while unattended or not paying attention. Gives
            // the user and a GUI time to do what is needed before resetting, like killing the
            // incoming stream. The same could be said about soft limits. While the position is not
            // lost, continued streaming could cause a serious crash if by chance it gets executed.
            while (bit_isfalse(sys_rt_exec_state, EXEC_RESET));
        }
    }

    if (sys_rt_exec_state && (rt_exec = system_clear_exec_states())) { // Get and clear volatile sys_rt_exec_state atomically.

        // NOTE: not sure if some exec_state flags needs to preserved in some cases... If so must be set again from rt_exec later

        // Execute system abort.
        if (rt_exec & EXEC_RESET) {
            sys.abort = true;  // Only place this is set true.
            return; // Nothing else to do but exit.
        }

        // Execute and serial print status
        if (rt_exec & EXEC_STATUS_REPORT)
            report_realtime_status();

        // NOTE: Once hold is initiated, the system immediately enters a suspend state to block all
        // main program processes until either reset or resumed. This ensures a hold completes safely.
        if (rt_exec & (EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP)) {

            // State check for allowable states for hold methods.
            // If in CYCLE or JOG states, immediately initiate a motion HOLD.
            if (!(sys.state & (STATE_ALARM | STATE_CHECK_MODE))) {

                if (sys.state & (STATE_CYCLE | STATE_JOG) && !(sys.suspend.motion_cancel || sys.suspend.jog_cancel)) { // Block, if already holding.
                    st_update_plan_block_parameters(); // Notify stepper module to recompute for hold deceleration.
                    sys.step_control.execute_hold = 1; // Initiate suspend state with active flag.
                    if (sys.state == STATE_JOG && !(rt_exec & EXEC_SLEEP)) // Jog cancelled upon any hold event, except for sleeping.
                        sys.suspend.jog_cancel = true;
                }

                // If IDLE, Grbl is not in motion. Simply indicate suspend state and hold is complete.
                if (sys.state == STATE_IDLE) {
                    sys.suspend.value = 0;
                    sys.suspend.hold_complete = true;
                }

                // Execute and flag a motion cancel with deceleration and return to idle. Used primarily by probing cycle
                // to halt and cancel the remainder of the motion.
                if ((rt_exec & EXEC_MOTION_CANCEL) && sys.state != STATE_IDLE) {
                    // MOTION_CANCEL only occurs during a CYCLE, but a HOLD and SAFETY_DOOR may been initiated beforehand
                    // to hold the CYCLE. Motion cancel is valid for a single planner block motion only, while jog cancel
                    // will handle and clear multiple planner block motions.
                    if (!(sys.state & STATE_JOG))
                        sys.suspend.motion_cancel = true; // NOTE: State is STATE_CYCLE.
                }

                // Execute a feed hold with deceleration, if required. Then, suspend system.
                if (rt_exec & EXEC_FEED_HOLD) {
                // Block SAFETY_DOOR, JOG, and SLEEP states from changing to HOLD state.
                    if (!(sys.state & (STATE_SAFETY_DOOR | STATE_JOG | STATE_SLEEP)))
                        sys.state = STATE_HOLD;
                }

                // Execute a safety door stop with a feed hold and disable spindle/coolant.
                // NOTE: Safety door differs from feed holds by stopping everything no matter state, disables powered
                // devices (spindle/coolant), and blocks resuming until switch is re-engaged.
                if (rt_exec & EXEC_SAFETY_DOOR) {
                    report_feedback_message(Message_SafetyDoorAjar);
                    // If jogging, block safety door methods until jog cancel is complete. Just flag that it happened.
                    if (!sys.suspend.jog_cancel) {
                        // Check if the safety re-opened during a restore parking motion only. Ignore if
                        // already retracting, parked or in sleep state.
                        if (sys.state == STATE_SAFETY_DOOR) {
                            if (sys.suspend.initiate_restore) { // Actively restoring
                                #ifdef PARKING_ENABLE
                                // Set hold and reset appropriate control flags to restart parking sequence.
                                if (sys.step_control.execute_sys_motion) {
                                    st_update_plan_block_parameters(); // Notify stepper module to recompute for hold deceleration.
                                    sys.step_control.execute_hold = true;
                                    sys.step_control.execute_sys_motion = true;
                                    sys.suspend.hold_complete = false;
                                } // else NO_MOTION is active.
                                #endif
                                sys.suspend.retract_complete = sys.suspend.initiate_restore = sys.suspend.restore_complete = false;
                                sys.suspend.restart_retract = true;
                            }
                        } // TODO: add else?
                        if (sys.state != STATE_SLEEP)
                            sys.state = STATE_SAFETY_DOOR;
                    }
                    // NOTE: This flag doesn't change when the door closes, unlike sys.state. Ensures any parking motions
                    // are executed if the door switch closes and the state returns to HOLD.
                    sys.suspend.safety_door_ajar = true;
                }
            }

            if (rt_exec & EXEC_SLEEP) {
                if (sys.state == STATE_ALARM)
                    sys.suspend.retract_complete = sys.suspend.hold_complete = true;
                sys.state = STATE_SLEEP;
            }
        }

        // Execute a cycle start by starting the stepper interrupt to begin executing the blocks in queue.
        if (rt_exec & EXEC_CYCLE_START) {
            // Block if called at same time as the hold commands: feed hold, motion cancel, and safety door.
            // Ensures auto-cycle-start doesn't resume a hold without an explicit user-input.
            if (!(rt_exec & (EXEC_FEED_HOLD | EXEC_MOTION_CANCEL | EXEC_SAFETY_DOOR))) {
                // Resume door state when parking motion has retracted and door has been closed.
                if ((sys.state == STATE_SAFETY_DOOR) && !sys.suspend.safety_door_ajar) {
                    if (sys.suspend.restore_complete) {
                        sys.state = STATE_IDLE; // Set to IDLE to immediately resume the cycle.
                    } else if (sys.suspend.retract_complete) {
                        // Flag to re-energize powered components and restore original position, if disabled by SAFETY_DOOR.
                        // NOTE: For a safety door to resume, the switch must be closed, as indicated by HOLD state, and
                        // the retraction execution is complete, which implies the initial feed hold is not active. To
                        // restore normal operation, the restore procedures must be initiated by the following flag. Once,
                        // they are complete, it will call CYCLE_START automatically to resume and exit the suspend.
                        sys.suspend.initiate_restore = true;
                    }
                }
                // Cycle start only when IDLE or when a hold is complete and ready to resume.
                if ((sys.state == STATE_IDLE) || ((sys.state & STATE_HOLD) && sys.suspend.hold_complete)) {
                    if (sys.state == STATE_HOLD && sys.spindle_stop_ovr.value) {
                        sys.spindle_stop_ovr.restore_cycle = true; // Set to restore in suspend routine and cycle start after.
                    } else {
                        // Start cycle only if queued motions exist in planner buffer and the motion is not canceled.
                        sys.step_control.value = 0; // Restore step control to normal operation
                        if (plan_get_current_block() && !sys.suspend.motion_cancel) {
                            sys.suspend.value = 0; // Break suspend state.
                            sys.state = STATE_CYCLE;
                            st_prep_buffer(); // Initialize step segment buffer before beginning cycle.
                            st_wake_up();
                        } else { // Otherwise, do nothing. Set and resume IDLE state.
                            sys.suspend.value = 0; // Break suspend state.
                            sys.state = STATE_IDLE;
                        }
                    }
                }
            }
        }

        if (rt_exec & EXEC_CYCLE_STOP) {
            // Reinitializes the cycle plan and stepper system after a feed hold for a resume. Called by
            // realtime command execution in the main program, ensuring that the planner re-plans safely.
            // NOTE: Bresenham algorithm variables are still maintained through both the planner and stepper
            // cycle reinitializations. The stepper path should continue exactly as if nothing has happened.
            // NOTE: EXEC_CYCLE_STOP is set by the stepper subsystem when a cycle or feed hold completes.
            if ((sys.state & (STATE_HOLD|STATE_SAFETY_DOOR|STATE_SLEEP)) && !sys.soft_limit && !sys.suspend.jog_cancel) {
                // Hold complete. Set to indicate ready to resume.  Remain in HOLD or DOOR states until user
                // has issued a resume command or reset.
                plan_cycle_reinitialize();
                if (sys.step_control.execute_hold) {
                    sys.suspend.hold_complete = true;
                    sys.step_control.execute_hold = false;
                }
                sys.step_control.execute_sys_motion = false;
            } else {
                // Motion complete. Includes CYCLE/JOG/HOMING states and jog cancel/motion cancel/soft limit events.
                // NOTE: Motion and jog cancel both immediately return to idle after the hold completes.
                if (sys.suspend.jog_cancel) {   // For jog cancel, flush buffers and sync positions.
                    sys.step_control.value = 0;
                    plan_reset();
                    st_reset();
                    gc_sync_position();
                    plan_sync_position();
                }
                if (sys.suspend.safety_door_ajar) { // Only occurs when safety door opens during jog.
                    sys.suspend.jog_cancel = false;
                    sys.suspend.hold_complete = true;
                    sys.state = STATE_SAFETY_DOOR;
                } else {
                    sys.suspend.value = 0;
                    /*          if(sys.state == STATE_JOG) {
                    sys.state = STATE_IDLE;
                    system_clear_exec_state_flag(EXEC_MOTION_CANCEL);
                    } else NOTE: not sure this patch is needed anymore */
                    sys.state = STATE_IDLE;
                }
            }
        }
    }

    // Execute overrides.

    if((rt_exec = get_feed_ovr())) {

        uint8_t new_f_override = sys.f_override, new_r_override = sys.r_override;

        do {

          switch(rt_exec) {

              case CMD_FEED_OVR_RESET:
                  new_f_override = DEFAULT_FEED_OVERRIDE;
                  break;

              case CMD_FEED_OVR_COARSE_PLUS:
                  new_f_override += FEED_OVERRIDE_COARSE_INCREMENT;
                  break;

              case CMD_FEED_OVR_COARSE_MINUS:
                  new_f_override -= FEED_OVERRIDE_COARSE_INCREMENT;
                  break;

              case CMD_FEED_OVR_FINE_PLUS:
                  new_f_override += FEED_OVERRIDE_FINE_INCREMENT;
                  break;

              case CMD_FEED_OVR_FINE_MINUS:
                  new_f_override -= FEED_OVERRIDE_FINE_INCREMENT;
                  break;

              case CMD_RAPID_OVR_RESET:
                  new_r_override = DEFAULT_RAPID_OVERRIDE;
                  break;

              case CMD_RAPID_OVR_MEDIUM:
                  new_r_override = RAPID_OVERRIDE_MEDIUM;
                  break;

              case CMD_RAPID_OVR_LOW:
                  new_r_override = RAPID_OVERRIDE_LOW;
                  break;
          }

        } while(rt_exec = get_feed_ovr());

        new_f_override = max(min(new_f_override, MAX_FEED_RATE_OVERRIDE), MIN_FEED_RATE_OVERRIDE);

        if ((new_f_override != sys.f_override) || (new_r_override != sys.r_override)) {
          sys.f_override = new_f_override;
          sys.r_override = new_r_override;
          sys.report_ovr_counter = 0; // Set to report change immediately
          plan_update_velocity_profile_parameters();
          plan_cycle_reinitialize();
        }
    }

    if((rt_exec = get_accessory_ovr())) {

        bool spindle_stop = false;
        uint8_t last_s_override =  sys.spindle_speed_ovr, coolant_state = gc_state.modal.coolant;

        do {

          switch(rt_exec) {

              case CMD_SPINDLE_OVR_RESET:
                  last_s_override = DEFAULT_SPINDLE_SPEED_OVERRIDE;
                  break;

              case CMD_SPINDLE_OVR_COARSE_PLUS:
                  last_s_override += SPINDLE_OVERRIDE_COARSE_INCREMENT;
                  break;

              case CMD_SPINDLE_OVR_COARSE_MINUS:
                  last_s_override -= SPINDLE_OVERRIDE_COARSE_INCREMENT;
                  break;

              case CMD_SPINDLE_OVR_FINE_PLUS:
                  last_s_override += SPINDLE_OVERRIDE_FINE_INCREMENT;
                  break;

              case CMD_SPINDLE_OVR_FINE_MINUS:
                  last_s_override -= SPINDLE_OVERRIDE_FINE_INCREMENT;
                  break;

              case CMD_SPINDLE_OVR_STOP:
                  spindle_stop = !spindle_stop;
                  break;

            #ifdef ENABLE_M7
              case CMD_COOLANT_MIST_OVR_TOGGLE:
                  if ((sys.state == STATE_IDLE) || (sys.state & (STATE_CYCLE | STATE_HOLD))) {
                      if (bit_istrue(coolant_state, COOLANT_MIST_ENABLE))
                          bit_false(coolant_state, COOLANT_MIST_ENABLE);
                      else
                          bit_true(coolant_state, COOLANT_MIST_ENABLE);
                  }
                  break;
            #endif

              case CMD_COOLANT_FLOOD_OVR_TOGGLE:
                  if ((sys.state == STATE_IDLE) || (sys.state & (STATE_CYCLE | STATE_HOLD))) {
                      if (bit_istrue(coolant_state, COOLANT_FLOOD_ENABLE))
                          bit_false(coolant_state, COOLANT_FLOOD_ENABLE);
                      else
                          bit_true(coolant_state, COOLANT_FLOOD_ENABLE);
                  }
                  break;

                  default:
                      if(hal.userdefined_rt_command_execute)
                          hal.userdefined_rt_command_execute(rt_exec);
                      break;
              }

        } while((rt_exec = get_accessory_ovr()));

        // NOTE: Unlike motion overrides, spindle overrides do not require a planner reinitialization.
        last_s_override = max(min(last_s_override, MAX_SPINDLE_SPEED_OVERRIDE), MIN_SPINDLE_SPEED_OVERRIDE);

        if (last_s_override != sys.spindle_speed_ovr) {
            sys.step_control.update_spindle_pwm = true;
            sys.spindle_speed_ovr = last_s_override;
            sys.report_ovr_counter = 0; // Set to report change immediately
        }

      // NOTE: Since coolant state always performs a planner sync whenever it changes, the current
      // run state can be determined by checking the parser state.
        if(coolant_state != gc_state.modal.coolant) {
            coolant_set_state(coolant_state); // Report counter set in coolant_set_state().
            gc_state.modal.coolant = coolant_state;
        }

        if (spindle_stop && sys.state == STATE_HOLD) {
            // Spindle stop override allowed only while in HOLD state.
            // NOTE: Report counters are set in spindle_set_state() when spindle stop is executed.
                if (!sys.spindle_stop_ovr.value)
                sys.spindle_stop_ovr.initiate = true;
            else if (sys.spindle_stop_ovr.enabled)
                sys.spindle_stop_ovr.restore = true;
        }
    }

    // End execute overrides.

    // Reload step segment buffer
    if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_SAFETY_DOOR | STATE_HOMING | STATE_SLEEP| STATE_JOG))
        st_prep_buffer();
}


// Handles Grbl system suspend procedures, such as feed hold, safety door, and parking motion.
// The system will enter this loop, create local variables for suspend tasks, and return to
// whatever function that invoked the suspend, such that Grbl resumes normal operation.
// This function is written in a way to promote custom parking motions. Simply use this as a
// template
static void protocol_exec_rt_suspend ()
{
  #ifdef PARKING_ENABLE
    // Declare and initialize parking local variables
    float restore_target[N_AXIS];
    float parking_target[N_AXIS];
    float retract_waypoint = PARKING_PULLOUT_INCREMENT;
    plan_line_data_t plan_data;
    plan_line_data_t *pl_data = &plan_data;
    memset(pl_data,0,sizeof(plan_line_data_t));
    pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
   #ifdef USE_LINE_NUMBERS
    pl_data->line_number = PARKING_MOTION_LINE_NUMBER;
   #endif
  #endif

    plan_block_t *block = plan_get_current_block();
    uint8_t restore_condition;
  #ifdef VARIABLE_SPINDLE
    float restore_spindle_speed;
    if (block == NULL) {
        restore_condition = (gc_state.modal.spindle | gc_state.modal.coolant);
        restore_spindle_speed = gc_state.spindle_speed;
    } else {
        restore_condition = block->condition;
        restore_spindle_speed = block->spindle_speed;
    }
   #ifdef DISABLE_LASER_DURING_HOLD
    if (settings.flags.laser_mode)
        enqueue_accessory_ovr(CMD_SPINDLE_OVR_STOP);
   #endif
  #else
    restore_condition = block == NULL ? (gc_state.modal.spindle | gc_state.modal.coolant) : block->condition;
  #endif

    while (sys.suspend.value) {

        if (sys.abort)
            return;

        // Block until initial hold is complete and the machine has stopped motion.
        if (sys.suspend.hold_complete) {

            // Parking manager. Handles de/re-energizing, switch state checks, and parking motions for
            // the safety door and sleep states.
            if (sys.state & (STATE_SAFETY_DOOR | STATE_SLEEP)) {

                // Handles retraction motions and de-energizing.
                if (!sys.suspend.retract_complete) {

                    // Ensure any prior spindle stop override is disabled at start of safety door routine.
                    sys.spindle_stop_ovr.value = 0;

                  #ifndef PARKING_ENABLE

                    spindle_set_state(SPINDLE_DISABLE,0.0f); // De-energize
                    coolant_set_state(COOLANT_DISABLE);     // De-energize

                  #else

                    // Get current position and store restore location and spindle retract waypoint.
                    system_convert_array_steps_to_mpos(parking_target,sys_position);
                    if (!sys.suspend.restart_retract) {
                        memcpy(restore_target,parking_target,sizeof(parking_target));
                        retract_waypoint += restore_target[PARKING_AXIS];
                        retract_waypoint = min(retract_waypoint,PARKING_TARGET);
                    }

                    // Execute slow pull-out parking retract motion. Parking requires homing enabled, the
                    // current location not exceeding the parking target location, and laser mode disabled.
                    // NOTE: State is will remain DOOR, until the de-energizing and retract is complete.
                   #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
                    if (settings.flags.homing_enable && (parking_target[PARKING_AXIS] < PARKING_TARGET) && !settings.flags.laser_mode && (sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
                   #else
                    if (settings.flags.homing_enable) && (parking_target[PARKING_AXIS] < PARKING_TARGET) && !settings.flags.laser_mode) {
                   #endif
                        // Retract spindle by pullout distance. Ensure retraction motion moves away from
                        // the workpiece and waypoint motion doesn't exceed the parking target location.
                        if (parking_target[PARKING_AXIS] < retract_waypoint) {
                            parking_target[PARKING_AXIS] = retract_waypoint;
                            pl_data->feed_rate = PARKING_PULLOUT_RATE;
                            pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); // Retain accessory state
                            pl_data->spindle_speed = restore_spindle_speed;
                            mc_parking_motion(parking_target, pl_data);
                        }

                        // NOTE: Clear accessory state after retract and after an aborted restore motion.
                        pl_data->condition = (PL_COND_FLAG_SYSTEM_MOTION|PL_COND_FLAG_NO_FEED_OVERRIDE);
                        pl_data->spindle_speed = 0.0;
                        spindle_set_state(SPINDLE_DISABLE,0.0); // De-energize
                        coolant_set_state(COOLANT_DISABLE); // De-energize

                        // Execute fast parking retract motion to parking target location.
                        if (parking_target[PARKING_AXIS] < PARKING_TARGET) {
                            parking_target[PARKING_AXIS] = PARKING_TARGET;
                            pl_data->feed_rate = PARKING_RATE;
                            mc_parking_motion(parking_target, pl_data);
                        }

                    } else {

                        // Parking motion not possible. Just disable the spindle and coolant.
                        // NOTE: Laser mode does not start a parking motion to ensure the laser stops immediately.
                        spindle_set_state(SPINDLE_DISABLE,0.0); // De-energize
                        coolant_set_state(COOLANT_DISABLE);     // De-energize

                    }
                  #endif

                    sys.suspend.restart_retract = false;
                    sys.suspend.retract_complete = true;

                } else {

                    if (sys.state == STATE_SLEEP) {
                        report_feedback_message(Message_SleepMode);
                        // Spindle and coolant should already be stopped, but do it again just to be sure.
                        spindle_set_state(SPINDLE_DISABLE, 0.0f); // De-energize
                        coolant_set_state(COOLANT_DISABLE); // De-energize
                        st_go_idle(); // Disable steppers
                        while (!(sys.abort))
                            protocol_exec_rt_system(); // Do nothing until reset.
                        return; // Abort received. Return to re-initialize.
                    }

                    // Allows resuming from parking/safety door. Actively checks if safety door is closed and ready to resume.
                    if (sys.state == STATE_SAFETY_DOOR && !system_check_safety_door_ajar())
                        sys.suspend.safety_door_ajar = false; // Reset door ajar flag to denote ready to resume.

                    // Handles parking restore and safety door resume.
                    if (sys.suspend.initiate_restore) {

                      #ifdef PARKING_ENABLE
                        // Execute fast restore motion to the pull-out position. Parking requires homing enabled.
                        // NOTE: State is will remain DOOR, until the de-energizing and retract is complete.
                       #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
                        if (settings.flags.homing_enable && !settings.flags.laser_mode && sys.override_ctrl == OVERRIDE_PARKING_MOTION) {
                       #else
                        if (settings.flags.homing_enable && !settings.flags.laser_mode) {
                       #endif
                            // Check to ensure the motion doesn't move below pull-out position.
                            if (parking_target[PARKING_AXIS] <= PARKING_TARGET) {
                                parking_target[PARKING_AXIS] = retract_waypoint;
                                pl_data->feed_rate = PARKING_RATE;
                                mc_parking_motion(parking_target, pl_data);
                            }
                        }
                      #endif

                        // Delayed Tasks: Restart spindle and coolant, delay to power-up, then resume cycle.
                        // Block if safety door re-opened during prior restore actions.
                        if (gc_state.modal.spindle != SPINDLE_DISABLE && !sys.suspend.restart_retract) {
                            if (settings.flags.laser_mode)
                            // When in laser mode, ignore spindle spin-up delay. Set to turn on laser when cycle starts.
                                sys.step_control.update_spindle_pwm = true;
                            else {
                                spindle_set_state(restore_condition & PL_COND_FLAGS_SPINDLE, restore_spindle_speed);
                                delay_sec(SAFETY_DOOR_SPINDLE_DELAY, DelayMode_SysSuspend);
                            }
                        }

                        // Block if safety door re-opened during prior restore actions.
                        if (gc_state.modal.coolant != COOLANT_DISABLE && !sys.suspend.restart_retract) {
                            // NOTE: Laser mode will honor this delay. An exhaust system is often controlled by this pin.
                            coolant_set_state(restore_condition & PL_COND_FLAGS_COOLANT);
                            delay_sec(SAFETY_DOOR_COOLANT_DELAY, DelayMode_SysSuspend);
                        }

                      #ifdef PARKING_ENABLE
                        // Execute slow plunge motion from pull-out position to resume position.
                       #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
                        if (settings.flags.homing_enable && !settings.flags.laser_mode && sys.override_ctrl == OVERRIDE_PARKING_MOTION) {
                       #else
                        if (settings.flags.homing_enable && !settings.flags.laser_mode) {
                       #endif
                            // Block if safety door re-opened during prior restore actions.
                            if (!sys.suspend.restart_retract) {
                                // Regardless if the retract parking motion was a valid/safe motion or not, the
                                // restore parking motion should logically be valid, either by returning to the
                                // original position through valid machine space or by not moving at all.
                                pl_data->feed_rate = PARKING_PULLOUT_RATE;
                                pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); // Restore accessory state
                                pl_data->spindle_speed = restore_spindle_speed;
                                mc_parking_motion(restore_target, pl_data);
                            }
                        }
                      #endif

                        if (!sys.suspend.restart_retract) {
                            sys.suspend.restore_complete = true;
                            system_set_exec_state_flag(EXEC_CYCLE_START); // Set to resume program.
                        }
                    }
                }

            } else {

                // Feed hold manager. Controls spindle stop override states.
                // NOTE: Hold ensured as completed by condition check at the beginning of suspend routine.
                if (sys.spindle_stop_ovr.value) {
                    // Handles beginning of spindle stop
                    if (sys.spindle_stop_ovr.initiate) {
                        sys.spindle_stop_ovr.value = 0; // Clear stop override state
                        if (gc_state.modal.spindle != SPINDLE_DISABLE) {
                            spindle_set_state(SPINDLE_DISABLE, 0.0f); // De-energize
                            sys.spindle_stop_ovr.enabled = true; // Set stop override state to enabled, if de-energized.
                        }
                    // Handles restoring of spindle state
                    } else if (sys.spindle_stop_ovr.restore || sys.spindle_stop_ovr.restore_cycle) {
                        if (gc_state.modal.spindle != SPINDLE_DISABLE) {
                            report_feedback_message(Message_SpindleRestore);
                            if (settings.flags.laser_mode) // When in laser mode, ignore spindle spin-up delay. Set to turn on laser when cycle starts.
                                sys.step_control.update_spindle_pwm = true;
                            else
                                spindle_set_state(restore_condition & PL_COND_FLAGS_SPINDLE, restore_spindle_speed);
                        }
                        if (sys.spindle_stop_ovr.restore_cycle)
                            system_set_exec_state_flag(EXEC_CYCLE_START);  // Set to resume program.
                        sys.spindle_stop_ovr.value = 0; // Clear stop override state
                    }
                } else if (sys.step_control.update_spindle_pwm) {
                    // Handles spindle state during hold. NOTE: Spindle speed overrides may be altered during hold state.
                    // NOTE: sys.step_control.update_spindle_pwm is automatically reset upon resume in step generator.
                    spindle_set_state(restore_condition & PL_COND_FLAGS_SPINDLE, restore_spindle_speed);
                    sys.step_control.update_spindle_pwm = false;
                }
            }
        }
        protocol_exec_rt_system();
    }
}

bool protocol_process_realtime (int32_t data) {

	bool add = true;

	switch (data) {

	    case CMD_RESET: // Call motion control reset routine.
	        mc_reset();
	        add = false;
	        break;

	    case CMD_EXIT: // Call motion control reset routine.
	        mc_reset();
	        sys.exit = true;
	        add = false;
	        break;

	    case CMD_STATUS_REPORT: // Set as true
	        system_set_exec_state_flag(EXEC_STATUS_REPORT);
	        add = false;
	        break;

	    case CMD_CYCLE_START: // Set as true
	        system_set_exec_state_flag(EXEC_CYCLE_START);
	        add = false;
	        break;

	    case CMD_FEED_HOLD: // Set as true
	        system_set_exec_state_flag(EXEC_FEED_HOLD);
	        add = false;
	        break;

	    case CMD_SAFETY_DOOR: // Set as true
	        system_set_exec_state_flag(EXEC_SAFETY_DOOR);
	        add = false;
	        break;

	    case CMD_JOG_CANCEL: // Cancel jogging
            char_counter = 0;
            hal.serial_cancel_read_buffer();
            break;

        case CMD_FEED_OVR_RESET:
        case CMD_FEED_OVR_COARSE_PLUS:
        case CMD_FEED_OVR_COARSE_MINUS:
        case CMD_FEED_OVR_FINE_PLUS:
        case CMD_FEED_OVR_FINE_MINUS:
        case CMD_RAPID_OVR_RESET:
        case CMD_RAPID_OVR_MEDIUM:
        case CMD_RAPID_OVR_LOW:
            enqueue_feed_ovr(data);
            break;

        default:
            if(data > 0x7F)
                enqueue_accessory_ovr(data);
            break;
	}

	return add && data <= 0x7F;
}
