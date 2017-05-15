/*
  system.c - Handles system level commands and real-time processes
  Part of Grbl

  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

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

// Pin change interrupt for pin-out commands, i.e. cycle start, feed hold, and reset. Sets
// only the realtime command execute variable to have the main program execute these when
// its ready. This works exactly like the character-based realtime commands when picked off
// directly from the incoming serial data stream.
void control_interrupt_handler (control_signals_t signals)
{
    if (signals.value) {
        if (signals.reset)
            mc_reset();
        else if (signals.cycle_start)
            bit_true(sys_rt_exec_state, EXEC_CYCLE_START);
      #ifndef ENABLE_SAFETY_DOOR_INPUT_PIN
        else if (signals.feed_hold)
            bit_true(sys_rt_exec_state, EXEC_FEED_HOLD);
      #else
        else if (signals.safety_door)
            bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
      #endif
    }
}


// Executes user startup script, if stored.
void system_execute_startup (char *line)
{
    uint8_t n;
    for (n=0; n < N_STARTUP_LINE; n++) {
        if (!(settings_read_startup_line(n, line)))
            report_execute_startup_message(line, Status_SettingReadFail);
        else if (line[0] != '\0')
            report_execute_startup_message(line, gc_execute_line(line));
    }
}


// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also executes Grbl internal commands, such as
// settings, initiating the homing cycle, and toggling switch states. This differs from
// the realtime command module by being susceptible to when Grbl is ready to execute the
// next line during a cycle, so for switches like block delete, the switch only effects
// the lines that are processed afterward, not necessarily real-time during a cycle,
// since there are motions already stored in the buffer. However, this 'lag' should not
// be an issue, since these commands are not typically used during a cycle.
status_code_t system_execute_line (char *line)
{

    uint8_t counter = 1;
    float parameter, value;
    status_code_t retval = Status_OK;
    control_signals_t control_signals;

    switch (line[1]) {

        case '\0' :
            report_grbl_help();
            break;

        case 'J' : // Jogging
            // Execute only if in IDLE or JOG states.
            if (sys.state != STATE_IDLE && sys.state != STATE_JOG)
                retval = Status_IdleError;
            else
                retval = line[2] != '=' ? Status_InvalidStatement : gc_execute_line(line); // NOTE: $J= is ignored inside g-code parser and used to detect jog motions.
            break;

        case '$' : // Prints Grbl settings
            if (line[2] != '\0' )
                retval = Status_InvalidStatement;
            else if (sys.state & (STATE_CYCLE | STATE_HOLD))
                retval = Status_IdleError; // Block during cycle. Takes too long to print.
            else
                report_grbl_settings();
            break;

        case 'G' : // Prints gcode parser state
            if (line[2] != '\0' )
                retval = Status_InvalidStatement;
            else
                // TODO: Move this to realtime commands for GUIs to request this data during suspend-state.
                report_gcode_modes();
            break;

        case 'B' : // Toggle block delete mode
            if (line[2] != '\0')
                retval = Status_InvalidStatement;
            else {
                sys.block_delete_enabled = !sys.block_delete_enabled;
                report_feedback_message(sys.block_delete_enabled ? Message_Enabled : Message_Disabled);
            }
            break;

        case 'C' : // Set check g-code mode [IDLE/CHECK]
            if (line[2] != '\0')
                retval = Status_InvalidStatement;
            // Perform reset when toggling off. Check g-code mode should only work if Grbl
            // is idle and ready, regardless of alarm locks. This is mainly to keep things
            // simple and consistent.
            else if (sys.state == STATE_CHECK_MODE) {
                mc_reset();
                report_feedback_message(Message_Disabled);
            }
            else if (sys.state)  // Requires idle mode.
                retval = Status_IdleError;
            else {
                sys.state = STATE_CHECK_MODE;
                report_feedback_message(Message_Enabled);
            }
            break;

        case 'X' : // Disable alarm lock [ALARM]
            if (line[2] != '\0' )
                retval = Status_InvalidStatement;
            else if (sys.state == STATE_ALARM) {

            	control_signals = system_control_get_state();

            	// Block if safety door is ajar.
                if (control_signals.safety_door_ajar)
                    return Status_CheckDoor;

            	// Block if safety reset is active.
                if(control_signals.reset)
                    return Status_Reset;

                report_feedback_message(Message_AlarmUnlock);
                sys.state = STATE_IDLE;
                // Don't run startup script. Prevents stored moves in startup from causing accidents.
            } // Otherwise, no effect.
            break;

        default :

            // Block any system command that requires the state as IDLE/ALARM. (i.e. EEPROM, homing)
            if (sys.state == STATE_IDLE || sys.state == STATE_ALARM) switch(line[1]) {

                case '#' : // Print Grbl NGC parameters
                    if (line[2] != '\0')
                        return Status_InvalidStatement;
                    report_ngc_parameters();
                    break;

                case 'H' : // Perform homing cycle [IDLE/ALARM]

                    if (!settings.flags.homing_enable)
                        return Status_SettingDisabled;

                    control_signals = system_control_get_state();

                    // Block if safety door is ajar.
                    if (control_signals.safety_door_ajar)
                        return Status_CheckDoor;

                	// Block if safety reset is active.
                    if(control_signals.reset)
                        return Status_Reset;

                    sys.state = STATE_HOMING; // Set system state variable

                    if (line[2] == '\0') {
                        mc_homing_cycle(HOMING_CYCLE_ALL);
                  #ifdef HOMING_SINGLE_AXIS_COMMANDS
                    } else if (line[3] == '\0') {
                        switch (line[2]) {
                            case 'X':
                                mc_homing_cycle(HOMING_CYCLE_X);
                                break;
                            case 'Y':
                                mc_homing_cycle(HOMING_CYCLE_Y);
                                break;
                            case 'Z':
                                mc_homing_cycle(HOMING_CYCLE_Z);
                                break;
                            default:
                                return Status_InvalidStatement;
                        }
                  #endif
                    } else
                        return Status_InvalidStatement;

                    if (!sys.abort) {  // Execute startup scripts after successful homing.
                        sys.state = STATE_IDLE; // Set to IDLE when complete.
                        st_go_idle(); // Set steppers to the settings idle state before returning.
                        if (line[2] == '\0')
                            system_execute_startup(line);
                    }
                    break;

                case 'S' : // Puts Grbl to sleep [IDLE/ALARM]
                    if ((line[2] != 'L') || (line[3] != 'P') || (line[4] != '\0'))
                        return Status_InvalidStatement;
                    system_set_exec_state_flag(EXEC_SLEEP); // Set to execute sleep mode immediately
                    break;

                case 'I' : // Print or store build info. [IDLE/ALARM]
                    if (line[2] == '\0') {
                        settings_read_build_info(line);
                        report_build_info(line);
                    }
                  #ifdef ENABLE_BUILD_INFO_WRITE_COMMAND
                    else if (line[2] == '=' && strlen(&line[3]) < (MAX_STORED_LINE_LENGTH - 1))
                        settings_store_build_info(&line[3]);
                  #endif
                    else
                        retval = Status_InvalidStatement;
                    break;

                case 'R' : // Restore defaults [IDLE/ALARM]

                    if ((line[2] != 'S') || (line[3] != 'T') || (line[4] != '=') || (line[6] != '\0'))
                        return Status_InvalidStatement;

                    switch (line[5]) {

                      #ifdef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS
                        case '$':
                            settings_restore SETTINGS_RESTORE_DEFAULTS;
                            break;
                      #endif

                      #ifdef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS
                        case '#':
                            settings_restore(SETTINGS_RESTORE_PARAMETERS);
                            break;
                      #endif

                      #ifdef ENABLE_RESTORE_EEPROM_WIPE_ALL
                        case '*':
                            settings_restore(SETTINGS_RESTORE_ALL);
                            break;
                      #endif

                        default: return Status_InvalidStatement;
                    }
                    report_feedback_message(Message_RestoreDefaults);
                    mc_reset(); // Force reset to ensure settings are initialized correctly.
                    break;

                case 'N' : // Startup lines. [IDLE/ALARM]
                    if (line[++counter] == '\0') { // Print startup lines
                        for (counter = 0; counter < N_STARTUP_LINE; counter++) {
                            if (!(settings_read_startup_line(counter, line)))
                                report_status_message(Status_SettingReadFail);
                            else
                                report_startup_line(counter, line);
                        }
                        break;
                    }
                    if (sys.state != STATE_IDLE) { // Store startup line [IDLE Only] Prevents motion during ALARM.
                        retval = Status_IdleError;
                        break;
                    }
                    // No break. Continues into default: to read remaining command characters.

                default :  // Storing setting methods [IDLE/ALARM]
                    if(!read_float(line, &counter, &parameter))
                        retval = Status_BadNumberFormat;
                    else if(line[counter++] != '=')
                        retval = Status_InvalidStatement;
                    else if (line[1] == 'N') { // Store startup line
                        line = &line[counter];
                        if(strlen(line) >= (MAX_STORED_LINE_LENGTH - 1))
                            retval = Status_Overflow;
                        else if ((retval = gc_execute_line(line)) == Status_OK) // Execute gcode block to ensure block is valid.
                            settings_store_startup_line((uint8_t)truncf(parameter), line);
                    } else { // Store global setting.
                        if(!read_float(line, &counter, &value))
                            retval = Status_BadNumberFormat;
                        else if((line[counter] != 0) || (parameter > 255))
                            retval = Status_InvalidStatement;
                        else
                            retval = settings_store_global_setting((uint8_t)parameter, value);
                    }
            } else
                retval = Status_IdleError;
    }

    return retval; // If '$' command makes it to here, then everything's ok.
}



void system_flag_wco_change ()
{
  #ifdef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE
    protocol_buffer_synchronize();
  #endif
    sys.report_wco_counter = 0;
}


// Returns machine position of axis 'idx'. Must be sent a 'step' array.
// NOTE: If motor steps and machine position are not in the same coordinate frame, this function
//   serves as a central place to compute the transformation.
inline float system_convert_axis_steps_to_mpos (int32_t *steps, uint32_t idx)
{
  #ifdef COREXY
    return (float)(idx == X_AXIS ? system_convert_corexy_to_x_axis_steps(steps) : (idx == Y_AXIS ? system_convert_corexy_to_y_axis_steps(steps) : steps[idx])) / settings.steps_per_mm[idx];
  #else
    return steps[idx] / settings.steps_per_mm[idx];
  #endif
}


void system_convert_array_steps_to_mpos (float *position, int32_t *steps)
{
    uint32_t idx = N_AXIS;
    do {
        idx--;
        position[idx] = system_convert_axis_steps_to_mpos(steps, idx);
    } while(idx);
}


// CoreXY calculation only. Returns x or y-axis "steps" based on CoreXY motor steps.
#ifdef COREXY
int32_t system_convert_corexy_to_x_axis_steps (int32_t *steps)
{
    return (steps[A_MOTOR] + steps[B_MOTOR]) / 2;
}
int32_t system_convert_corexy_to_y_axis_steps (int32_t *steps)
{
    return (steps[A_MOTOR] - steps[B_MOTOR]) / 2 ;
}
#endif


// Checks and reports if target array exceeds machine travel limits. Returns true if check failed.
bool system_check_travel_limits(float *target)
{
    bool failed = false;
    uint32_t idx = N_AXIS;

    do {
        idx--;
    #ifdef HOMING_FORCE_SET_ORIGIN
    // When homing forced set origin is enabled, soft limits checks need to account for directionality.
    // NOTE: max_travel is stored as negative
        failed = bit_istrue(settings.homing_dir_mask, bit(idx))
                  ? (target[idx] < 0.0f || target[idx] > -settings.max_travel[idx])
                  : (target[idx] > 0.0f || target[idx] < settings.max_travel[idx]);
    #else
        // NOTE: max_travel is stored as negative
        failed = target[idx] > 0.0f || target[idx] < settings.max_travel[idx];
    #endif
    } while(!failed && idx);

  return failed;
}

