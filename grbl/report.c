/*
  report.c - reporting and messaging methods
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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

/*
  This file functions as the primary feedback interface for Grbl. Any outgoing data, such
  as the protocol status messages, feedback messages, and status reports, are stored here.
  For the most part, these functions primarily are called from protocol.c methods. If a
  different style feedback is desired (i.e. JSON), then a user can change these following
  methods to accomodate their needs.
*/

#include "grbl.h"

// Internal report utilities to reduce flash with repetitive tasks turned into functions.
static void report_util_setting_prefix (uint8_t n) {
    serial_write('$');
    print_uint8_base10(n);
    serial_write('=');
}

inline static void report_util_line_feed () {
    printPgmString(PSTR("\r\n"));
}

inline static void report_util_feedback_line_feed () {
    serial_write(']');
    report_util_line_feed();
}

inline static void report_util_gcode_modes_G () {
    printPgmString(PSTR(" G"));
}

inline static void report_util_gcode_modes_M () {
    printPgmString(PSTR(" M"));
}

// static void report_util_comment_line_feed() { serial_write(')'); report_util_line_feed(); }

static void report_util_axis_values (float *axis_value) {
    uint8_t idx;
    for (idx=0; idx<N_AXIS; idx++) {
        printFloat_CoordValue(axis_value[idx]);
        if (idx < (N_AXIS - 1))
            serial_write(',');
    }
}

static void report_util_uint8_setting (setting_type_t n, int val) {
    report_util_setting_prefix((uint8_t)n);
    print_uint8_base10(val);
    report_util_line_feed();
 // report_util_setting_string(n);
}

static void report_util_float_setting (uint8_t n, float val, uint8_t n_decimal) {
    report_util_setting_prefix(n);
    printFloat(val,n_decimal);
    report_util_line_feed();
 // report_util_setting_string(n);
}


// Handles the primary confirmation protocol response for streaming interfaces and human-feedback.
// For every incoming line, this method responds with an 'ok' for a successful command or an
// 'error:'  to indicate some error event with the line or some critical system error during
// operation. Errors events can originate from the g-code parser, settings module, or asynchronously
// from a critical error, such as a triggered hard limit. Interface should always monitor for these
// responses.
void report_status_message (status_code_t status_code)
{
    switch(status_code) {

        case Status_OK: // STATUS_OK
            printPgmString(PSTR("ok\r\n")); break;

        default:
            printPgmString(PSTR("error:"));
            print_uint8_base10((uint8_t)status_code);
            report_util_line_feed();
    }
}

// Prints alarm messages.
void report_alarm_message (alarm_code_t alarm_code)
{
    printPgmString(PSTR("ALARM:"));
    print_uint8_base10((uint8_t)alarm_code);
    report_util_line_feed();
    delay_ms(500); // Force delay to ensure message clears serial write buffer.
}

// Prints feedback messages. This serves as a centralized method to provide additional
// user feedback for things that are not of the status/alarm message protocol. These are
// messages such as setup warnings, switch toggling, and how to exit alarms.
// NOTE: For interfaces, messages are always placed within brackets. And if silent mode
// is installed, the message number codes are less than zero.
void report_feedback_message(message_code_t message_code)
{

    printPgmString(PSTR("[MSG:"));

    switch(message_code) {

        case Message_CriticalEvent:
            printPgmString(PSTR("Reset to continue"));
            break;

        case Message_AlarmLock:
            printPgmString(PSTR("'$H'|'$X' to unlock"));
            break;

        case Message_AlarmUnlock:
            printPgmString(PSTR("Caution: Unlocked"));
            break;

        case Message_Enabled:
            printPgmString(PSTR("Enabled"));
            break;

        case Message_Disabled:
            printPgmString(PSTR("Disabled"));
            break;

        case Message_SafetyDoorAjar:
            printPgmString(PSTR("Check Door"));
            break;

        case Message_CheckLimits:
            printPgmString(PSTR("Check Limits"));
            break;

        case Message_ProgramEnd:
            printPgmString(PSTR("Pgm End"));
            break;

        case Message_RestoreDefaults:
            printPgmString(PSTR("Restoring defaults"));
            break;

        case Message_SpindleRestore:
            printPgmString(PSTR("Restoring spindle"));
            break;

        case Message_SleepMode:
            printPgmString(PSTR("Sleeping"));
            break;
    }
    report_util_feedback_line_feed();
}


// Welcome message
void report_init_message ()
{
    printPgmString(PSTR("\r\nGrbl " GRBL_VERSION " ['$' for help]\r\n"));
}

// Grbl help message
void report_grbl_help () {
    printPgmString(PSTR("[HLP:$$ $# $G $I $N $x=val $Nx=line $J=line $SLP $C $X $H $B ~ ! ? ctrl-x]\r\n"));
}


// Grbl global settings print out.
// NOTE: The numbering scheme here must correlate to storing in settings.c
void report_grbl_settings() {

    // Print Grbl settings.
    report_util_uint8_setting(Setting_PulseMicroseconds, settings.pulse_microseconds);
    report_util_uint8_setting(Setting_StepperIdleLockTime, settings.stepper_idle_lock_time);
    report_util_uint8_setting(Setting_StepInvertMask, settings.step_invert_mask.value);
    report_util_uint8_setting(Setting_DirInvertMask, settings.dir_invert_mask.value);
    report_util_uint8_setting(Setting_InvertStepperEnable, settings.flags.invert_st_enable);
    report_util_uint8_setting(Setting_InvertLimitPins, settings.limit_invert_mask.value);
    report_util_uint8_setting(Setting_InvertProbePin, settings.flags.invert_probe_pin);
    report_util_uint8_setting(Setting_StatusReportMask, settings.status_report_mask.value);
    report_util_float_setting((uint8_t)Setting_JunctionDeviation, settings.junction_deviation, N_DECIMAL_SETTINGVALUE);
    report_util_float_setting((uint8_t)Setting_ArcTolerance, settings.arc_tolerance, N_DECIMAL_SETTINGVALUE);
    report_util_uint8_setting(Setting_ReportInches, settings.flags.report_inches);
    report_util_uint8_setting(Setting_SoftLimitsEnable, settings.flags.soft_limit_enable);
    report_util_uint8_setting(Setting_HardLimitsEnable, settings.flags.hard_limit_enable);
    report_util_uint8_setting(Setting_HomingEnable, settings.flags.homing_enable);
    report_util_uint8_setting(Setting_HomingDirMask, settings.homing_dir_mask);
    report_util_float_setting(Setting_HomingFeedRate, settings.homing_feed_rate, N_DECIMAL_SETTINGVALUE);
    report_util_float_setting(Setting_HomingSeekRate, settings.homing_seek_rate, N_DECIMAL_SETTINGVALUE);
    report_util_uint8_setting(Setting_HomingDebounceDelay, settings.homing_debounce_delay);
    report_util_float_setting(Setting_HomingPulloff, settings.homing_pulloff, N_DECIMAL_SETTINGVALUE);
    report_util_float_setting(Setting_RpmMax, settings.rpm_max, N_DECIMAL_RPMVALUE);
    report_util_float_setting(Setting_RpmMin, settings.rpm_min, N_DECIMAL_RPMVALUE);
  #ifdef VARIABLE_SPINDLE
    report_util_uint8_setting(Setting_LaserMode, settings.flags.laser_mode);
  #else
    report_util_uint8_setting(Setting_LaserMode, 0);
  #endif
    // Print axis settings
    uint8_t idx, set_idx;
    uint8_t val = (uint8_t)Setting_AxisSettingsBase;
    for (set_idx=0; set_idx<AXIS_N_SETTINGS; set_idx++) {

        for (idx=0; idx<N_AXIS; idx++) {

            switch ((axis_setting_type_t)set_idx) {

                case AxisSetting_StepsPerMM:
                    report_util_float_setting(val + idx, settings.steps_per_mm[idx], N_DECIMAL_SETTINGVALUE);
                    break;

                case AxisSetting_MaxRate:
                    report_util_float_setting(val + idx, settings.max_rate[idx], N_DECIMAL_SETTINGVALUE);
                    break;

                case AxisSetting_Acceleration:
                    report_util_float_setting(val + idx, settings.acceleration[idx] / (60.0f * 60.0f), N_DECIMAL_SETTINGVALUE);
                    break;

                case AxisSetting_MaxTravel:
                    report_util_float_setting(val + idx, -settings.max_travel[idx], N_DECIMAL_SETTINGVALUE);
                    break;
            }
        }
        val += AXIS_SETTINGS_INCREMENT;
    }
}


// Prints current probe parameters. Upon a probe command, these parameters are updated upon a
// successful probe or upon a failed probe with the G38.3 without errors command (if supported).
// These values are retained until Grbl is power-cycled, whereby they will be re-zeroed.
void report_probe_parameters ()
{
    // Report in terms of machine position.
    printPgmString(PSTR("[PRB:"));
    float print_position[N_AXIS];
    system_convert_array_steps_to_mpos(print_position, sys_probe_position);
    report_util_axis_values(print_position);
    serial_write(':');
    print_uint8_base10(sys.probe_succeeded);
    report_util_feedback_line_feed();
}


// Prints Grbl NGC parameters (coordinate offsets, probing)
void report_ngc_parameters ()
{
    float coord_data[N_AXIS];
    uint8_t coord_select;

    for (coord_select = 0; coord_select <= SETTING_INDEX_NCOORD; coord_select++) {

        if (!(settings_read_coord_data(coord_select, coord_data))) {
            report_status_message(Status_SettingReadFail);
            return;
        }

        printPgmString(PSTR("[G"));

        switch (coord_select) {

            case 6:
                printPgmString(PSTR("28"));
                break;
            case 7:
                printPgmString(PSTR("30"));
                break;
            default:
                print_uint8_base10(coord_select + 54);
                break; // G54-G59
        }
        serial_write(':');
        report_util_axis_values(coord_data);
        report_util_feedback_line_feed();
    }

    printPgmString(PSTR("[G92:")); // Print G92,G92.1 which are not persistent in memory
    report_util_axis_values(gc_state.coord_offset);
    report_util_feedback_line_feed();

    printPgmString(PSTR("[TLO:")); // Print tool length offset value
    printFloat_CoordValue(gc_state.tool_length_offset);
    report_util_feedback_line_feed();

    report_probe_parameters(); // Print probe parameters. Not persistent in memory.
}


// Print current gcode parser mode state
void report_gcode_modes ()
{
    printPgmString(PSTR("[GC:G"));
    if (gc_state.modal.motion >= MotionMode_ProbeToward) {
        printPgmString(PSTR("38."));
        print_uint8_base10(gc_state.modal.motion - (MotionMode_ProbeToward - 2));
    } else
        print_uint8_base10(gc_state.modal.motion);

    report_util_gcode_modes_G();
    print_uint8_base10(gc_state.modal.coord_select + 54);

    report_util_gcode_modes_G();
    print_uint8_base10(gc_state.modal.plane_select + 17);

    report_util_gcode_modes_G();
    print_uint8_base10(21 - gc_state.modal.units);

    report_util_gcode_modes_G();
    print_uint8_base10(gc_state.modal.distance + 90);

    report_util_gcode_modes_G();
    print_uint8_base10(94 - gc_state.modal.feed_rate);

    if (gc_state.modal.program_flow) {
        report_util_gcode_modes_M();
        switch (gc_state.modal.program_flow) {

            case ProgramFlow_Paused:
                serial_write('0');
                break;

  /*            case PROGRAM_FLOW_OPTIONAL_STOP: // M1 is ignored and not supported.
                serial_write('1');
                break; */

            case ProgramFlow_CompletedM2:
            case ProgramFlow_CompletedM30:
                print_uint8_base10((uint8_t)gc_state.modal.program_flow);
                break;
        }
    }

    report_util_gcode_modes_M();
    switch (gc_state.modal.spindle) {

        case SPINDLE_ENABLE_CW:
            serial_write('3'); break;

        case SPINDLE_ENABLE_CCW:
            serial_write('4');
            break;

        case SPINDLE_DISABLE:
            serial_write('5'); break;
    }

    report_util_gcode_modes_M();
    #ifdef ENABLE_M7
    if (gc_state.modal.coolant) { // Note: Multiple coolant states may be active at the same time.
        if (gc_state.modal.coolant & PL_COND_FLAG_COOLANT_MIST)
            serial_write('7');
        if (gc_state.modal.coolant & PL_COND_FLAG_COOLANT_FLOOD)
            serial_write('8');
    } else
        serial_write('9');
    #else
    if (gc_state.modal.coolant)
        serial_write('8');
    else
        serial_write('9');
    #endif

    #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
    if (sys.override_ctrl == OVERRIDE_PARKING_MOTION) {
        report_util_gcode_modes_M();
        print_uint8_base10(56);
    }
    #endif

    printPgmString(PSTR(" T"));
    print_uint8_base10(gc_state.tool);

    printPgmString(PSTR(" F"));
    printFloat_RateValue(gc_state.feed_rate);

  #ifdef VARIABLE_SPINDLE
    printPgmString(PSTR(" S"));
    printFloat(gc_state.spindle_speed,N_DECIMAL_RPMVALUE);
  #endif

    report_util_feedback_line_feed();
}

// Prints specified startup line
void report_startup_line (uint8_t n, char *line)
{
    printPgmString(PSTR("$N"));
    print_uint8_base10(n);
    serial_write('=');
    printString(line);
    report_util_line_feed();
}

void report_execute_startup_message (char *line, status_code_t status_code)
{
    serial_write('>');
    printString(line);
    serial_write(':');
    report_status_message(status_code);
}

// Prints build info line
void report_build_info(char *line)
{
    printPgmString(PSTR("[VER:" GRBL_VERSION "." GRBL_VERSION_BUILD ":"));
    printString(line);
    report_util_feedback_line_feed();
    printPgmString(PSTR("[OPT:")); // Generate compile-time build option list
  #ifdef VARIABLE_SPINDLE
    serial_write('V');
  #endif
  #ifdef USE_LINE_NUMBERS
    serial_write('N');
  #endif
  #ifdef ENABLE_M7
    serial_write('M');
  #endif
  #ifdef COREXY
    serial_write('C');
  #endif
  #ifdef PARKING_ENABLE
    serial_write('P');
  #endif
  #ifdef HOMING_FORCE_SET_ORIGIN
    serial_write('Z');
  #endif
  #ifdef HOMING_SINGLE_AXIS_COMMANDS
    serial_write('H');
  #endif
  #ifdef LIMITS_TWO_SWITCHES_ON_AXES
    serial_write('L');
  #endif
  #ifdef ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES
    serial_write('A');
  #endif
  #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
    serial_write('D');
  #endif
  #ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
    serial_write('0');
  #endif
  #ifdef ENABLE_SOFTWARE_DEBOUNCE
    serial_write('S');
  #endif
  #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
    serial_write('R');
  #endif
  #ifndef ENABLE_RESTORE_EEPROM_WIPE_ALL // NOTE: Shown when disabled.
    serial_write('*');
  #endif
  #ifndef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS // NOTE: Shown when disabled.
    serial_write('$');
  #endif
  #ifndef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS // NOTE: Shown when disabled.
    serial_write('#');
  #endif
  #ifndef ENABLE_BUILD_INFO_WRITE_COMMAND // NOTE: Shown when disabled.
    serial_write('I');
  #endif
  #ifndef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE // NOTE: Shown when disabled.
    serial_write('E');
  #endif
  #ifndef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE // NOTE: Shown when disabled.
    serial_write('W');
  #endif
  #ifndef HOMING_INIT_LOCK
    serial_write('L');
  #endif

  // NOTE: Compiled values, like override increments/max/min values, may be added at some point later.
  serial_write(',');
  print_uint8_base10(BLOCK_BUFFER_SIZE - 1);
  serial_write(',');
  print_uint32_base10(SERIAL_RX_BUFFER_SIZE);

  report_util_feedback_line_feed();
}


// Prints the character string line Grbl has received from the user, which has been pre-parsed,
// and has been sent into protocol_execute_line() routine to be executed by Grbl.
void report_echo_line_received (char *line)
{
    printPgmString(PSTR("[echo: "));
    printString(line);
    report_util_feedback_line_feed();
}


 // Prints real-time data. This function grabs a real-time snapshot of the stepper subprogram
 // and the actual location of the CNC machine. Users may change the following function to their
 // specific needs, but the desired real-time data report must be as short as possible. This is
 // requires as it minimizes the computational overhead and allows grbl to keep running smoothly,
 // especially during g-code programs with fast, short line segments and high frequency reports (5-20Hz).
void report_realtime_status ()
{
    uint8_t idx;
    int32_t current_position[N_AXIS]; // Copy current state of the system position variable
    float print_position[N_AXIS];

    memcpy(current_position, sys_position, sizeof(sys_position));
    system_convert_array_steps_to_mpos(print_position, current_position);

    // Report current machine state and sub-states
    serial_write('<');

    switch (sys.state) {

        case STATE_IDLE:
            printPgmString(PSTR("Idle"));
            break;

        case STATE_CYCLE:
            printPgmString(PSTR("Run"));
            break;

        case STATE_HOLD:
            if (!sys.suspend.jog_cancel) {
                printPgmString(PSTR("Hold:"));
                serial_write(sys.suspend.hold_complete ? '0' /*Ready to resume*/ : '1' /*Actively holding*/ );
                break;
            } // Continues to print jog state during jog cancel.

        case STATE_JOG:
            printPgmString(PSTR("Jog"));
            break;

        case STATE_HOMING:
            printPgmString(PSTR("Home"));
            break;

        case STATE_ALARM:
            printPgmString(PSTR("Alarm"));
            break;

        case STATE_CHECK_MODE:
            printPgmString(PSTR("Check"));
            break;

        case STATE_SAFETY_DOOR:
            printPgmString(PSTR("Door:"));
            if (sys.suspend.initiate_restore)
                serial_write('3'); // Restoring
            else {
                if (sys.suspend.retract_complete)
                    serial_write(sys.suspend.safety_door_ajar ? '1' /*Door ajar*/ : '0' /*Door closed and ready to resume*/);
                else
                    serial_write('2'); // Retracting
            }
            break;

        case STATE_SLEEP:
            printPgmString(PSTR("Sleep"));
            break;
    }

    float wco[N_AXIS];
    if (!settings.status_report_mask.position_type || sys.report_wco_counter == 0) {
        for (idx=0; idx< N_AXIS; idx++) {
            // Apply work coordinate offsets and tool length offset to current position.
            wco[idx] = gc_state.coord_system[idx] + gc_state.coord_offset[idx] + (idx == TOOL_LENGTH_OFFSET_AXIS ? gc_state.tool_length_offset : 0.0f);
            if (idx == TOOL_LENGTH_OFFSET_AXIS)
                wco[idx] += gc_state.tool_length_offset;
            if (!settings.status_report_mask.position_type)
                print_position[idx] -= (gc_state.coord_system[idx] + gc_state.coord_offset[idx] + (idx == TOOL_LENGTH_OFFSET_AXIS ? gc_state.tool_length_offset : 0.0f));
        }
    }

    // Report machine position
    if (settings.status_report_mask.position_type)
        printPgmString(PSTR("|MPos:"));
    else
        printPgmString(PSTR("|WPos:"));

    report_util_axis_values(print_position);

    // Returns planner and serial read buffer states.
  #ifdef REPORT_FIELD_BUFFER_STATE
    if (settings.status_report_mask.buffer_state) {
        printPgmString(PSTR("|Bf:"));
        print_uint8_base10(plan_get_block_buffer_available());
        serial_write(',');
        print_uint32_base10(serial_get_rx_buffer_available());
    }
  #endif

  #ifdef USE_LINE_NUMBERS
   #ifdef REPORT_FIELD_LINE_NUMBERS
    // Report current line number
    plan_block_t *cur_block = plan_get_current_block();
    if (cur_block != NULL) {
        uint32_t ln = cur_block->line_number;
        if (ln > 0) {
            printPgmString(PSTR("|Ln:"));
            printInteger(ln);
        }
    }
   #endif
  #endif

    // Report realtime feed speed
  #ifdef REPORT_FIELD_CURRENT_FEED_SPEED
   #ifdef VARIABLE_SPINDLE
    printPgmString(PSTR("|FS:"));
    printFloat_RateValue(st_get_realtime_rate());
    serial_write(',');
    printFloat(sys.spindle_speed,N_DECIMAL_RPMVALUE);
   #else
    printPgmString(PSTR("|F:"));
    printFloat_RateValue(st_get_realtime_rate());
   #endif
  #endif

  #ifdef REPORT_FIELD_PIN_STATE
    axes_signals_t lim_pin_state = (axes_signals_t)limits_get_state();
    control_signals_t ctrl_pin_state = system_control_get_state();
    uint8_t prb_pin_state = probe_get_state();

    if (lim_pin_state.value | ctrl_pin_state.value | prb_pin_state | sys.block_delete_enabled) {

        printPgmString(PSTR("|Pn:"));

        if (prb_pin_state)
            serial_write('P');

        if (lim_pin_state.value) {
            if(lim_pin_state.x)
                serial_write('X');
            if(lim_pin_state.y)
                serial_write('Y');
            if (lim_pin_state.z)
                serial_write('Z');
        }

        if (ctrl_pin_state.value) {
          #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
            if (ctrl_pin_state.safety_door)
                serial_write('D');
          #endif
            if (ctrl_pin_state.reset)
                serial_write('R');
            if (ctrl_pin_state.feed_hold)
                serial_write('H');
            if (ctrl_pin_state.cycle_start)
                serial_write('S');
        }

        if(sys.block_delete_enabled)
            serial_write('B');
    }
  #endif

  #ifdef REPORT_FIELD_WORK_COORD_OFFSET
    if (sys.report_wco_counter > 0)
        sys.report_wco_counter--;
    else {
        sys.report_wco_counter = sys.state & (STATE_HOMING | STATE_CYCLE | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR)
                                  ? (REPORT_WCO_REFRESH_BUSY_COUNT - 1) // Reset counter for slow refresh
                                  : (REPORT_WCO_REFRESH_IDLE_COUNT - 1);
        if (sys.report_ovr_counter == 0)
            sys.report_ovr_counter = 1; // Set override on next report.
        printPgmString(PSTR("|WCO:"));
        report_util_axis_values(wco);
    }
  #endif

  #ifdef REPORT_FIELD_OVERRIDES
    if (sys.report_ovr_counter > 0)
        sys.report_ovr_counter--;
    else {

        sys.report_ovr_counter = sys.state & (STATE_HOMING | STATE_CYCLE | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR)
                                  ? (REPORT_OVR_REFRESH_BUSY_COUNT - 1) // Reset counter for slow refresh
                                  : (REPORT_OVR_REFRESH_IDLE_COUNT - 1);

        printPgmString(PSTR("|Ov:"));
        print_uint8_base10(sys.f_override);
        serial_write(',');
        print_uint8_base10(sys.r_override);
        serial_write(',');
        print_uint8_base10(sys.spindle_speed_ovr);

        spindle_state_t sp_state = spindle_get_state();
        coolant_state_t cl_state = coolant_get_state();
        if (sp_state != Spindle_Off || cl_state.value) {

            printPgmString(PSTR("|A:"));

            if (sp_state != Spindle_Off) { // != SPINDLE_STATE_DISABLE
              #ifdef VARIABLE_SPINDLE
               #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
                serial_write('S'); // CW
               #else
                serial_write(sp_state == Spindle_CW ? 'S' /*CW*/ : 'C' /*CCW*/);
               #endif
              #else
                serial_write(sp_state == Spindle_CW ? 'S' /*CW*/ : 'C' /*CCW*/);
              #endif
            }
            if (cl_state.flood)
                serial_write('F');
          #ifdef ENABLE_M7
            if (cl_state.mist)
                serial_write('M');
          #endif
        }
    }
  #endif

    serial_write('>');
    report_util_line_feed();
}
