/*
  settings.c - eeprom configuration handling
  Part of Grbl

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

settings_t settings;

// Method to store startup lines into EEPROM
void settings_store_startup_line (uint8_t n, char *line)
{
	if(hal.eeprom.type != EEPROM_None) {
		memcpy_to_eeprom_with_checksum(EEPROM_ADDR_STARTUP_BLOCK + n * (MAX_STORED_LINE_LENGTH + 1), (uint8_t *)line, MAX_STORED_LINE_LENGTH);
      #ifdef EMULATE_EEPROM
		if(hal.eeprom.type == EEPROM_Emulated)
		    settings_dirty.startup_lines[n] = settings_dirty.is_dirty = true;
      #endif
	}
}

// Method to store build info into EEPROM
void settings_store_build_info (char *line)
{
	if(hal.eeprom.type != EEPROM_None) {
		memcpy_to_eeprom_with_checksum(EEPROM_ADDR_BUILD_INFO, (uint8_t *)line, MAX_STORED_LINE_LENGTH);
      #ifdef EMULATE_EEPROM
        if(hal.eeprom.type == EEPROM_Emulated)
            settings_dirty.build_info = settings_dirty.is_dirty = true;
      #endif
    }
}

// Method to store coord data parameters into EEPROM
void settings_write_coord_data (uint8_t coord_select, float *coord_data)
{
	if(hal.eeprom.type != EEPROM_None) {
		memcpy_to_eeprom_with_checksum(EEPROM_ADDR_PARAMETERS + coord_select * (sizeof(float) * N_AXIS + 1), (uint8_t *)coord_data, sizeof(float) * N_AXIS);
      #ifdef EMULATE_EEPROM
        if(hal.eeprom.type == EEPROM_Emulated)
            settings_dirty.coord_data[coord_select] = settings_dirty.is_dirty = true;
      #endif
	}
}


// Method to store Grbl global settings struct and version number into EEPROM
void write_global_settings ()
{
	if(hal.eeprom.type != EEPROM_None) {
	    settings.version = SETTINGS_VERSION;
		eeprom_put_byte(0, SETTINGS_VERSION);
		memcpy_to_eeprom_with_checksum(EEPROM_ADDR_GLOBAL, (uint8_t *)&settings, sizeof(settings_t));
      #ifdef EMULATE_EEPROM
		if(hal.eeprom.type == EEPROM_Emulated)
            settings_dirty.global_settings = settings_dirty.is_dirty = true;
      #endif
	}
}


// Method to restore EEPROM-saved Grbl global settings back to defaults.
void settings_restore (uint8_t restore_flag) {

    if (restore_flag & SETTINGS_RESTORE_DEFAULTS) {

    	settings.version = SETTINGS_VERSION;

        settings.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS;
	    settings.stepper_idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME;
	    settings.step_invert_mask.value = DEFAULT_STEPPING_INVERT_MASK;
	    settings.dir_invert_mask.value = DEFAULT_DIRECTION_INVERT_MASK;
	    settings.status_report_mask.value = DEFAULT_STATUS_REPORT_MASK;
	    settings.junction_deviation = DEFAULT_JUNCTION_DEVIATION;
	    settings.arc_tolerance = DEFAULT_ARC_TOLERANCE;

	    settings.rpm_max = DEFAULT_SPINDLE_RPM_MAX;
	    settings.rpm_min = DEFAULT_SPINDLE_RPM_MIN;

	    settings.homing_dir_mask = DEFAULT_HOMING_DIR_MASK;
	    settings.homing_feed_rate = DEFAULT_HOMING_FEED_RATE;
	    settings.homing_seek_rate = DEFAULT_HOMING_SEEK_RATE;
	    settings.homing_debounce_delay = DEFAULT_HOMING_DEBOUNCE_DELAY;
	    settings.homing_pulloff = DEFAULT_HOMING_PULLOFF;

	    settings.flags.value = 0;

	    if (DEFAULT_REPORT_INCHES)
	        settings.flags.report_inches = on;

	    if (DEFAULT_LASER_MODE)
            settings.flags.laser_mode = on;

	    if (DEFAULT_INVERT_ST_ENABLE)
            settings.flags.invert_st_enable = on;

	    if (DEFAULT_HARD_LIMIT_ENABLE)
            settings.flags.hard_limit_enable = on;

	    if (DEFAULT_HOMING_ENABLE)
            settings.flags.homing_enable = on;

	    if (DEFAULT_SOFT_LIMIT_ENABLE)
            settings.flags.soft_limit_enable = on;

	    if (DEFAULT_INVERT_PROBE_PIN)
            settings.flags.invert_probe_pin = on;

	  #ifndef ENABLE_M7
        settings.flags.disable_M7 = on;
	  #endif

  	  #ifdef REPORT_FIELD_BUFFER_STATE
		settings.status_report_mask.buffer_state = on;
	  #endif

	  #ifdef REPORT_FIELD_LINE_NUMBERS
		settings.status_report_mask.line_numbers = on;
	  #endif

	  #ifdef REPORT_FIELD_CURRENT_FEED_SPEED
		settings.status_report_mask.feed_speed = on;
	  #endif

	  #ifdef REPORT_FIELD_PIN_STATE
		settings.status_report_mask.pin_state = on;
	  #endif

	  #ifdef REPORT_FIELD_WORK_COORD_OFFSET
		settings.status_report_mask.work_coord_offset = on;
	  #endif

	  #ifdef REPORT_FIELD_OVERRIDES
		settings.status_report_mask.overrrides = on;
	  #endif

      #ifdef INVERT_LIMIT_PIN_MASK
	    settings.limit_invert_mask.value = INVERT_LIMIT_PIN_MASK;
      #endif

	  #ifdef INVERT_CONTROL_PIN_MASK
		settings.control_invert_mask.value = INVERT_CONTROL_PIN_MASK;
	  #endif

	  #ifdef INVERT_SPINDLE_ENABLE_PIN
		settings.spindle_invert_mask.on = on;
      #endif

	  #ifdef DISABLE_LIMIT_PINS_PULL_UP_MASK
		settings.limit_disable_pullup_mask.value = DISABLE_LIMIT_PINS_PULL_UP_MASK;
	  #endif

	  #ifdef DISABLE_PROBE_PIN_PULL_UP
		settings.flags.disable_probe_pullup = on;
	  #endif

	  #ifdef DISABLE_CONTROL_PINS_PULL_UP_MASK
		settings.control_disable_pullup_mask.value = DISABLE_CONTROL_PINS_PULL_UP_MASK;
	  #endif

	    settings.steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM;
	    settings.steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM;
	    settings.steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM;
	    settings.max_rate[X_AXIS] = DEFAULT_X_MAX_RATE;
	    settings.max_rate[Y_AXIS] = DEFAULT_Y_MAX_RATE;
	    settings.max_rate[Z_AXIS] = DEFAULT_Z_MAX_RATE;
	    settings.acceleration[X_AXIS] = DEFAULT_X_ACCELERATION;
	    settings.acceleration[Y_AXIS] = DEFAULT_Y_ACCELERATION;
	    settings.acceleration[Z_AXIS] = DEFAULT_Z_ACCELERATION;
	    settings.max_travel[X_AXIS] = (-DEFAULT_X_MAX_TRAVEL);
	    settings.max_travel[Y_AXIS] = (-DEFAULT_Y_MAX_TRAVEL);
	    settings.max_travel[Z_AXIS] = (-DEFAULT_Z_MAX_TRAVEL);
	  #ifdef A_AXIS
		settings.steps_per_mm[A_AXIS] = DEFAULT_A_STEPS_PER_MM;
		settings.max_rate[A_AXIS] = DEFAULT_A_MAX_RATE;
		settings.acceleration[A_AXIS] = DEFAULT_A_ACCELERATION;
		settings.max_travel[A_AXIS] = (-DEFAULT_A_MAX_TRAVEL);
	  #endif
	  #ifdef B_AXIS
		settings.steps_per_mm[B_AXIS] = DEFAULT_B_STEPS_PER_MM;
		settings.max_rate[B_AXIS] = DEFAULT_B_MAX_RATE;
		settings.acceleration[B_AXIS] = DEFAULT_B_ACCELERATION;
		settings.max_travel[B_AXIS] = (-DEFAULT_B_MAX_TRAVEL);
	  #endif
	  #ifdef C_AXIS
		settings.steps_per_mm[C_AXIS] = DEFAULT_C_STEPS_PER_MM;
		settings.acceleration[C_AXIS] = DEFAULT_C_ACCELERATION;
		settings.max_rate[C_AXIS] = DEFAULT_C_MAX_RATE;
		settings.max_travel[C_AXIS] = (-DEFAULT_C_MAX_TRAVEL);
	  #endif

	  #if AXIS_N_SETTINGS > 4
	    settings.current[X_AXIS] = DEFAULT_X_CURRENT;
	    settings.current[Y_AXIS] = DEFAULT_Y_CURRENT;
	    settings.current[Z_AXIS] = DEFAULT_Z_CURRENT;
	  #endif

	    settings.spindle_pwm_freq = DEFAULT_SPINDLE_PWM_FREQ;
	    settings.spindle_pwm_off_value = DEFAULT_SPINDLE_PWM_OFF_VALUE;
	    settings.spindle_pwm_min_value = DEFAULT_SPINDLE_PWM_MIN_VALUE;
	    settings.spindle_pwm_max_value = DEFAULT_SPINDLE_PWM_MAX_VALUE;
	    settings.rpm_max = DEFAULT_SPINDLE_RPM_MAX;
	    settings.rpm_min = DEFAULT_SPINDLE_RPM_MIN;

	    write_global_settings();
    }

    if (restore_flag & SETTINGS_RESTORE_PARAMETERS) {
        uint32_t idx;
        float coord_data[N_AXIS];
        memset(&coord_data, 0, sizeof(coord_data));
        for (idx = 0; idx <= SETTING_INDEX_NCOORD; idx++)
            settings_write_coord_data(idx, coord_data);
    }

    if (hal.eeprom.type != EEPROM_None && (restore_flag & SETTINGS_RESTORE_STARTUP_LINES)) {
      #if N_STARTUP_LINE > 0
        eeprom_put_byte(EEPROM_ADDR_STARTUP_BLOCK, 0);
      #endif
      #if N_STARTUP_LINE > 1
        eeprom_put_byte(EEPROM_ADDR_STARTUP_BLOCK + (MAX_STORED_LINE_LENGTH + 1), 0);
      #endif
    }

    if (restore_flag & SETTINGS_RESTORE_BUILD_INFO && hal.eeprom.type != EEPROM_None)
        eeprom_put_byte(EEPROM_ADDR_BUILD_INFO , 0);
}

// Reads startup line from EEPROM. Updated pointed line string data.
bool settings_read_startup_line (uint8_t n, char *line)
{
    if (!(hal.eeprom.type != EEPROM_None && memcpy_from_eeprom_with_checksum((uint8_t *)line, EEPROM_ADDR_STARTUP_BLOCK + n * (MAX_STORED_LINE_LENGTH + 1), MAX_STORED_LINE_LENGTH))) {
        // Reset line with default value
        line[0] = 0; // Empty line
        settings_store_startup_line(n, line);
        return false;
    }
    return true;
}


// Reads startup line from EEPROM. Updated pointed line string data.
bool settings_read_build_info(char *line)
{
    if (!(hal.eeprom.type != EEPROM_None && memcpy_from_eeprom_with_checksum((uint8_t *)line, EEPROM_ADDR_BUILD_INFO, MAX_STORED_LINE_LENGTH))) {
        // Reset line with default value
        line[0] = 0; // Empty line
        settings_store_build_info(line);
        return false;
    }
    return true;
}


// Read selected coordinate data from EEPROM. Updates pointed coord_data value.
bool settings_read_coord_data (uint8_t coord_select, float *coord_data)
{
    if (!(hal.eeprom.type != EEPROM_None && memcpy_from_eeprom_with_checksum((uint8_t *)coord_data, EEPROM_ADDR_PARAMETERS + coord_select * (sizeof(float) * N_AXIS + 1), sizeof(float) * N_AXIS))) {
        // Reset with default zero vector
        clear_vector_float(coord_data);
        settings_write_coord_data(coord_select, coord_data);
        return false;
    }
    return true;
}

// Reads Grbl global settings struct from EEPROM.
bool read_global_settings () {
    // Check version-byte of eeprom
    return hal.eeprom.type != EEPROM_None && SETTINGS_VERSION == eeprom_get_byte(0) && memcpy_from_eeprom_with_checksum((uint8_t *)&settings, EEPROM_ADDR_GLOBAL, sizeof(settings_t));
}

// A helper method to set settings from command line
status_code_t settings_store_global_setting (uint8_t parameter, float value) {

    if (value < 0.0f)
        return Status_NegativeValue;

    if ((setting_type_t)parameter >= Setting_AxisSettingsBase) {
        // Store axis configuration. Axis numbering sequence set by AXIS_SETTING defines.
        // NOTE: Ensure the setting index corresponds to the report.c settings printout.
        parameter -= (uint8_t)Setting_AxisSettingsBase;
        uint8_t set_idx = 0;

        while (set_idx < AXIS_N_SETTINGS) {

            if (parameter < N_AXIS) {
            // Valid axis setting found.
                switch ((axis_setting_type_t)set_idx) {

                    case AxisSetting_StepsPerMM:
                        #ifdef MAX_STEP_RATE_HZ
                        if (value * settings.max_rate[parameter] > (MAX_STEP_RATE_HZ * 60.0f))
                            return Status_MaxStepRateExceeded;
                        #endif
                        settings.steps_per_mm[parameter] = value;
                        break;

                    case AxisSetting_MaxRate:
                        #ifdef MAX_STEP_RATE_HZ
                        if (value * settings.steps_per_mm[parameter] > (MAX_STEP_RATE_HZ * 60.0f))
                            return Status_MaxStepRateExceeded;
                        #endif
                        settings.max_rate[parameter] = value;
                        break;

                    case AxisSetting_Acceleration:
                        settings.acceleration[parameter] = value * 60.0f * 60.0f; // Convert to mm/min^2 for grbl internal use.
                        break;

                    case AxisSetting_MaxTravel:
                        settings.max_travel[parameter] = -value; // Store as negative for grbl internal use.
                        break;

				  #if AXIS_N_SETTINGS > 4
                    case AxisSetting_StepperCurrent:
                    	settings.current[parameter] = value;
                    	break;
				  #endif

                    default: // for stopping compiler warning
                    	break;

                }
                break; // Exit while-loop after setting has been configured and proceed to the EEPROM write call.

            } else {
                set_idx++;
                // If axis index greater than N_AXIS or setting index greater than number of axis settings, error out.
                if ((parameter < AXIS_SETTINGS_INCREMENT) || (set_idx == AXIS_N_SETTINGS))
                    return Status_InvalidStatement;
                parameter -= AXIS_SETTINGS_INCREMENT;
            }
        }
    } else {
        // Store non-axis Grbl settings
        uint8_t int_value = (uint8_t)truncf(value);
        switch((setting_type_t)parameter) {

            case Setting_PulseMicroseconds:
                if (int_value < 3)
                    return Status_SettingStepPulseMin;
                settings.pulse_microseconds = int_value;
                break;

            case Setting_StepperIdleLockTime:
                settings.stepper_idle_lock_time = int_value;
                break;

            case Setting_StepInvertMask:
                settings.step_invert_mask.value = int_value;
                break;

            case Setting_DirInvertMask:
                settings.dir_invert_mask.value = int_value;
                break;

            case Setting_InvertStepperEnable: // Reset to ensure change. Immediate re-init may cause problems.
                settings.flags.invert_st_enable = int_value != 0;
                break;

            case Setting_LimitPinsInvertMask: // Reset to ensure change. Immediate re-init may cause problems.
                settings.limit_invert_mask.value = int_value;
                break;

            case Setting_InvertProbePin: // Reset to ensure change. Immediate re-init may cause problems.
                settings.flags.invert_probe_pin = int_value !=0;
                probe_configure_invert_mask(false);
                break;

            case Setting_StatusReportMask:
                settings.status_report_mask.value = int_value;
                break;

            case Setting_JunctionDeviation:
                settings.junction_deviation = value;
                break;

            case Setting_ArcTolerance:
                settings.arc_tolerance = value;
                break;

            case Setting_ReportInches:
                settings.flags.report_inches = int_value != 0;
                system_flag_wco_change(); // Make sure WCO is immediately updated.
                break;

            case Setting_ControlInvertMask:
                settings.control_invert_mask.value = int_value;
                break;

            case Setting_CoolantInvertMask:
                settings.coolant_invert_mask.value = int_value;
                break;

            case Setting_SpindleInvertMask:
                settings.spindle_invert_mask.value = int_value;
                break;

            case Setting_ControlPullUpDisableMask:
				settings.control_disable_pullup_mask.value = int_value & 0x0F;
				break;

            case Setting_LimitPullUpDisableMask:
				settings.limit_disable_pullup_mask.value = int_value;
				break;

			case Setting_ProbePullUpDisable:
				settings.flags.disable_probe_pullup = int_value != 0;
				break;

			case Setting_SoftLimitsEnable:
                if (int_value && !settings.flags.homing_enable)
                    return Status_SoftLimitError;
                settings.flags.soft_limit_enable = int_value != 0;
                break;

            case Setting_HardLimitsEnable:
                settings.flags.hard_limit_enable = int_value != 0;
                limits_init(); // Re-init to immediately change. NOTE: Nice to have but could be problematic later.
                break;

            case Setting_HomingEnable:
                settings.flags.homing_enable = int_value != 0;
                if (!int_value)
                    settings.flags.soft_limit_enable = 0; // Force disable soft-limits.
                break;

            case Setting_HomingDirMask:
                settings.homing_dir_mask = int_value;
                break;

            case Setting_HomingFeedRate:
                settings.homing_feed_rate = value;
                break;

            case Setting_HomingSeekRate:
                settings.homing_seek_rate = value;
                break;

            case Setting_HomingDebounceDelay:
                settings.homing_debounce_delay = int_value;
                break;

            case Setting_HomingPulloff:
                settings.homing_pulloff = value;
                break;

            case Setting_RpmMax:
                settings.rpm_max = value;
                // spindle_init();
                break; // Re-initialize spindle rpm calibration

            case Setting_RpmMin:
                settings.rpm_min = value;
                // spindle_init();
                break; // Re-initialize spindle rpm calibration

            case Setting_LaserMode:
              #ifdef VARIABLE_SPINDLE
                settings.flags.laser_mode = int_value;
              #else
                return Status_SettingDisabledLaser;
              #endif
                break;

            case Setting_PWMFreq:
            	settings.spindle_pwm_freq = value;
//            	spindle_init();
            	break; // Re-initialize spindle pwm calibration

            case Setting_PWMOffValue:
            	settings.spindle_pwm_off_value = value;
//            	spindle_init();
            	break; // Re-initialize spindle pwm calibration

            case Setting_PWMMinValue:
            	settings.spindle_pwm_min_value = value;
//            	spindle_init();
            	break; // Re-initialize spindle pwm calibration

            case Setting_PWMMaxValue:
            	settings.spindle_pwm_max_value = value;
//            	spindle_init();
            	break; // Re-initialize spindle pwm calibration

            default:
                return Status_InvalidStatement;
        }
    }

    write_global_settings();
    hal.settings_changed(&settings);

    return Status_OK;
}

// Initialize the config subsystem
void settings_init() {
    if(!read_global_settings()) {
        report_status_message(Status_SettingReadFail);
        settings_restore(SETTINGS_RESTORE_ALL); // Force restore all EEPROM data.
        report_grbl_settings();
    } else
        hal.settings_changed(&settings);
}
