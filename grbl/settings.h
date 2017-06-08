/*
  settings.h - eeprom configuration handling
  Part of Grbl

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

#ifndef settings_h
#define settings_h

#include "grbl.h"
#include "system.h"

// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION 12  // NOTE: Check settings_reset() when moving to next version.

// Define settings restore bitflags.
#define SETTINGS_RESTORE_DEFAULTS bit(0)
#define SETTINGS_RESTORE_PARAMETERS bit(1)
#define SETTINGS_RESTORE_STARTUP_LINES bit(2)
#define SETTINGS_RESTORE_BUILD_INFO bit(3)
#ifndef SETTINGS_RESTORE_ALL
  #define SETTINGS_RESTORE_ALL 0xFF // All bitflags
#endif

// Define EEPROM memory address location values for Grbl settings and parameters
// NOTE: 1KB EEPROM is the minimum required. The upper half is reserved for parameters and
// the startup script. The lower half contains the global settings and space for future
// developments.
#define EEPROM_ADDR_GLOBAL         1U
#define EEPROM_ADDR_PARAMETERS     512U
#define EEPROM_ADDR_STARTUP_BLOCK  768U
#define EEPROM_ADDR_BUILD_INFO     942U

// Define EEPROM address indexing for coordinate parameters
#define N_COORDINATE_SYSTEM 6  // Number of supported work coordinate systems (from index 1)
#define SETTING_INDEX_NCOORD N_COORDINATE_SYSTEM+1 // Total number of system stored (from index 0)
// NOTE: Work coordinate indices are (0=G54, 1=G55, ... , 6=G59)
#define SETTING_INDEX_G28    N_COORDINATE_SYSTEM    // Home position 1
#define SETTING_INDEX_G30    N_COORDINATE_SYSTEM+1  // Home position 2
// #define SETTING_INDEX_G92    N_COORDINATE_SYSTEM+2  // Coordinate offset (G92.2,G92.3 not supported)

// Define Grbl axis settings numbering scheme. Starts at Setting_AxisSettingsBase, every INCREMENT, over N_SETTINGS.
// change AXIS_N_SETTINGS to 5 to enable stepper current settings
#define AXIS_N_SETTINGS          4
#define AXIS_SETTINGS_INCREMENT  10  // Must be greater than the number of axis settings

typedef enum {
    Setting_PulseMicroseconds = 0,
    Setting_StepperIdleLockTime = 1,
    Setting_StepInvertMask = 2,
    Setting_DirInvertMask = 3,
    Setting_InvertStepperEnable = 4,
    Setting_LimitPinsInvertMask = 5,
    Setting_InvertProbePin = 6,
    Setting_StatusReportMask = 10,
    Setting_JunctionDeviation = 11,
    Setting_ArcTolerance = 12,
    Setting_ReportInches = 13,
    Setting_ControlInvertMask = 14,
    Setting_CoolantInvertMask = 15,
    Setting_SpindleInvertMask = 16,
    Setting_ControlPullUpDisableMask = 17,
    Setting_LimitPullUpDisableMask = 18,
    Setting_ProbePullUpDisable = 19,
    Setting_SoftLimitsEnable = 20,
    Setting_HardLimitsEnable = 21,
    Setting_HomingEnable = 22,
    Setting_HomingDirMask = 23,
    Setting_HomingFeedRate = 24,
    Setting_HomingSeekRate = 25,
    Setting_HomingDebounceDelay = 26,
    Setting_HomingPulloff = 27,
    Setting_RpmMax = 30,
    Setting_RpmMin = 31,
    Setting_LaserMode = 32,
    Setting_PWMFreq = 33,
    Setting_PWMOffValue = 34,
    Setting_PWMMinValue = 35,
    Setting_PWMMaxValue = 36,
    Setting_AxisSettingsBase = 100 // NOTE: Reserving settings values >= 100 for axis settings. Up to 255.
} setting_type_t;

typedef enum {
    AxisSetting_StepsPerMM = 0,
    AxisSetting_MaxRate = 1,
    AxisSetting_Acceleration = 2,
    AxisSetting_MaxTravel = 3,
    AxisSetting_StepperCurrent = 4
} axis_setting_type_t;

typedef union {
    uint16_t value;
    struct {
        uint16_t report_inches     :1,
                 laser_mode        :1,
                 invert_st_enable  :1,
                 hard_limit_enable :1,
                 homing_enable     :1,
                 soft_limit_enable :1,
                 invert_probe_pin  :1,
                 spindle_disable_with_zero_speed :1,
                 disable_probe_pullup :1,
				 disable_M7           :1,
				 unassigned           :6;
    };
} settingflags_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t position_type     :1,
                buffer_state      :1,
				line_numbers      :1,
				feed_speed        :1,
				pin_state         :1,
				work_coord_offset :1,
				overrrides        :1,
				unassigned        :1;
    };
} reportmask_t;

// Global persistent settings (Stored from byte EEPROM_ADDR_GLOBAL onwards)
typedef struct {
    // Settings struct version
    uint32_t version;
    // Axis settings
    float steps_per_mm[N_AXIS];
    float max_rate[N_AXIS];
    float acceleration[N_AXIS];
    float max_travel[N_AXIS];
  #if AXIS_N_SETTINGS > 4
    float current[N_AXIS];
  #endif
    float junction_deviation;
    float arc_tolerance;
    float homing_feed_rate;
    float homing_seek_rate;
    float homing_pulloff;
    float rpm_max;
    float rpm_min;
    float spindle_pwm_freq;
    float spindle_pwm_period;
    float spindle_pwm_off_value;
    float spindle_pwm_min_value;
    float spindle_pwm_max_value;

    uint8_t stepper_idle_lock_time; // If max value 255, steppers do not disable.
    control_signals_t control_invert_mask;
    control_signals_t control_disable_pullup_mask;
    axes_signals_t limit_invert_mask;
    axes_signals_t limit_disable_pullup_mask;
    axes_signals_t step_invert_mask;
    axes_signals_t dir_invert_mask;
    coolant_state_t coolant_invert_mask;
    spindle_state_t spindle_invert_mask;
    uint8_t homing_dir_mask;
    uint16_t homing_debounce_delay;
    uint8_t pulse_microseconds;
    uint8_t pulse_delay_microseconds;
    reportmask_t status_report_mask; // Mask to indicate desired report data.
    settingflags_t flags;  // Contains default boolean settings

} settings_t;

extern settings_t settings;

// Initialize the configuration subsystem (load settings from EEPROM)
void settings_init();

// Helper function to clear and restore EEPROM defaults
void settings_restore(uint8_t restore_flag);

// A helper method to set new settings from command line
status_code_t settings_store_global_setting(uint8_t parameter, float value);

// Stores the protocol line variable as a startup line in EEPROM
void settings_store_startup_line(uint8_t n, char *line);

// Reads an EEPROM startup line to the protocol line variable
bool settings_read_startup_line(uint8_t n, char *line);

// Stores build info user-defined string
void settings_store_build_info(char *line);

// Reads build info user-defined string
bool settings_read_build_info(char *line);

// Writes selected coordinate data to EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data);

// Reads selected coordinate data from EEPROM
bool settings_read_coord_data(uint8_t coord_select, float *coord_data);

#endif
