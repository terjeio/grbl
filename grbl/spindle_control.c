/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon
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

// Set spindle speed override
// NOTE: Unlike motion overrides, spindle overrides do not require a planner reinitialization.
void spindle_set_override (uint8_t speed_ovr)
{
	speed_ovr = max(min(speed_ovr, MAX_SPINDLE_SPEED_OVERRIDE), MIN_SPINDLE_SPEED_OVERRIDE);

    if (speed_ovr != sys.spindle_speed_ovr) {
        sys.step_control.update_spindle_pwm = on;
        sys.spindle_speed_ovr = speed_ovr;
        sys.report_ovr_counter = 0; // Set to report change immediately
    }
}

// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
#ifdef VARIABLE_SPINDLE
void spindle_set_state(spindle_state_t state, float rpm)
#else
void _spindle_set_state(spindle_state_t state)
#endif
{
    if (!sys.abort) { // Block during abort.

        if (!state.on) { // Halt or set spindle direction and rpm.
          #ifdef VARIABLE_SPINDLE
            sys.spindle_speed = 0.0f;
          #endif
            spindle_stop();
        } else {
          #ifdef VARIABLE_SPINDLE
          // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
            if (settings.flags.laser_mode && state.ccw)
                rpm = 0.0f; // TODO: May need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);

            spindle_set_speed(spindle_compute_pwm_value(rpm, sys.spindle_speed_ovr));
          #endif

          #if (defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && !defined(SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED)) || !defined(VARIABLE_SPINDLE)
            hal.spindle_set_status(state, 0.0f, DEFAULT_SPINDLE_SPEED_OVERRIDE);
          #endif
        }
        sys.report_ovr_counter = -1; // Set to report change immediately
    }
}

// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails
// if an abort or check-mode is active.
#ifdef VARIABLE_SPINDLE
    void spindle_sync(spindle_state_t state, float rpm)
    {
        if (sys.state != STATE_CHECK_MODE) {
            protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
            spindle_set_state(state, rpm);
        }
    }
#else
    void _spindle_sync(spindle_state_t state)
    {
        if (sys.state != STATE_CHECK_MODE) {
            protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
            _spindle_set_state(state);
        }
    }
#endif
