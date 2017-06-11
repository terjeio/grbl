/*
  spindle_control.h - spindle control methods
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

#ifndef spindle_control_h
#define spindle_control_h

typedef union {
    uint8_t value;
    struct {
        uint8_t on        :1,
                ccw       :1,
				reserved2 :1,
				reserved3 :1,
				reserved4 :1,
				reserved5 :1,
				reserved6 :1,
				reserved7 :1;
    };
} spindle_state_t;

// used by driver
typedef struct {
	uint32_t period;
	uint32_t off_value;
	uint32_t min_value;
	uint32_t max_value;
	float pwm_gradient; // Precalulated value to speed up rpm to PWM conversions.
} spindle_pwm_t;

#define spindle_get_state() hal.spindle_get_state()

// Disables the spindle and sets PWM output to zero when PWM variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
#define spindle_stop() hal.spindle_set_status((spindle_state_t){0}, 0.0f, 0)

#define spindle_set_speed(s) hal.spindle_set_speed(s)
#define spindle_compute_pwm_value(s, o) hal.spindle_compute_pwm_value(s, o)

void spindle_set_override (uint8_t speed_ovr);

// Called by g-code parser when setting spindle state and requires a buffer sync.
// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by spindle_sync() after sync and parking motion/spindle stop override during restore.
#ifdef VARIABLE_SPINDLE

  // Called by g-code parser when setting spindle state and requires a buffer sync.
  void spindle_sync(spindle_state_t state, float rpm);

  // Sets spindle running state with direction, enable, and spindle PWM.
  void spindle_set_state(spindle_state_t state, float rpm);

#else

  // Called by g-code parser when setting spindle state and requires a buffer sync.
  #define spindle_sync(state, rpm) _spindle_sync(state)
  void _spindle_sync(spindle_state_t state);

  // Sets spindle running state with direction and enable.
  #define spindle_set_state(state, rpm) _spindle_set_state(state)
  void _spindle_set_state(spindle_state_t state);

#endif

#endif
