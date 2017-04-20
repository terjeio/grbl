/*
  coolant_control.h - spindle control methods
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

#ifndef coolant_control_h
#define coolant_control_h

//#define COOLANT_NO_SYNC     false
//#define COOLANT_FORCE_SYNC  true

#define COOLANT_STATE_DISABLE   0  // Must be zero

typedef union {
    uint8_t value;
    struct {
        uint8_t flood :1,
                mist  :1;
    };
} coolant_state_t;

#define coolant_get_state() hal.coolant_get_state()

// Immediately disables coolant pins.
// Directly called by coolant_init(), coolant_set_state(), and mc_reset(), which can be at
// an interrupt-level. No report flag set, but only called by routines that don't need it.
#define coolant_stop() hal.coolant_set_state(COOLANT_STATE_DISABLE)

// Sets the coolant pins according to state specified.
void coolant_set_state(uint8_t mode);

// G-code parser entry-point for setting coolant states. Checks for and executes additional conditions.
void coolant_sync(uint8_t mode);

#endif
