/*
  hal.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver interface definition for Hardware Abstration Layer

  Part of Grbl

  Copyright (c) 2016-2017 Terje Io
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

#ifndef __HAL__
#define __HAL__

#include <stdint.h>
#include <stdbool.h>
#include "gcode.h"
#include "system.h"
#include "coolant_control.h"
#include "spindle_control.h"

#define F_STEPTIMER 20000000 // stepper ISR timer clock frequency TODO: use hal.f_step_timer?

//#define bit_true_atomic(var, bit) HWREGBITW(&var, bit) = 1;
//#define bit_false_atomic(var, bit) HWREGBITW(&var, bit) = 0;

typedef enum {
    EEPROM_None = 0,
    EEPROM_Physical,
    EEPROM_Emulated
} eeprom_type;

// driver capabilities, to be set by driver in driver_init(), flags may be cleared after to switch off option
typedef union {
    uint16_t value;
    struct {
        uint16_t mist_control      :1,
                 variable_spindle  :1,
                 safety_door       :1,
                 spindle_dir       :1,
                 software_debounce :1,
                 step_pulse_delay  :1,
                 limits_pull_up    :1,
                 control_pull_up   :1,
                 probe_pull_up     :1,
                 amass_level       :2; // 0...3
    };
} driver_cap_t;

typedef struct {
    eeprom_type type;
    unsigned char (*get_char)(unsigned int addr);
    void (*put_char)(unsigned int addr, unsigned char new_value);
    void (*memcpy_to_with_checksum)(unsigned int destination, char *source, unsigned int size);
    int (*memcpy_from_with_checksum)(char *destination, unsigned int source, unsigned int size);
} eeprom_io_t;

typedef struct HAL {
	uint32_t version;
	uint32_t f_step_timer;
	uint32_t rx_buffer_size;
	uint32_t spindle_pwm_off;

	bool (*driver_setup)(void);

	void (*limits_enable)(bool on);
    axes_signals_t (*limits_get_state)(void);
	void (*coolant_set_state)(coolant_state_t mode);
	coolant_state_t (*coolant_get_state)(void);
	void (*delay_ms)(uint16_t ms);
	void (*delay_us)(uint32_t us);

	bool (*probe_get_state)(void);
	void (*probe_configure_invert_mask)(bool is_probe_away);

	void (*spindle_set_status)(spindle_state_t state, float rpm, uint8_t spindle_speed_ovr);
	spindle_state_t (*spindle_get_state)(void);
	uint32_t (*spindle_set_speed)(uint32_t pwm_value);
	uint32_t (*spindle_compute_pwm_value)(float rpm, uint8_t spindle_speed_ovr);
	control_signals_t (*system_control_get_state)(void);

	void (*stepper_wake_up)(void);
	void (*stepper_go_idle)(void);
	void (*stepper_enable)(bool on);
	void (*stepper_set_outputs)(axes_signals_t step_outbits);
	void (*stepper_set_directions)(axes_signals_t dir_outbits);
	void (*stepper_cycles_per_tick)(uint32_t cycles_per_tick);
	void (*stepper_pulse_start)(axes_signals_t dir_outbits, axes_signals_t step_outbits, uint32_t spindle_pwm);

	uint16_t (*serial_get_rx_buffer_available)(void);
	void (*serial_write)(uint8_t data);
    void (*serial_write_string)(const char *s);
	int32_t (*serial_read)(void);
	void (*serial_reset_read_buffer)(void);
	void (*serial_cancel_read_buffer)(void);

	void (*set_bits_atomic)(volatile uint8_t *value, uint8_t bits);
	uint8_t (*clear_bits_atomic)(volatile uint8_t *value, uint8_t bits);
    uint8_t (*set_value_atomic)(volatile uint8_t *value, uint8_t bits);

	void (*settings_changed)(settings_t *settings);

	// optional entry points, may be unassigned (null)
    bool (*driver_release)(void);
    void (*execute_realtime)(uint8_t state);
	uint8_t (*userdefined_mcode_check)(uint8_t mcode);
	status_code_t (*userdefined_mcode_validate)(parser_block_t *gc_block, uint16_t *value_words);
    void (*userdefined_mcode_execute)(uint8_t state, parser_block_t *gc_block);
    void (*userdefined_rt_command_execute)(uint8_t cmd);
    bool (*get_position)(int32_t (*position)[3]);
    eeprom_io_t eeprom;

	// callbacks - set up by library before MCU init
    bool (*protocol_enqueue_gcode)(char *data);
	bool (*protocol_process_realtime)(int32_t data);
	void (*stepper_interrupt_callback)(void);
	void (*limit_interrupt_callback)(axes_signals_t state);
	void (*control_interrupt_callback)(control_signals_t signals);

	driver_cap_t driver_cap;
} HAL;

extern HAL hal;
extern bool driver_init (void);

#endif
