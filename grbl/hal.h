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

#define F_CPU 20000000 // really ticks per counter increment

//#define bit_true_atomic(var, bit) HWREGBITW(&var, bit) = 1;
//#define bit_false_atomic(var, bit) HWREGBITW(&var, bit) = 0;

typedef struct HAL {
	uint32_t version;
	uint32_t f_step_timer;
	void (*initMCU)(void);
	bool (*releaseMCU)(void);
    void (*limits_enable)(bool on);
	uint8_t (*limits_get_state)(void);
	void (*coolant_set_state)(uint8_t mode);
	uint8_t (*coolant_get_state)(void);
	void (*delay_ms)(uint16_t ms);
	void (*delay_us)(uint32_t us);
	bool (*probe_get_state)(void);
	void (*probe_configure_invert_mask)(bool is_probe_away);
	void (*spindle_set_state)(uint8_t state, float rpm);
	uint8_t (*spindle_get_state)(void);
	uint32_t (*spindle_set_speed)(uint32_t pwm_value);
	uint32_t (*spindle_compute_pwm_value)(float rpm);
	uint8_t (*system_control_get_state)(void);
	void (*stepper_wake_up)(uint8_t delay);
	void (*stepper_go_idle)(void);
	void (*stepper_enable)(bool on);
	void (*stepper_set_outputs)(uint8_t step_outbits);
	void (*stepper_set_directions)(uint8_t dir_outbits);
	void (*stepper_cycles_per_tick)(uint16_t prescaler, uint16_t cycles_per_tick);
	void (*stepper_pulse_start)(uint8_t dir_outbits, uint8_t step_outbits, uint32_t spindle_pwm);
	uint16_t (*serial_get_rx_buffer_size)(void);
	uint16_t (*serial_get_rx_buffer_available)(void);
	void (*serial_write)(uint8_t data);
	int32_t (*serial_read)(void);
	void (*serial_reset_read_buffer)(void);
	void (*serial_cancel_read_buffer)(void);
	unsigned char (*eeprom_get_char)(unsigned int addr);
	void (*eeprom_put_char)(unsigned int addr, unsigned char new_value);
	void (*memcpy_to_eeprom_with_checksum)(unsigned int destination, char *source, unsigned int size);
	int (*memcpy_from_eeprom_with_checksum)(char *destination, unsigned int source, unsigned int size);
	void (*set_bits_atomic)(volatile uint8_t *value, uint8_t bits);
	void (*clear_bits_atomic)(volatile uint8_t *value, uint8_t bits);
	void (*settings_changed)(settings_t *settings);
	bool (*unknown_mcode_handler)(uint8_t code, char *params);
	void (*execute_realtime)(uint8_t state);
	bool (*get_position)(int32_t (*position)[3]);
	// callbacks - set up by library before MCU init
	bool (*protocol_process_realtime)(int32_t data);
	void (*stepper_interrupt_callback)(void);
	void (*limit_interrupt_callback)(uint8_t state);
	void (*control_interrupt_callback)(uint8_t pins);
	bool (*protocol_enqueue_gcode)(char *data);
	bool hasEEPROM;
} HAL;

extern HAL hal;
extern bool driver_init (void);

#endif
