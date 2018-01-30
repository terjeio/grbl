### grblHAL specification rev 4

# ** INITIAL DRAFT **

A lot of details has to be added...

This HAL \(Hardware Abstraction Layer\) implementation uses a function pointer struct for interfacing the HAL driver to grbl proper.

Reference HAL implementations:

* TI MSP432 - a complete CMSIS \(bare metal\) driver with no extras.

* TI Tiva C (TM4C123) - a combined driverlib and bare metal driver with I2C keyboard for jogging etc., M-code extensions, example ATC \(Automatic Tool Changer\) code, ramped spindle PWM, driver specific settings stored in EEPROM and more.

Other implementations:

* Cypress PSoC - some parts of the code (typically signal inversions) are implemented in UDBs \("hardware"\).

* NXP LPC 1769 - LPC Expesso board. Note: Smoothieboard is not well suited for grbl as there is no interrupt capable GPIO pins available for control- and limit signals. TBC - compiles in GNU MCU Eclipse, but I do not yet have hardware available for testing.

* TI MSP430F5529 - a 16 bit processor with sufficient RAM/flash, just because it could be done...

Misc:

* A CNC BoosterPack for TI LaunchPads with the above MCUs are in the pipeline.

* Initial support for Trinamic StepStick drivers.

---

The restrictions imposed in the original implementation where signal groups could not share the same port, or span ports, does not apply if the driver can assure that no GPIO register read-write-modify \(RMW\) cycle can be interrupted. This can be achieved by disabling master interrupts \(not recommended\) or by utilizing bit-banding \(ARM MCUs\) wich is implicitly atomic. The downside by utilizing bit-banding is that output signals does not change in sync and it takes a bit longer to execute.

Mapping from the internal signal format \(bit consecutive\) to physical pins can be done via lookup tables, bit-shifting or bit-banding.

--- 

```c
typedef struct HAL {
    uint32_t version; 
    char *info;
    uint32_t f_step_timer;
    uint32_t rx_buffer_size;
    uint32_t spindle_pwm_off;

    bool (*driver_setup)(settings_t *settings);

    void (*limits_enable)(bool on);
    axes_signals_t (*limits_get_state)(void);
    void (*coolant_set_state)(coolant_state_t mode);
    coolant_state_t (*coolant_get_state)(void);
    void (*delay_milliseconds)(uint32_t ms, void (*callback)(void));

    bool (*probe_get_state)(void);
    void (*probe_configure_invert_mask)(bool is_probe_away);

    void (*spindle_set_status)(spindle_state_t state, float rpm, uint8_t spindle_speed_ovr);
    spindle_state_t (*spindle_get_state)(void);
    uint32_t (*spindle_set_speed)(uint32_t pwm_value);
    uint32_t (*spindle_compute_pwm_value)(float rpm, uint8_t spindle_speed_ovr);
    control_signals_t (*system_control_get_state)(void);

    void (*stepper_wake_up)(void);
    void (*stepper_go_idle)(void);
    void (*stepper_enable)(axes_signals_t enable);
    void (*stepper_set_outputs)(axes_signals_t step_outbits);
    void (*stepper_set_directions)(axes_signals_t dir_outbits);
    void (*stepper_cycles_per_tick)(uint32_t cycles_per_tick);
    void (*stepper_pulse_start)(axes_signals_t dir_outbits, axes_signals_t step_outbits, uint32_t spindle_pwm);

    uint16_t (*serial_get_rx_buffer_available)(void);
    bool (*serial_write)(char c);
    void (*serial_write_string)(const char *s);
    int16_t (*serial_read)(void);
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
    status_code_t (*userdefined_mcode_validate)(parser_block_t *gc_block, uint32_t *value_words);
    void (*userdefined_mcode_execute)(uint8_t state, parser_block_t *gc_block);
    void (*userdefined_rt_command_execute)(uint8_t cmd);
    bool (*get_position)(int32_t (*position)[N_AXIS]);
    void (*tool_select)(tool_data_t *tool);
    void (*tool_change)(parser_state_t *gc_state);
    void (*show_message)(const char *msg);
    bool (*driver_setting)(uint_fast16_t setting, float value);
    void (*driver_settings_restore)(uint8_t restore_flag);
    void (*driver_settings_report)(bool axis_settings);

    eeprom_io_t eeprom;

    // callbacks - set up by library before MCU init
    bool (*protocol_enqueue_gcode)(char *data);
    bool (*protocol_process_realtime)(char data);
    bool (*serial_blocking_callback)(void);
    void (*stepper_interrupt_callback)(void);
    void (*limit_interrupt_callback)(axes_signals_t state);
    void (*control_interrupt_callback)(control_signals_t signals);

    driver_cap_t driver_cap;
} HAL;
```

Note: the HAL struct is cleared on startup. All values defaults to 0 \(equal to null or false\).

#### The first part of the HAL struct contains some basic information about the HAL implementation.

```c
uint32_t version;
```
HAL version number, currently 4, used by HAL initialization for compatibility check - see below.
```c
char *info;
```
Optional info string, will be displayed as part of the $I message if supplied.
Example:
```c
hal.info = "myDriver";
```
```c
uint32_t f_step_timer;
```
Stepper timer clock frequency in Hz.
```c
uint32_t rx_buffer_size;
```
Serial receive buffer size in bytes.

```c
uint32_t spindle_pwm_off;
```
Spindle pwm off value, defaults to 0;

#### The following entry points are mandatory and are must set by the driver initialization code.
```c
bool (*driver_setup)(settings_t *settings);
```
Called by grbl after EEPROM settings has been loaded. Used for basic setup of the MCU peripherals.
Returns false if fails. Neg. Compatibility check...
```c
void (*limits_enable)(bool on);
```
Enable/disable limit switches interrupt.

```c
typedef union {
    uint8_t mask;
    uint8_t value;
    struct {
        uint8_t x :1,
                y :1,
                z :1,
                a :1,
                b :1,
                c :1;
    };
} axes_signals_t;
```
Axes signals union, used for step, dir, stepper enable and limits signals.
```c
axes_signals_t (*limits_get_state)(void);
```
Returns limit switches status in a axes_signals_t type.

```c
typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t flood     :1,
                mist      :1,
                reserved2 :1,
                reserved3 :1,
                reserved4 :1,
                reserved5 :1,
                reserved6 :1,
                reserved7 :1;
    };
} coolant_state_t;
```
Coolant state union.
```c
void (*coolant_set_state)(coolant_state_t mode);
```
Set/clear coolant outputs
```c
coolant_state_t (*coolant_get_state)(void);
```
Get coolant state
```c
void (*delay_milliseconds)(uint32_t ms, void (*callback)(void));
```
Delay for ms number of milliseconds. A pointer to an optional callback function may be supplied, this will be called on delay timeout and if provided the delay code returns immediately. Currently used by disable steppers function to avoid a long delay when the processor is in interrupt code.
```c
bool (*probe_get_state)(void);
```
Returns probe state.
```c
void (*probe_configure_invert_mask)(bool is_probe_away);
```
Sets probe signal inversion.

```c
typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t on        :1,
                ccw       :1,
                at_speed  :1,
                reserved3 :1,
                reserved4 :1,
                reserved5 :1,
                reserved6 :1,
                reserved7 :1;
    };
} spindle_state_t;
```
Spindle state union definition. at_speed is never inverted, true when spindle is at programmed speed.

```c
void (*spindle_set_status)(spindle_state_t state, float rpm, uint8_t spindle_speed_ovr);
```
Set spindle state, speed and speed override. A single entry point is used even if the HAL does not support setting the spindle speed.
```c
spindle_state_t (*spindle_get_state)(void);
```
Gets spindle state.
```c
uint32_t (*spindle_set_speed)(uint32_t pwm_value);
```
Sets spindle speed.
```c
uint32_t (*spindle_compute_pwm_value)(float rpm, uint8_t spindle_speed_ovr);
```
Returns spindle PWM value computed from given rpm and speed override.

```c
typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t reset               :1,
                feed_hold           :1,
                cycle_start         :1,
                safety_door_ajar    :1,
                block_delete        :1,
                stop_disable        :1, // M1
                safety_door_pending :1,
                deasserted          :1; // this flag is set if signals are deasserted. Note: do NOT pass on event to Grbl control_interrupt_handler if set.
    };
} control_signals_t;
```
Control signals union.
```c
control_signals_t (*system_control_get_state)(void);
```
Get control signals \(feed hold, cycle start, reset etc.\) state. 
```c
void (*stepper_wake_up)(void);
```
Enables the stepper drivers and enables the stepper driver interrupt timer.
```c
void (*stepper_go_idle)(void);
```
Disables the stepper driver interrupt timer.
```c
void (*stepper_enable)(axes_signals_t enable);
```
Enables/disables the stepper drivers. Use x-axis value if hardware does not support single axis enable/disable.
```c
void (*stepper_set_outputs)(axes_signals_t step_outbits);
```
Sets step pin outputs. Used by initialization code only.
```c
void (*stepper_set_directions)(axes_signals_t dir_outbits);
```
Sets direction pins. Used by initialization code only.
```c
void (*stepper_cycles_per_tick)(uint32_t cycles_per_tick);
```
Reloads the stepper timer interval.
```c
void (*stepper_pulse_start)(axes_signals_t dir_outbits, axes_signals_t step_outbits, uint32_t spindle_pwm);
```
Outputs the dir signals, step signals (possibly after a delay) and sets spindle RPM.
To reduce overhead two versions of this function may be provided, one to be used when a step pulse delay is configured and one if not. The driver code can then switch to the relevant code in the HAL entry point. 
```c
uint16_t (*serial_get_rx_buffer_available)(void);
```
Returns number of bytes available in the serial input buffer.
```c
bool (*serial_write)(char c);
```
Write a single character to the serial output.
```c
void (*serial_write_string)(const char *s);
```
Write a null terminated character string to the serial output.
```c
int16_t (*serial_read)(void);
```
Read a character from the serial input buffer. Returns -1 if empty.
```c
void (*serial_reset_read_buffer)(void);
```
Flushes the serial input buffer.
```c
void (*serial_cancel_read_buffer)(void);
```
Flushes the serial input buffer and adds a cancel (CAN) character. This character is used to flush the current input buffer (block buffer) for any pending commands. This allows for reliable jogging via keyboard keys - either via serial input or direct input.
```c
void (*set_bits_atomic)(volatile uint8_t *value, uint8_t bits);
```
Set bits atomically, disable master interrupt for reliable read-modify-write.  
```c
uint8_t (*clear_bits_atomic)(volatile uint8_t *value, uint8_t bits);
```
Clear bits atomically, disable master interrupt for reliable read-modify-write.
Returns current value. 
```c
uint8_t (*set_value_atomic)(volatile uint8_t *value, uint8_t bits);
```
The value atomically, disable master interrupt for reliable read-modify-write.
```c
void (*settings_changed)(settings_t *settings);
```
Called when settings are changed, use configure peripherals. 

#### Callbacks - used by the driver to inform grbl about asynchronous events

NOTE: These entry points are set by grbl.

```c
bool (*protocol_enqueue_gcode)(char *data);
```
Allows the driver to issue commands. Returns true when command is accepted, requires grbl to be in _idle_ or _jog_ state.
```c
bool (*protocol_process_realtime)(char data);
```
Allows the driver to issue real-time commands such as ovverides.
```c
bool (*serial_blocking_callback)(void);
```
Called when the serial transmit buffer is full and the send function is blocking.
```c
void (*stepper_interrupt_callback)(void);
```
Called on stepper timer timeout. May also be called by the wake up function to initiate a run.
```c
void (*limit_interrupt_callback)(axes_signals_t state);
```
Called when a limit interrupt occurs, possibly after debouncing has taken place.
```c
void (*control_interrupt_callback)(control_signals_t signals);
```
Called when a control interrupt signal occurs.

#### Optional entry points, may be unassigned (null)

```c
bool (*driver_release)(void);
```
Unlink grbl from HAL.
```c
void (*execute_realtime)(uint8_t state);
```
Called from main grbl loop, allows driver to do its own stuff \(multitasking switch\).
```c
uint8_t (*userdefined_mcode_check)(uint8_t mcode);
```
Precheck for user defined M code, return _mcode_ parameter if handled, 0 if not.
```c
status_code_t (*userdefined_mcode_validate)(parser_block_t *gc_block, uint32_t *value_words);
```
Parameter check for user defined M code. Return 0 \(_Status_OK_\) if valid, appropriate status code if not.
To force a sync before execution set _gc_block->user_defined_mcode_sync = true;_ before returning.
```c
void (*userdefined_mcode_execute)(uint8_t state, parser_block_t *gc_block);
```
Execute a user defined M code.
```c
void (*userdefined_rt_command_execute)(uint8_t cmd);
```
Entry point for user defined real-time commands.
```c
bool (*get_position)(int32_t (*position)[N_AXIS]);
```
Used for reading current machine coordinates, useful if grbl shares position with other motion control code.
```c
void (*tool_select)(tool_data_t *tool);
```
Called when a tool from the configured tool database range is selected \(Tn\). May be used to position a tool carousel before actual tool change.
```c
void (*tool_change)(parser_state_t *gc_state);
```
Called when a tool needs to be changed \(M6\).
```c
void (*show_message)(const char *msg);
```
Display a G code message string. \(MSG,<msg>\)
```c
bool (*driver_setting)(uint_fast16_t setting, float value);
```
Called when an unknown setting parameter is issued, return _true_ if the driver handles it. 
```c
void (*driver_settings_restore)(uint8_t restore_flag);
```
Called when parameter settings are to be restored to their default values.
```c
bool (*driver_settings_report)(bool axis_settings);
```
Report driver specific settings. Called two times on a settings report.
First at the end of non-axis settings, secondly after standard axis settings has been reported.

Persistent storage of parameter settings. Preferably this can be done to EEPROM but flash may also be employed. If neither is available default settings may be used and overridden.
The default is to enable EEPROM emulation, even if EEPROM storage is available, as this buffers any changes and writes them to peristent storage only when grbl is in _idle_ mode and not running a file.
```c
typedef enum {
    EEPROM_None = 0,
    EEPROM_Physical,
    EEPROM_Emulated
} eeprom_type;

typedef struct {
    uint16_t address;
    uint16_t size;
} eeprom_driver_area_t;

typedef struct {
    eeprom_type type;
    eeprom_driver_area_t driver_area;
    uint8_t (*get_byte)(uint32_t addr);
    void (*put_byte)(uint32_t addr, uint8_t new_value);
    void (*memcpy_to_with_checksum)(uint32_t destination, uint8_t *source, uint32_t size);
    bool (*memcpy_from_with_checksum)(uint8_t *destination, uint32_t source, uint32_t size);
    bool (*memcpy_from_flash)(uint8_t *dest);
    bool (*memcpy_to_flash)(uint8_t *source);
} eeprom_io_t;

eeprom_io_t eeprom;
```

```c
eeprom_type type;
```
```c
uint8_t (*get_byte)(uint32_t addr);
```
Required when _hal.eprom.type_ is set to _EEPROM_Physical_. Returns a byte read from physical EEPROM.
```c
void (*put_byte)(uint32_t addr, uint8_t new_value);
```
Required when _hal.eprom.type_ is set to _EEPROM_Physical_. Write a byte to physical EEPROM.
```c
void (*memcpy_to_with_checksum)(uint32_t destination, uint8_t *source, uint32_t size);
```
Required when _hal.eprom.type_ is set to _EEPROM_Physical_. Write bytes with added checksum to physical EEPROM.
```c
bool (*memcpy_from_with_checksum)(uint8_t *destination, uint32_t source, uint32_t size);
```
Required when _hal.eprom.type_ is set to _EEPROM_Physical_. Reads a block of bytes from physical EEPROM. Returns _true_ if checksum matches, _false_ otherwise.
```c
bool (*memcpy_from_flash)(uint8_t *dest);
```
Optional, used for reading the whole emulated "EEPROM" data from flash memory.
```c
bool (*memcpy_to_flash)(uint8_t *source);
```
Optional, used for writing the whole emulated "EEPROM" data to flash memory.

If driver specific settings are handled by the driver a "dirty" flag may be employed to handle write to persistent storage in the same manner as the standard settings. This requires EEPROM type set to emulated and the address (offset) and size of the driver specific data must be specified in the eeprom struct (eeprom.driver_area). Note that the size is ex. the checksum byte that is added by the standard code.

```c
typedef struct {
    bool is_dirty;
    bool global_settings;
    bool build_info;
    bool driver_settings;
    uint8_t startup_lines;
    uint16_t coord_data;
#ifdef N_TOOLS
    uint16_t tool_data;
#endif
} settings_dirty_t;
```
 
#### Driver capabilities

These bits are used to "negotiate" which features are required/enabled.
If a driver is not up to the task grbl will be halted and the message "_Grbl: incompatible driver_" will be sent to the serial output stream.

```c
typedef union {
    uint16_t value;
    struct {
        uint16_t mist_control            :1,
                 variable_spindle        :1,
                 safety_door             :1,
                 spindle_dir             :1,
                 software_debounce       :1,
                 step_pulse_delay        :1,
                 limits_pull_up          :1,
                 control_pull_up         :1,
                 probe_pull_up           :1,
                 amass_level             :2, // 0...3
                 stepper_current_control :1,
                 program_stop            :1,
                 spindle_at_speed        :1,
                 unused14                :1,
                 unused15                :1;
    };
} driver_cap_t;
```

2018-01-30
