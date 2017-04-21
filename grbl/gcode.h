/*
  gcode.h - rs274/ngc parser.
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

#ifndef gcode_h
#define gcode_h

#include "system.h"

//TODO: untangle triple definition of spindle and coolant modes?
// Modal Group M7: Spindle control
#define SPINDLE_DISABLE 0 // M5 (Default: Must be zero)
#define SPINDLE_ENABLE_CW   PL_COND_FLAG_SPINDLE_CW // M3 (NOTE: Uses planner condition bit flag)
#define SPINDLE_ENABLE_CCW  PL_COND_FLAG_SPINDLE_CCW // M4 (NOTE: Uses planner condition bit flag)

// Modal Group M8: Coolant control
#define COOLANT_DISABLE 0 // M9 (Default: Must be zero)
#define COOLANT_FLOOD_ENABLE  PL_COND_FLAG_COOLANT_FLOOD // M8 (NOTE: Uses planner condition bit flag)
#define COOLANT_MIST_ENABLE   PL_COND_FLAG_COOLANT_MIST  // M7 (NOTE: Uses planner condition bit flag)

// Define modal group internal numbers for checking multiple command violations and tracking the
// type of command that is called in the block. A modal group is a group of g-code commands that are
// mutually exclusive, or cannot exist on the same line, because they each toggle a state or execute
// a unique motion. These are defined in the NIST RS274-NGC v3 g-code standard, available online,
// and are similar/identical to other g-code interpreters by manufacturers (Haas,Fanuc,Mazak,etc).
// NOTE: Modal group define values must be sequential and starting from zero.
typedef enum {
    ModalGroup_G0 = 0,  // [G4,G10,G28,G28.1,G30,G30.1,G53,G92,G92.1] Non-modal
    ModalGroup_G1,      // [G0,G1,G2,G3,G38.2,G38.3,G38.4,G38.5,G80] Motion
    ModalGroup_G2,      // [G17,G18,G19] Plane selection
    ModalGroup_G3,      // [G90,G91] Distance mode
    ModalGroup_G4,      // [G91.1] Arc IJK distance mode
    ModalGroup_G5,      // [G93,G94] Feed rate mode
    ModalGroup_G6,      // [G20,G21] Units
    ModalGroup_G7,      // [G40] Cutter radius compensation mode. G41/42 NOT SUPPORTED.
    ModalGroup_G8,      // [G43.1,G49] Tool length offset
    ModalGroup_G12,     // [G54,G55,G56,G57,G58,G59] Coordinate system selection
    ModalGroup_G13,     // [G61] Control mode

    ModalGroup_M4,      // [M0,M1,M2,M30] Stopping
    ModalGroup_M7,      // [M3,M4,M5] Spindle turning
    ModalGroup_M8,      // [M7,M8,M9] Coolant control
    ModalGroup_M9,      // [M56] Override control
} modal_group_t;

// Define parameter word mapping.
typedef enum {
    Word_F = 0,
    Word_I,
    Word_J,
    Word_K,
    Word_L,
    Word_N,
    Word_P,
    Word_R,
    Word_S,
    Word_T,
    Word_X,
    Word_Y,
    Word_Z,
    Word_Q,
} parameter_word_t;

typedef union {
    parameter_word_t parameter;
    modal_group_t group;
} word_bit_t;

// Define command actions for within execution-type modal groups (motion, stopping, non-modal). Used
// internally by the parser to know which command to execute.
// NOTE: Some macro values are assigned specific values to make g-code state reporting and parsing
// compile a litte smaller. Necessary due to being completely out of flash on the 328p. Although not
// ideal, just be careful with values that state 'do not alter' and check both report.c and gcode.c
// to see how they are used, if you need to alter them.

// Modal Group G0: Non-modal actions
typedef enum {
    NonModal_NoAction = 0,                  // (Default: Must be zero)
    NonModal_Dwell = 4,                     // G4 (Do not alter value)
    NonModal_SetCoordinateData = 10,        // G10 (Do not alter value)
    NonModal_GoHome_0 = 28,                 // G28 (Do not alter value)
    NonModal_SetHome_0 = 38,                // G28.1 (Do not alter value)
    NonModal_GoHome_1 = 30,                 // G30 (Do not alter value)
    NonModal_SetHome_1 = 40,                // G30.1 (Do not alter value)
    NonModal_AbsoluteOverride= 53,          // G53 (Do not alter value)
    NonModal_SetCoordinateOffset = 92,      // G92 (Do not alter value)
    NonModal_ResetCoordinateOffset = 102,   //G92.1 (Do not alter value)
    NonModal_UserDefinedMCode = 200,        // Mx (Do not alter value)
} non_modal_t;

// Modal Group G1: Motion modes
typedef enum {
    MotionMode_Seek = 0,                    // G0 (Default: Must be zero)
    MotionMode_Linear = 1,                  // G1 (Do not alter value)
    MotionMode_CwArc = 2,                   // G2 (Do not alter value)
    MotionMode_CcwArc = 3,                  // G3 (Do not alter value)
    MotionMode_ProbeToward = 140,           // G38.2 (Do not alter value)
    MotionMode_ProbeTowardNoError = 141,    // G38.3 (Do not alter value)
    MotionMode_ProbeAway = 142,             // G38.4 (Do not alter value)
    MotionMode_ProbeAwayNoError = 143,      // G38.5 (Do not alter value)
    MotionMode_None = 80                    // G80 (Do not alter value)
} motion_mode_t;

// Modal Group G2: Plane select
typedef enum {
    PlaneSelect_XY = 0, // G17 (Default: Must be zero)
    PlaneSelect_ZX = 1, // G18 (Do not alter value)
    PlaneSelect_YZ = 2  // G19 (Do not alter value)
} plane_select_t;

// Modal Group G3: Distance mode
typedef enum {
    DistanceMode_Absolute    = 0,   // G90 (Default: Must be zero)
    DistanceMode_Incremental = 1    // G91 (Do not alter value)
} distance_mode_t;

// Modal Group G4: Arc IJK distance mode
//#define DISTANCE_ARC_MODE_INCREMENTAL 0 // G91.1 (Default: Must be zero)

// Modal Group M4: Program flow
typedef enum {
    ProgramFlow_Running = 0,        // (Default: Must be zero)
    ProgramFlow_Paused  = 3,        // M0
    ProgramFlow_OptionalStop = 1,   // M1 NOTE: Not supported, but valid and ignored.
    ProgramFlow_CompletedM2 = 2,    // M2 (Do not alter value)
    ProgramFlow_CompletedM30 = 30   // M30 (Do not alter value)
} program_flow_t;

// Modal Group G5: Feed rate mode
typedef enum {
    FeedMode_UnitsPerMin = 0,   // G94 (Default: Must be zero)
    FeedMode_InverseTime = 1    // G93 (Do not alter value)
} feed_mode_t;

// Modal Group G6: Units mode
typedef enum {
    UnitsMode_MM = 0,       // G21 (Default: Must be zero)
    UnitsMode_Inches = 1    // G20 (Do not alter value)
} units_mode_t;

// Modal Group G7: Cutter radius compensation mode
//#define CUTTER_COMP_DISABLE 0 // G40 (Default: Must be zero)

// Modal Group G13: Control mode
//#define CONTROL_MODE_EXACT_PATH 0 // G61 (Default: Must be zero)

// Modal Group G8: Tool length offset
typedef enum {
    ToolLengthOffset_Cancel = 0,        // G49 (Default: Must be zero)
    ToolLengthOffset_EnableDynamic = 1  // G43.1
} tool_length_offset_t;

// Modal Group M9: Override control
#ifdef DEACTIVATE_PARKING_UPON_INIT
    typedef enum {
        ParkingOverride_Disabled = 0,   // G49 (Default: Must be zero)
        ParkingOverride_Motion = 1      // G43.1
    } parking_override_t;

//  #define OVERRIDE_DISABLED  0 // (Default: Must be zero)
//  #define OVERRIDE_PARKING_MOTION 1 // M56
#else
    typedef enum {
        ParkingOverride_Disabled = 1,   // Parking disabled.
        ParkingOverride_Motion = 0      // M56 (Default: Must be zero)
    } parking_override_t;
//  #define OVERRIDE_PARKING_MOTION 0 // M56 (Default: Must be zero)
//  #define OVERRIDE_DISABLED  1 // Parking disabled.
#endif

// Modal Group G12: Active work coordinate system
// N/A: Stores coordinate system value (54-59) to change to.

// Define g-code parser position updating flags
typedef enum {
    GCUpdatePos_Target = 0,
    GCUpdatePos_System,
    GCUpdatePos_None
} pos_update_t;

// Define probe cycle exit states and assign proper position updating.
typedef enum {
    GCProbe_Found = GCUpdatePos_System,
    GCProbe_Abort = GCUpdatePos_None,
    GCProbe_FailInit = GCUpdatePos_None,
    GCProbe_FailEnd = GCUpdatePos_Target,
  #ifdef SET_CHECK_MODE_PROBE_TO_START
    GCProbe_CheckMode = GCUpdatePos_None
  #else
    GCProbe_CheckMode = GCUpdatePos_Target
  #endif
} gc_probe_t;

// Define gcode parser flags for handling special cases.

typedef union {
    uint8_t value;
    struct {
        uint8_t jog_motion        :1,
                check_mantissa    :1,
                arc_is_clockwise  :1,
                probe_is_away     :1,
                probe_is_no_error :1,
                laser_force_sync  :1,
                laser_disable     :1,
                laser_is_motion   :1;
    };
} gc_parser_flags_t;

// NOTE: When this struct is zeroed, the above defines set the defaults for the system.
typedef struct {
    motion_mode_t motion;           // {G0,G1,G2,G3,G38.2,G80}
    feed_mode_t feed_rate;          // {G93,G94}
    units_mode_t units;             // {G20,G21}
    distance_mode_t distance;       // {G90,G91}
    // uint8_t distance_arc;        // {G91.1} NOTE: Don't track. Only default supported.
    plane_select_t plane_select;    // {G17,G18,G19}
    // uint8_t cutter_comp;         // {G40} NOTE: Don't track. Only default supported.
    tool_length_offset_t tool_length;   // {G43.1,G49}
    uint8_t coord_select;           // {G54,G55,G56,G57,G58,G59}
    // uint8_t control;             // {G61} NOTE: Don't track. Only default supported.
    program_flow_t program_flow;    // {M0,M1,M2,M30}
    uint8_t coolant;                // {M7,M8,M9}
    uint8_t spindle;                // {M3,M4,M5}
    parking_override_t override;    // {M56}
} gc_modal_t;

typedef struct {
    float f;         // Feed
    float ijk[3];    // I,J,K Axis arc offsets
    float p;         // G10 or dwell parameters
    float q;         // User defined M-code parameter (G82 peck drilling, not supported)
    float r;         // Arc radius
    float s;         // Spindle speed
    float xyz[N_AXIS]; // X,Y,Z Translational axes
    float coord_data[N_AXIS]; // Coordinate data
    int32_t n;       // Line number
    uint8_t t;       // Tool selection
    uint8_t l;       // G10 or canned cycles parameters
} gc_values_t;


typedef struct {
    gc_modal_t modal;

    float spindle_speed;          // RPM
    float feed_rate;              // Millimeters/min
    float position[N_AXIS];       // Where the interpreter considers the tool to be at this point in the code

    float coord_system[N_AXIS];    // Current work coordinate system (G54+). Stores offset from absolute machine
                                 // position in mm. Loaded from EEPROM when called.
    float coord_offset[N_AXIS];    // Retains the G92 coordinate offset (work coordinates) relative to
                                 // machine zero in mm. Non-persistent. Cleared upon reset and boot.
    float tool_length_offset;      // Tracks tool length offset value when enabled.
    int32_t line_number;          // Last line number sent
    uint8_t tool;                 // Tracks tool number. NOT USED.
} parser_state_t;

extern parser_state_t gc_state;


typedef struct {
    non_modal_t non_modal_command;
    uint8_t user_defined_mcode;
    bool user_defined_mcode_sync;
    gc_modal_t modal;
    gc_values_t values;
} parser_block_t;


// Initialize the parser
void gc_init();

// Execute one block of rs275/ngc/g-code
status_code_t gc_execute_line(char *line);

// Set g-code parser position. Input in steps.
void gc_sync_position();

#endif
