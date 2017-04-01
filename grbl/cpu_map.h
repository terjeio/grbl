/*
  cpu_map.h - CPU and pin mapping configuration file
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

/* The cpu_map.h files serve as a central pin mapping selection file for different
   processor types or alternative pin layouts. This version of Grbl officially supports
   only the Arduino Mega328p. */


#ifndef cpu_map_h
#define cpu_map_h


#ifdef CPU_MAP_GENERIC // generic map, makes no assumptions about the underlying hardware - the hardware driver maps to/from physical pins

#define X_DIRECTION_BIT X_AXIS
#define Y_DIRECTION_BIT Y_AXIS
#define Z_DIRECTION_BIT Z_AXIS
#define X_STEP_BIT X_AXIS
#define Y_STEP_BIT Y_AXIS
#define Z_STEP_BIT Z_AXIS
#define STEP_MASK ((1 << X_AXIS)|(1 << Y_AXIS)|(1 << Z_AXIS))
#define DIRECTION_MASK ((1 << X_AXIS)|(1 << Y_AXIS)|(1 << Z_AXIS))
#define LIMIT_MASK ((1 << X_AXIS)|(1 << Y_AXIS)|(1 << Z_AXIS))

#define ENABLE_M7

#endif

/*
#ifdef CPU_MAP_CUSTOM_PROC
  // For a custom pin map or different processor, copy and edit one of the available cpu
  // map files and modify it to your needs. Make sure the defined name is also changed in
  // the config.h file.
#endif
*/

#endif
