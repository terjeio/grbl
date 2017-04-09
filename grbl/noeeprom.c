/*
  noeeprom.h - RAM based EEPROM emulation
  Part of Grbl

  Copyright (c) 2017 Terje Io
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License.
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Can be used by MCUs with no nonvolatile storage options, be sure to allocate enough heap memory before use
// Call noeeprom_init(), if returns true then set up EEPROM HAL pointers to these functions
//

#include "grbl.h"

static unsigned char *noepromdata = 0;

//
// Allocate 1Kb for EEPROM emulation
//
bool noeeprom_init()
{
    return (noepromdata = malloc(1024)) != 0;
}

unsigned char noeeprom_get_char (unsigned int addr)
{
    return noepromdata[addr];
}

void noeeprom_put_char (unsigned int addr, unsigned char new_value)
{
    noepromdata[addr] = new_value;
}

// Extensions added as part of Grbl

void memcpy_to_noeeprom_with_checksum (unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) {
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += *source;
    eeprom_put_char(destination++, *(source++));
  }
  noeeprom_put_char(destination, checksum);
}

int memcpy_from_noeeprom_with_checksum (char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) {
    data = noeeprom_get_char(source++);
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += data;
    *(destination++) = data;
  }
  return(checksum == eeprom_get_char(source));
}
