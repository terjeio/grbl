/*
  eeprom.h - EEPROM methods
  Part of Grbl

  Copyright (c) 2017 Terje Io
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

#ifndef __noeeprom_h__
#define __noeeprom_h__

bool noeeprom_init();
unsigned char noeeprom_get_char(unsigned int addr);
void noeeprom_put_char(unsigned int addr, unsigned char new_value);
void memcpy_to_noeeprom_with_checksum(unsigned int destination, char *source, unsigned int size);
int memcpy_from_noeeprom_with_checksum(char *destination, unsigned int source, unsigned int size);

#endif
