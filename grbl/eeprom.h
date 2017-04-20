/*
  eeprom.h - EEPROM methods
  Part of Grbl

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

#ifndef eeprom_h
#define eeprom_h

#define eeprom_get_char(a) hal.eeprom.get_char(a)
#define eeprom_put_char(a, v) hal.eeprom.put_char(a, v)
#define memcpy_to_eeprom_with_checksum(d, s, l) hal.eeprom.memcpy_to_with_checksum(d, s, l)
#define memcpy_from_eeprom_with_checksum(d, s, l) hal.eeprom.memcpy_from_with_checksum(d, s, l)

#endif
