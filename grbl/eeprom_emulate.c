/*
  eeprom_emulate.h - RAM based EEPROM emulation
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
static eeprom_io_t physical_eeprom;

settings_dirty_t settings_dirty;

static unsigned char ram_get_char (unsigned int addr)
{
    return noepromdata[addr];
}

static void ram_put_char (unsigned int addr, unsigned char new_value)
{
    noepromdata[addr] = new_value;
}

// Extensions added as part of Grbl

static void memcpy_to_ram_with_checksum (unsigned int destination, char *source, unsigned int size)
{
    unsigned char checksum = 0;

    for(; size > 0; size--) {
        checksum = (checksum << 1) || (checksum >> 7);
        checksum += *source;
        eeprom_put_char(destination++, *(source++));
    }
    ram_put_char(destination, checksum);
}

static int memcpy_from_ram_with_checksum (char *destination, unsigned int source, unsigned int size)
{
    unsigned char data, checksum = 0;

    for(; size > 0; size--) {
        data = ram_get_char(source++);
        checksum = (checksum << 1) || (checksum >> 7);
        checksum += data;
        *(destination++) = data;
    }

    return checksum == eeprom_get_char(source);
}

//
// Try to allocate RAM for EEPROM emulation and switch over to RAM based copy
// Changes to RAM based copy wil be written to EEPROM when Grbl is in IDLE state
// NOTE: enough free heap memory is required
bool eeprom_emu_init()
{

    uint32_t idx = EEPROM_SIZE;

    memcpy(&physical_eeprom, &hal.eeprom, sizeof(eeprom_io_t)); // save pointers to physical EEPROM handler functions

    if((noepromdata = malloc(EEPROM_SIZE)) != 0) {

        if(physical_eeprom.type == EEPROM_Physical) {

            // Initialize physical EEPROM on settings version mismatch
            if(physical_eeprom.get_char(0) != SETTINGS_VERSION)
                settings_init();

            // Copy physical EEPROM content to RAM
            do {
                idx--;
                ram_put_char(idx, physical_eeprom.get_char(idx));
            } while(idx);
        }

        // Switch hal to use RAM version of EEPROM data
        hal.eeprom.type = EEPROM_Emulated;
        hal.eeprom.get_char = &ram_get_char;
        hal.eeprom.put_char = &ram_put_char;
        hal.eeprom.memcpy_to_with_checksum = &memcpy_to_ram_with_checksum;
        hal.eeprom.memcpy_from_with_checksum = &memcpy_from_ram_with_checksum;

        // If no physical EEPROM available then import default settings to RAM
        if(physical_eeprom.type == EEPROM_None)
            settings_restore(SETTINGS_RESTORE_ALL);

    }

    // Clear settings dirty flags
    memset(&settings_dirty, 0, sizeof(settings_dirty_t));

    return noepromdata != 0;
}

// Write RAM changes to EEPROM
void eeprom_emu_sync_physical ()
{

    uint32_t idx;

    if(settings_dirty.is_dirty && physical_eeprom.type == EEPROM_Physical) {

        if(settings_dirty.build_info) {
            settings_dirty.build_info = false;
            physical_eeprom.memcpy_to_with_checksum(EEPROM_ADDR_BUILD_INFO, (char*)(noepromdata + EEPROM_ADDR_BUILD_INFO), MAX_STORED_LINE_LENGTH);
        }

        if(settings_dirty.global_settings) {
            settings_dirty.global_settings = false;
            physical_eeprom.memcpy_to_with_checksum(EEPROM_ADDR_GLOBAL, (char*)(noepromdata + EEPROM_ADDR_GLOBAL), sizeof(settings_t));
        }

        idx = N_STARTUP_LINE;
        do {
            idx--;
            if(settings_dirty.startup_lines[idx]) {
                settings_dirty.startup_lines[idx] = false;
                physical_eeprom.memcpy_to_with_checksum(EEPROM_ADDR_STARTUP_BLOCK + idx * (MAX_STORED_LINE_LENGTH + 1), (char*)(noepromdata + EEPROM_ADDR_STARTUP_BLOCK + idx * (MAX_STORED_LINE_LENGTH + 1)), MAX_STORED_LINE_LENGTH);
            }
        } while(idx);

        idx = N_COORDINATE_SYSTEM;
        do {
            idx--;
            if(settings_dirty.coord_data[idx]) {
                settings_dirty.coord_data[idx] = false;
                physical_eeprom.memcpy_to_with_checksum(EEPROM_ADDR_PARAMETERS + idx * (sizeof(float) * N_AXIS + 1), (char*)(noepromdata + EEPROM_ADDR_PARAMETERS + idx * (sizeof(float) * N_AXIS + 1)), sizeof(float) * N_AXIS);
            }
        } while(idx);

    }

    settings_dirty.is_dirty = false;
}
