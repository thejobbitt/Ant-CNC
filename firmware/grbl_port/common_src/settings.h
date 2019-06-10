/*
  settings.h - memory(flash or eeprom) configuration handling
  Part of grbl_port_opencm3 project, derived from the Grbl work.

  Copyright (c) 2017 Angelo Di Chello
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  
  Grbl_port_opencm3 is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl_port_opencm3 is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl_port_opencm3.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SETTINGS_H
#define SETTINGS_H

#ifdef NUCLEO
#include "flash.h"
#elif defined(CPU_MAP_ATMEGA328P) || defined(CPU_MAP_ATMEGA2560)
#include "eeprom.h"
#endif

// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom or flash dedicated sector.
#define SETTINGS_VERSION 9  // NOTE: Check settings_reset() when moving to next version.

// Define bit flag masks for the boolean settings in settings.flag.
#define BITFLAG_REPORT_INCHES      bit(0)
// #define BITFLAG_AUTO_START         bit(1) // Obsolete. Don't alter to keep back compatibility.
#define BITFLAG_INVERT_ST_ENABLE   bit(2)
#define BITFLAG_HARD_LIMIT_ENABLE  bit(3)
#define BITFLAG_HOMING_ENABLE      bit(4)
#define BITFLAG_SOFT_LIMIT_ENABLE  bit(5)
#define BITFLAG_INVERT_LIMIT_PINS  bit(6)
#define BITFLAG_INVERT_PROBE_PIN   bit(7)

// Define status reporting boolean enable bit flags in settings.status_report_mask
#define BITFLAG_RT_STATUS_MACHINE_POSITION  bit(0)
#define BITFLAG_RT_STATUS_WORK_POSITION     bit(1)
#define BITFLAG_RT_STATUS_PLANNER_BUFFER    bit(2)
#define BITFLAG_RT_STATUS_SERIAL_RX         bit(3)
#define BITFLAG_RT_STATUS_LIMIT_PINS        bit(4)

// Define settings restore bitflags.
#define SETTINGS_RESTORE_ALL 0xFF // All bitflags
#define SETTINGS_RESTORE_DEFAULTS bit(0)
#define SETTINGS_RESTORE_PARAMETERS bit(1)
#define SETTINGS_RESTORE_STARTUP_LINES bit(2)
#define SETTINGS_RESTORE_BUILD_INFO bit(3)

#ifdef NUCLEO
typedef enum main_sectore_restore_states_e
{
	MAIN_SECTOR_COPIED   = 0x000000FFU,
    MAIN_SECTOR_ERASED   = 0xFFFFFFFFU,
	MAIN_SECTOR_RESTORED = 0x00FFFFFFU,
	COPY_SECTOR_CLEARED  = 0x0000FFFFU
}eflash_restore_states;

#define EFLASH_ADDR_VERSION_OFFSET        0U
#define EFLASH_ADDR_GLOBAL_OFFSET         4U
#define EFLASH_ADDR_PARAMETERS_OFFSET     512U
#define EFLASH_ADDR_STARTUP_BLOCK_OFFSET  768U
#define EFLASH_ADDR_BUILD_INFO_OFFSET     944U
#define EFLASH_ERASE_AND_RESTORE_OFFSET   1020U

#define EFLASH_ADDR_VERSION_MAIN          (EFLASH_MAIN_BASE_ADDRESS + EFLASH_ADDR_VERSION_OFFSET)
#define EFLASH_ADDR_GLOBAL_MAIN           (EFLASH_MAIN_BASE_ADDRESS + EFLASH_ADDR_GLOBAL_OFFSET)
#define EFLASH_ADDR_PARAMETERS_MAIN       (EFLASH_MAIN_BASE_ADDRESS + EFLASH_ADDR_PARAMETERS_OFFSET)
#define EFLASH_ADDR_STARTUP_BLOCK_MAIN    (EFLASH_MAIN_BASE_ADDRESS + EFLASH_ADDR_STARTUP_BLOCK_OFFSET)
#define EFLASH_ADDR_BUILD_INFO_MAIN       (EFLASH_MAIN_BASE_ADDRESS + EFLASH_ADDR_BUILD_INFO_OFFSET)
#define EFLASH_MAIN_SECTOR_STATUS         (EFLASH_MAIN_BASE_ADDRESS + EFLASH_ERASE_AND_RESTORE_OFFSET)
#define EFLASH_ADDR_GLOBAL_COPY           (EFLASH_COPY_BASE_ADDRESS + EFLASH_ADDR_GLOBAL_OFFSET)
#define EFLASH_ADDR_PARAMETERS_COPY       (EFLASH_COPY_BASE_ADDRESS + EFLASH_ADDR_PARAMETERS_OFFSET)
#define EFLASH_ADDR_STARTUP_BLOCK_COPY    (EFLASH_COPY_BASE_ADDRESS + EFLASH_ADDR_STARTUP_BLOCK_OFFSET)
#define EFLASH_ADDR_BUILD_INFO_COPY       (EFLASH_COPY_BASE_ADDRESS + EFLASH_ADDR_BUILD_INFO_OFFSET)
#else
// Define EEPROM memory address location values for Grbl settings and parameters
// NOTE: The Atmega328p has 1KB EEPROM. The upper half is reserved for parameters and
// the startup script. The lower half contains the global settings and space for future 
// developments.
#define EEPROM_ADDR_GLOBAL         1U
#define EEPROM_ADDR_PARAMETERS     512U
#define EEPROM_ADDR_STARTUP_BLOCK  768U
#define EEPROM_ADDR_BUILD_INFO     942U
#endif

// Define EEPROM address indexing for coordinate parameters
#define N_COORDINATE_SYSTEM 6  // Number of supported work coordinate systems (from index 1)
#define SETTING_INDEX_NCOORD N_COORDINATE_SYSTEM+1 // Total number of system stored (from index 0)
// NOTE: Work coordinate indices are (0=G54, 1=G55, ... , 6=G59)
#define SETTING_INDEX_G28    N_COORDINATE_SYSTEM    // Home position 1
#define SETTING_INDEX_G30    N_COORDINATE_SYSTEM+1  // Home position 2
// #define SETTING_INDEX_G92    N_COORDINATE_SYSTEM+2  // Coordinate offset (G92.2,G92.3 not supported)

// Define Grbl axis settings numbering scheme. Starts at START_VAL, every INCREMENT, over N_SETTINGS.
#define AXIS_N_SETTINGS          4
#define AXIS_SETTINGS_START_VAL  100 // NOTE: Reserving settings values >= 100 for axis settings. Up to 255.
#define AXIS_SETTINGS_INCREMENT  10  // Must be greater than the number of axis settings

// Global persistent settings (Stored from byte EEPROM_ADDR_GLOBAL onwards)
typedef struct {
  // Axis settings
  float steps_per_mm[N_AXIS];
  float max_rate[N_AXIS];
  float acceleration[N_AXIS];
  float max_travel[N_AXIS];

  // Remaining Grbl settings
  uint8_t pulse_microseconds;
  uint8_t step_invert_mask;
  uint8_t dir_invert_mask;
  uint8_t stepper_idle_lock_time; // If max value 255, steppers do not disable.
  uint8_t status_report_mask; // Mask to indicate desired report data.
  float junction_deviation;
  float arc_tolerance;
  
  uint8_t flags;  // Contains default boolean settings

  uint8_t homing_dir_mask;
  float homing_feed_rate;
  float homing_seek_rate;
  uint16_t homing_debounce_delay;
  float homing_pulloff;

  uint32_t spindle_pwm_period;
  uint32_t spindle_pwm_max_time_on;
  uint32_t spindle_pwm_min_time_on;
  uint32_t spindle_pwm_enable_at_start;
} settings_t;
extern settings_t settings;

// Initialize the configuration subsystem (load settings from EEPROM)
void settings_init(void);

// Helper function to clear and restore EEPROM defaults
void settings_restore(uint8_t restore_flag);

// A helper method to set new settings from command line
uint8_t settings_store_global_setting(uint8_t parameter, float value);

// Stores the protocol line variable as a startup line in EEPROM
void settings_store_startup_line(uint8_t n, char *line);

// Reads an EEPROM startup line to the protocol line variable
uint8_t settings_read_startup_line(uint8_t n, char *line);

// Stores build info user-defined string
void settings_store_build_info(char *line);

// Reads build info user-defined string
uint8_t settings_read_build_info(char *line);

// Writes selected coordinate data to EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data);

// Reads selected coordinate data from EEPROM
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data);

// Returns the step pin mask according to Grbl's internal axis numbering
uint16_t get_step_pin_mask(uint8_t i);

// Returns the direction pin mask according to Grbl's internal axis numbering
uint16_t get_direction_pin_mask(uint8_t i);

// Returns the limit pin mask according to Grbl's internal axis numbering
uint16_t get_limit_pin_mask(uint8_t i);


#endif /* SETTINGS_H */
