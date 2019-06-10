/*
  defaults_cpcb_large.h - defaults settings configuration file for large version of Compact PCB Maker machine.
  Part of grbl_port_opencm3 project.

  Copyright (c) 2017 Angelo Di Chello

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

/* The defaults.h file serves as a central default settings file for different machine
   types, from DIY CNC mills to CNC conversions of off-the-shelf machines. The settings
   here are supplied by users, so your results may vary. However, this should give you
   a good starting point as you get to know your machine and tweak the settings for your
   nefarious needs. */


//#ifndef GRBL_PORT_STM32_F4_NUCLEO_F401RE_GRBL_DEFAULTS_DEFAULTS_CPCB_LARGE_H_
//#define GRBL_PORT_STM32_F4_NUCLEO_F401RE_GRBL_DEFAULTS_DEFAULTS_CPCB_LARGE_H

#ifndef defaults_h
#define defaults_h

  // Grbl generic default settings. Should work across different machines.
  #define DEFAULT_X_STEPS_PER_MM 20.0
  #define DEFAULT_Y_STEPS_PER_MM 20.0
  #define DEFAULT_Z_STEPS_PER_MM 50.0
  #define DEFAULT_X_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_X_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 0
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK ((BITFLAG_RT_STATUS_MACHINE_POSITION)|(BITFLAG_RT_STATUS_WORK_POSITION))
  #define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 1 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 1  // false
  #define DEFAULT_HOMING_ENABLE 1  // true
  #define DEFAULT_HOMING_DIR_MASK 3 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 500.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm

#endif /* GRBL_PORT_STM32_F4_NUCLEO_F401RE_GRBL_DEFAULTS_DEFAULTS_CPCB_H */
