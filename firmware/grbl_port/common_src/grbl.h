/*
  grbl.h - main Grbl include file
  Part of grbl_port_opencm3 project, derived from the Grbl work.

  Copyright (c) 2017 Angelo Di Chello
  Copyright (c) 2015 Sungeun K. Jeon

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

#ifndef GRBL_H
#define GRBL_H

// Grbl versioning system
#define GRBL_VERSION "0.9j"
#define GRBL_VERSION_BUILD __DATE__" "__TIME__

#include "config.h"
#include "cpu_map.h"
// Define standard libraries used by Grbl.
#ifdef NUCLEO
#include <libopencm3/cm3/nvic.h>
#include <libopencmsis/core_cm3.h>
#elif defined(CPU_MAP_ATMEGA328P) || defined(CPU_MAP_ATMEGA2560)
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#endif
#include <math.h>
#include <inttypes.h>    
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// Define the Grbl system include files. NOTE: Do not alter organization.
#include "nuts_bolts.h"
#include "settings.h"
#include "system.h"
#include "defaults.h"
#include "pwm_spindle_params.h"
#include "coolant_control.h"

#include "gcode.h"
#include "limits.h"
#include "motion_control.h"
#include "planner.h"
#include "print.h"
#include "probe.h"
#include "protocol.h"
#include "report.h"
#include "serial.h"
#include "spindle_control.h"
#include "stepper.h"

#endif /* GRBL_H */
