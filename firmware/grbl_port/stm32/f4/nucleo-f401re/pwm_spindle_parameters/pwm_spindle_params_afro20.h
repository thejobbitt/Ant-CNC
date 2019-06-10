/*
  pwm_spindle_params_afro20.h - pwm spindle parameters configuration file for AFRO 20 ESC.
  Part of grbl_port_opencm3 project, derived from the Grbl work.

  Copyright (c) 2018 Angelo Di Chello

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

#ifndef PWM_SPINDLE_PARAMS_GENERIC_H
#define PWM_SPINDLE_PARAMS_GENERIC_H

  #define DEFAULT_SPINDLE_PWM_PERIOD 20000;
  #define DEFAULT_SPINDLE_PWM_MAX_TIME_ON 1860;
  #define DEFAULT_SPINDLE_PWM_MIN_TIME_ON 1060;
  #define DEFAULT_SPINDLE_PWM_ENABLE_AT_START 1;

#endif /* PWM_SPINDLE_PARAMS_GENERIC_H */
