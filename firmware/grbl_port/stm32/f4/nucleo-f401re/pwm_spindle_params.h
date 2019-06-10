/*
  pwm_spindle_params.h - pwm_spindle_params settings configuration file
  Part of grbl_port_opencm3 project.

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

/* The pwm_spindle_params.h file serves as a central pwm_spindle_params selector for different 
   motor drives.
   Ensure one and only one of these PWM_SPINDLE_PARAMS_XXX values is defined in config.h */

#ifndef PWM_SPINDLE_PARAMS_H
#define PWM_SPINDLE_PARAMS_H

#ifdef PWM_SPINDLE_PARAMS_GENERIC
  // Grbl generic pwm spindle parameters.
  #include "pwm_spindle_parameters/pwm_spindle_params_generic.h"

#elif defined(PWM_SPINDLE_PARAMS_AFRO20)
  // Grbl AFRO 20 ESC pwm spindle parameters.
  #include "pwm_spindle_parameters/pwm_spindle_params_afro20.h"

#else
  #error "Please define an existing PWM_SPINDLE parameters file"
#endif

#endif /* PWM_SPINDLE_PARAMS_H */
