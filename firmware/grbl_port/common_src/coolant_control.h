/*
  coolant_control.h - spindle control methods
  Part of grbl_port_opencm3 project, derived from the Grbl work.

  Copyright (c) 2017 Angelo Di Chello
  Copyright (c) 2012-2015 Sungeun K. Jeon

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

#ifndef COOLANT_CONTROL_H
#define COOLANT_CONTROL_H


void coolant_init(void);
void coolant_stop(void);
void coolant_set_state(uint8_t mode);
void coolant_run(uint8_t mode);

#endif /* COOLANT_CONTROL_H */
