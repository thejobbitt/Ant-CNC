/*
  coolant_control.c - coolant control methods
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

#include "grbl.h"

#ifdef NUCLEO

void coolant_init(void)
{
  rcc_periph_clock_enable(COOLANT_RCC);
  SET_COOLANT_FLOOD_DDR;
#ifdef ENABLE_M7
  SET_COOLANT_MIST_DDR;
#endif
  coolant_stop();
}
void coolant_stop(void)
{
  UNSET_COOLANT_FLOOD_BIT;
#ifdef ENABLE_M7
  UNSET_COOLANT_MIST_BIT;
#endif
}
void coolant_set_state(uint8_t mode)
{
  if (mode == COOLANT_FLOOD_ENABLE)
  {
    SET_COOLANT_FLOOD_BIT;

#ifdef ENABLE_M7
  }
  else if (mode == COOLANT_MIST_ENABLE)
  {
    SET_COOLANT_MIST_BIT;
#endif
  }
  else
  {
    coolant_stop();
  }
}

#else
void coolant_init()
{
  COOLANT_FLOOD_DDR |= (1 << COOLANT_FLOOD_BIT);
  #ifdef ENABLE_M7
    COOLANT_MIST_DDR |= (1 << COOLANT_MIST_BIT);
  #endif
  coolant_stop();
}


void coolant_stop()
{
  COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
  #ifdef ENABLE_M7
    COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
  #endif
}


void coolant_set_state(uint8_t mode)
{
  if (mode == COOLANT_FLOOD_ENABLE) {
    COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);

  #ifdef ENABLE_M7  
    } else if (mode == COOLANT_MIST_ENABLE) {
      COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
  #endif

  } else {
    coolant_stop();
  }
}

#endif //NUCLEO_F401

void coolant_run(uint8_t mode)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Ensure coolant turns on when specified in program.  
  coolant_set_state(mode);
}
