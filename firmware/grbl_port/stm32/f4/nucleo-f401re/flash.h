/*
  flash.c - flash access functions
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

#ifndef FLASH_H
#define FLASH_H

unsigned char flash_get_char( unsigned int addr );
void flash_put_char( unsigned int addr, unsigned char new_value);
unsigned int flash_verify_erase_need(char * destination, char *source, unsigned int size);
void memcpy_to_flash_with_checksum(unsigned int destination, char *source, unsigned int size);
int memcpy_from_flash_with_checksum(char *destination, unsigned int source, unsigned int size);
void update_main_sector_status(uint32_t updated_status);
void delete_main_sector(void);
void delete_copy_sector(void);
void copy_from_main_to_copy(uint32_t start_address_offset, uint32_t end_address_offset);
void restore_main_sector(void);
void restore_default_sector_status(void);

#endif /* FLASH_H */
