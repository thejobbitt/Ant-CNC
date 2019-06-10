/*
  reset_vector.c - Redefine weak function reset_handler, to copy code in RAM at startup.
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

#include "grbl.h"

#ifdef NUCLEO_F401

extern int main(void);

/* Symbols exported by the linker script(s): */
extern unsigned _data_loadaddr, _data, _edata, _bss, _ebss, _stack, _ramcode, _eramcode, _ramcode_loadaddr;

/* Redefine the reset handler to allow ram code copy */
void reset_handler(void)
{
	volatile unsigned *src, *dest;

	for (src = &_data_loadaddr, dest = &_data;
		dest < &_edata;
		src++, dest++) {
		*dest = *src;
	}

	for (dest = &_bss; dest < &_ebss; dest++) {
		*dest++ = 0;
	}

	for (src = &_ramcode_loadaddr, dest = &_ramcode;
			dest < &_eramcode;
			src++, dest++) {
			*dest = *src;
		}


	/* Disable interrupts to relocate VTOR */
   __disable_irq(); // Global disable interrupts
   /* Relocate vector table */
   SCB->VTOR = (uint32_t)&_ramcode;
   __enable_irq(); // Global enable interrupts

	/* Ensure 8-byte alignment of stack pointer on interrupts */
	/* Enabled by default on most Cortex-M parts, but not M3 r1 */
	SCB_CCR |= SCB_CCR_STKALIGN;

	/* Enable access to Floating-Point coprocessor. */
	SCB_CPACR |= SCB_CPACR_FULL * (SCB_CPACR_CP10 | SCB_CPACR_CP11);

	/* Call the application's entry point. */
	main();
}
#endif /* NUCLEOF401 */

