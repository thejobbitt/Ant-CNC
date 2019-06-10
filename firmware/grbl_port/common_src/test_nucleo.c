/*
  test_nucleo.c - test functions for grbl_port_opencm3 on nucleo
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

#include "test_nucleo.h"

void test_initialization()
{
	/* Enable GPIOA clock. */
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Set GPIO5 (in GPIO port A) to 'output push-pull'. */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
}




void test_interrupt_signalling(uint32_t num_signals)
{
    uint32_t j;
    uint32_t i;
    for(j = 0; j < num_signals; j++)
    {
    	gpio_set(GPIOA, GPIO5);	/* LED on/off */
    	for (i = 0; i < 100000; i++)
    	{	/* Wait a bit. */
    		__asm__("nop");
    	}
    	gpio_clear(GPIOA, GPIO5);	/* LED on/off */
    	for (i = 0; i < 100000; i++)
    	{	/* Wait a bit. */
    		__asm__("nop");
    	}
    }
}

void test_led_toggle(void)
{

	if (gpio_get(GPIOA, GPIO5) != 0)
		gpio_clear(GPIOA, GPIO5);	/* LED off */
	else
		gpio_set(GPIOA, GPIO5);	/* LED on */
}
