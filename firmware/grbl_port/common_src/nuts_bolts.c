/*
  nuts_bolts.c - Shared functions
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

#include "grbl.h"


#define MAX_INT_DIGITS 8 // Maximum number of digits in int32 (and float)


// Extracts a floating point value from a string. The following code is based loosely on
// the avr-libc strtod() function by Michael Stumpf and Dmitry Xmelkov and many freely
// available conversion method examples, but has been highly optimized for Grbl. For known
// CNC applications, the typical decimal value is expected to be in the range of E0 to E-4.
// Scientific notation is officially not supported by g-code, and the 'E' character may
// be a g-code word on some CNC systems. So, 'E' notation will not be recognized. 
// NOTE: Thanks to Radu-Eosif Mihailescu for identifying the issues with using strtod().
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr)                  
{
  char *ptr = line + *char_counter;
  unsigned char c;
    
  // Grab first character and increment pointer. No spaces assumed in line.
  c = *ptr++;
  
  // Capture initial positive/minus character
  bool isnegative = false;
  if (c == '-') {
    isnegative = true;
    c = *ptr++;
  } else if (c == '+') {
    c = *ptr++;
  }
  
  // Extract number into fast integer. Track decimal in terms of exponent value.
  uint32_t intval = 0;
  int8_t exp = 0;
  uint8_t ndigit = 0;
  bool isdecimal = false;
  while(1) {
    c -= '0';
    if (c <= 9) {
      ndigit++;
      if (ndigit <= MAX_INT_DIGITS) {
        if (isdecimal) { exp--; }
        intval = (((intval << 2) + intval) << 1) + c; // intval*10 + c
      } else {
        if (!(isdecimal)) { exp++; }  // Drop overflow digits
      }
    } else if (c == (('.'-'0') & 0xff)  &&  !(isdecimal)) {
      isdecimal = true;
    } else {
      break;
    }
    c = *ptr++;
  }
  
  // Return if no digits have been read.
  if (!ndigit) { return(false); };
  
  // Convert integer into floating point.
  float fval;
  fval = (float)intval;
  
  // Apply decimal. Should perform no more than two floating point multiplications for the
  // expected range of E0 to E-4.
  if (fval != 0) {
    while (exp <= -2) {
      fval *= 0.01; 
      exp += 2;
    }
    if (exp < 0) { 
      fval *= 0.1; 
    } else if (exp > 0) {
      do {
        fval *= 10.0;
      } while (--exp > 0);
    } 
  }

  // Assign floating point value with correct sign.    
  if (isnegative) {
    *float_ptr = -fval;
  } else {
    *float_ptr = fval;
  }

  *char_counter = ptr - line - 1; // Set char_counter to next statement
  
  return(true);
}


// Simple hypotenuse computation function.
float hypot_f(float x, float y) { return(sqrt(x*x + y*y)); }

#ifdef NUCLEO

static inline uint32_t SysTick_Config_polling(uint32_t n_ticks)
{
	/* constant from systick_set_reload -- as this returns something that's
	 * not void, this is the only possible error condition */
	if (n_ticks & ~0x00FFFFFF) {
		return 1;
	}

	systick_set_reload(n_ticks);
	systick_set_clocksource(true);
//	systick_interrupt_enable();
	systick_counter_enable();

	return 0;
}

void SysTick_Init(void)
{
	/****************************************
	 *SystemFrequency/1000      1ms         *
	 *SystemFrequency/100000    10us        *
	 *SystemFrequency/1000000   1us         *
	 *****************************************/
	//while (SysTick_Config((F_CPU / 8) / 1000) != 0)
	while (SysTick_Config_polling((F_CPU / 8) / 1000) != 0)
	{} // One SysTick interrupt now equals 1us
}

volatile uint32_t ticks;
void sys_tick_handler()//SysTick_Handler (void)
{
	ticks++;
}

#define MILLIS (ticks)

void _delay_ms (double __ms)
{
	  uint32_t start, end;
	  uint32_t uncounted = 1;
	  uint32_t threshold = 10;
	  uint32_t value = threshold;

	  if(__ms == 0)
		  return;

	  SysTick_Init();

	  start = MILLIS;
	  end = start + __ms;
	  if (start < end)
	  {
		while ((MILLIS >= start) && (MILLIS < end))
		{
			value = systick_get_value();
			if((uncounted == 1) && (value < threshold))
			{
				ticks++;
				uncounted = 0;
			}
			else if(value >= threshold)
				uncounted = 1;
		}
	  }
	  else
	  {
		  while ((MILLIS >= start) || (MILLIS < end))
		  {
				value = systick_get_value();
				if((uncounted == 1) && (value < threshold))
				{
					ticks++;
					uncounted = 0;
				}
				else if(value >= threshold)
					uncounted = 1;
		  }
	  }
}

void delay_ms (uint16_t ms)
{
  uint32_t start, end;
  uint32_t uncounted = 1;
  uint32_t threshold = 10;
  uint32_t value = threshold;

  if(ms == 0)
	  return;

  SysTick_Init();

  start = MILLIS;
  end = start + ms;
  if (start < end)
  {
	while ((MILLIS >= start) && (MILLIS < end))
	{
		value = systick_get_value();
		if((uncounted == 1) && (value < threshold))
		{
			ticks++;
			uncounted = 0;
		}
		else if(value >= threshold)
			uncounted = 1;
	}
  }
  else
  {
	  while ((MILLIS >= start) || (MILLIS < end))
	  {
			value = systick_get_value();
			if((uncounted == 1) && (value < threshold))
			{
				ticks++;
				uncounted = 0;
			}
			else if(value >= threshold)
				uncounted = 1;
	  }
  }
}



void delay_1_ms()
{
	  uint32_t start, end;
	  start = MILLIS;
	  end = start + 1;
	  if (start < end) {
	    while ((MILLIS >= start) && (MILLIS < end)) {continue;}
	  } else {
	    while ((MILLIS >= start) || (MILLIS < end)) {continue;}
	  }
}


#else
// Delays variable defined microseconds. Compiler compatibility fix for _delay_us(),
// which only accepts constants in future compiler releases. Written to perform more 
// efficiently with larger delays, as the counter adds parasitic time in each iteration.
void delay_us(uint32_t us) 
{
  while (us) {
    if (us < 10) { 
      _delay_us(1);
      us--;
    } else if (us < 100) {
      _delay_us(10);
      us -= 10;
    } else if (us < 1000) {
      _delay_us(100);
      us -= 100;
    } else {
      _delay_ms(1);
      us -= 1000;
    }
  }
}
#endif


