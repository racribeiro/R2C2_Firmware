/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
/* Copyright (c) 2014 Rui Ribeiro - racribeiro@gmail.com     */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdint.h>
#include <stdlib.h>

#include "lpc17xx_timer.h"
#include "lpc17xx_wdt.h"
#include "lpc17xx_adc.h"
#include "r2c2.h"

#include "machine.h"
#include "gcode_parse.h"
#include "gcode_process.h"
#include "pinout.h"
#include "debug.h"
#include "config.h"
#include "temp.h"

#include "planner.h"
#include "stepper.h"

#include "rtttl.h"

tTimer temperatureTimer;
tTimer extruderDriver;
tTimer commandLineDriver;
tTimer fanTimer;

tLineBuffer serial_line_buf;
tLineBuffer sd_line_buf;
tLineBuffer uart_line_buf;

long timer1 = 0;  
long timer2 = 0;

int is_plan_queue_full = 0;

/* Initialize ADC for reading sensors */
void adc_init(void)
{
  PINSEL_CFG_Type PinCfg;

  PinCfg.Funcnum = PINSEL_FUNC_2; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = EXTRUDER_0_SENSOR_ADC_PORT;
  PinCfg.Pinnum = EXTRUDER_0_SENSOR_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

  PinCfg.Funcnum = PINSEL_FUNC_2; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Portnum = HEATED_BED_0_ADC_PORT;
  PinCfg.Pinnum = HEATED_BED_0_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);

  ADC_Init(LPC_ADC, 200000); /* ADC conversion rate = 200Khz */
}

void io_init(void)
{
  /* Extruder 0 Heater pin */
  pin_mode(EXTRUDER_0_HEATER_PORT, EXTRUDER_0_HEATER_PIN, OUTPUT);
  extruder_heater_off();

  /* Heated Bed 0 Heater pin */
  pin_mode(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_PIN, OUTPUT);
  heated_bed_off();

  /* setup I/O pins */
  pin_mode(STEPPERS_RESET_PORT, STEPPERS_RESET_PIN, OUTPUT);
  digital_write(STEPPERS_RESET_PORT, STEPPERS_RESET_PIN, 1); /* Disable reset for all stepper motors */

  pin_mode(X_STEP_PORT, X_STEP_PIN, OUTPUT);
  pin_mode(X_DIR_PORT, X_DIR_PIN, OUTPUT);
  pin_mode(X_ENABLE_PORT, X_ENABLE_PIN, OUTPUT);
  x_enable();
  pin_mode(X_MIN_PORT, X_MIN_PIN, INPUT);

  pin_mode(Y_STEP_PORT, Y_STEP_PIN, OUTPUT);
  pin_mode(Y_DIR_PORT, Y_DIR_PIN, OUTPUT);
  pin_mode(Y_ENABLE_PORT, Y_ENABLE_PIN, OUTPUT);
  y_enable();
  pin_mode(Y_MIN_PORT, Y_MIN_PIN, INPUT);

  pin_mode(Z_STEP_PORT, Z_STEP_PIN, OUTPUT);
  pin_mode(Z_DIR_PORT, Z_DIR_PIN, OUTPUT);
  pin_mode(Z_ENABLE_PORT, Z_ENABLE_PIN, OUTPUT);
  z_enable();
  pin_mode(Z_MIN_PORT, Z_MIN_PIN, INPUT);

  pin_mode(E_STEP_PORT, E_STEP_PIN, OUTPUT);
  pin_mode(E_DIR_PORT, E_DIR_PIN, OUTPUT);
  pin_mode(E_ENABLE_PORT, E_ENABLE_PIN, OUTPUT);
  e_enable();

  pin_mode(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_PIN, OUTPUT);
  extruder_fan_off();

  adc_init();
}

void temperatureTimerCallback (tTimer *pTimer)
{
  // static long queue_next_ok = 0;
  long now;
  
  now = millis();
  
  /* Manage the temperatures */
  if (timer2 < now) {
    timer2 = now + config.temp_sample_rate;
    temp_tick(pTimer);
  }
  
  #define DELAY1 250  
  if (timer1 < now)
  {
    #ifdef REPETIER_MTEMP
    timer1 = now + DELAY1;	
    sersendf("MTEMP:%u %u\r\n", now, 
	  temp_get(EXTRUDER_0), temp_get_target(EXTRUDER_0) ,(uint8_t)(get_pid_val(EXTRUDER_0) * 2.55));
	  // temp_get(HEATED_BED_0), temp_get_target(HEATED_BED_0) ,(uint8_t)(get_pid_val(HEATED_BED_0) * 2.55));	  
	#endif

	#ifdef DEBUG  
	if (is_plan_queue_full) {
	  sersendf("- Planning Queue is Full\r\n");
	}
	#endif
	
	// Sends an "ok" every 25 seconds if nothing is happening. Prevents rare
	// case of buffer overrun by timers and host blockage

    /*	
	if (plan_queue_empty()) {
	  if (queue_next_ok == 0) {
	    queue_next_ok = now + 25000;
	  } else {
	    if (queue_next_ok < now) {
  	      sersendf("\r\nok\r\n");
		  queue_next_ok = 0;
        }
	  }
	} else {
	  queue_next_ok = 0;
	}
	*/
  }
  
}

void check_boot_request (void)
{
  if (digital_read (4, (1<<29)) == 0)
  {
    WDT_Init (WDT_CLKSRC_PCLK, WDT_MODE_RESET);
    WDT_Start (10);
    while (1);
  }
}

void bootButtonCallback()
{
  rtttl_play_axelf();
  serial_writestr("ok\r\n");
}

int is_boot_button_pressed()
{
  return digital_read (4, (1<<29)) == 0;
}

void check_boot_button (void func())
{
  if (is_boot_button_pressed())
  {
    func();
  }
}


void init(void)
{
  // set up inputs and outputs
  io_init();
  
  // set up inputs and outputs
  uart_init(57600);    
  uart_writestr("Hi UART Port!!\r\n");
  
  /* Initialize Gcode parse variables */
  gcode_parse_init();

  // set up default feedrate
  //TODO  current_position.F = startpoint.F = next_target.target.F =       config.search_feedrate_z;

  // say hi to host
  sersendf("Start\r\nok\r\n");
}

void processCommandQueue()
{    
    eParseResult parse_result;
	
    // process characters from the serial port
    while (!serial_line_buf.seen_lf && (serial_rxchars() != 0) )
    {
      unsigned char c = serial_popchar();
      
      if (serial_line_buf.len < MAX_LINE)
        serial_line_buf.data [serial_line_buf.len++] = c;

      if ((c==10) || (c==13))
      {
        if (serial_line_buf.len > 1)
          serial_line_buf.seen_lf = 1;
        else
          serial_line_buf.len = 0;
      }      
    }

    /* RR 2014-11-20 : the uart port config isn't correct. disabling as it may be interfering with fan or boot button
    while(uart_data_available()) {  
      unsigned char c = uart_receive();
      
      if (uart_line_buf.len < MAX_LINE)
        uart_line_buf.data [uart_line_buf.len++] = c;

      if ((c==10) || (c==13))
      {
        if (uart_line_buf.len > 1)
          uart_line_buf.seen_lf = 1;
        else
          uart_line_buf.len = 0;
      }
	  
	  if (uart_line_buf.len > 0) {
	    uart_sendf("got something [%u : %s]\r\n", uart_line_buf.len, uart_line_buf.data);
	    if (strcmp(uart_line_buf.data, "HELLO")) {
		  uart_writestr("HELLO!\r\n");
		}
	  }
	}
	*/
	
	// process SD file if no serial command pending
    if (!sd_line_buf.seen_lf && sd_printing)
    {
      if (sd_read_file (&sd_line_buf))
      {
          sd_line_buf.seen_lf = 1;
      } 
      else
      {
        sd_printing = false;
        debug("Done printing file\r\n");
      }
    }

    is_plan_queue_full = plan_queue_full();
	
    // if queue is full, we wait
    if (!is_plan_queue_full)
    {
  
      /* At end of each line, put the "GCode" on movebuffer.
       * If there are movement to do, Timer will start and execute code which
       * will take data from movebuffer and generate the required step pulses
       * for stepper motors.
       */
  
      parse_result = PR_UNKNOWN;

      // give priority to user commands
      if (serial_line_buf.seen_lf)
      {
        parse_result = gcode_parse_line (&serial_line_buf);
        serial_line_buf.len = 0;
        serial_line_buf.seen_lf = 0;
      }
      else if (sd_line_buf.seen_lf)
      {
        parse_result = gcode_parse_line (&sd_line_buf);
        sd_line_buf.len = 0;
        sd_line_buf.seen_lf = 0;
      }

      if (parse_result != PR_OK) {
	    switch (parse_result) {
		  case PR_UNKNOWN:
	        //serial_writestr("- Unknown\r\n");
		    break;
		  case PR_RESEND:
	        serial_writestr("- Resend\r\n");
		    break;
		  case PR_ERROR:
		    serial_writestr("- Error!\r\n");
			break;
          case PR_BUSY:
		    serial_writestr("- Busy!\r\n");
		    break;
		  default:
		    break;
		}
	  }	  	  
    }

}

void init_timers()
{
  // Checks and refresh temperature extruder/heater pattern
  // prints MTEMP
  AddSlowTimer (&temperatureTimer, "Temperature Monitor");
  StartSlowTimer (&temperatureTimer, 50, temperatureTimerCallback);
  temperatureTimer.AutoReload = 1;

  // One tick on extruder/heater pattern
  AddSlowTimer (&extruderDriver, "Temperature Driver");
  StartSlowTimer (&extruderDriver, 2, extruderDriverCallback);
  extruderDriver.AutoReload = 1;
  
  // One callback function to fan delay timer
  AddSlowTimer (&fanTimer, "Speeding up fan");
  fanTimer.AutoReload = 0;
    
  /*
  AddSlowTimer (&commandLineDriver, "Command Driver");
  StartSlowTimer (&commandLineDriver, 5, commandLineCallback);
  commandLineDriver.AutoReload = 1;
  */
  
}

int app_main (void)
{ 
  int count = 0;   
  init();

#ifdef DEBUG_ON_BOOT
  while (!is_boot_button_pressed()) {};

  while (count < 10) {
  count++;
  debug(" - count : %d\r\n", count); 
#endif  

  // init timers
  init_timers();
  buzzer_init();

  // Only readconfigs after timers and buzzer initiated
  read_config();

  buzzer_play(1500, 100); /* low beep */  
  buzzer_play(2500, 200); /* high beep */

  // grbl init
  plan_init();      
  st_init();    

  // temperature init
  temp_init(config.temp_sample_rate, config.temp_buffer_duration);    
	
  // main loop
#ifdef DEBUG_ON_BOOT    
  while (!is_boot_button_pressed())
#else
  for(;;)
#endif  
  {    
	processCommandQueue();
#ifdef USE_BOOT_BUTTON
    // OPTION: enter bootloader on "Boot" button
    check_boot_request();
#else
    check_boot_button(bootButtonCallback);
#endif

  }
  
#ifdef DEBUG_ON_BOOT  
  }
#endif  
  
  return count;
}
