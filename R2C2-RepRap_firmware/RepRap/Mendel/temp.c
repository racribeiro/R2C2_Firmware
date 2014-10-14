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

#include "temp.h"

/* {ADC value Extruder0, ADC value HeatedBed0, temperature} */
int16_t temptable[NUMTEMPS][3] = {
  {860, 60, 300},
  {1849, 95, 248},
  {2208, 119, 226},
  {2711, 215, 198},
  {2960, 293, 183},
  {3332, 447, 163},
  {3568, 641, 145},
  {3711, 865, 131},
  {3870, 1408, 105},
  {3960, 1906, 86},
  {4032, 2732, 64},
  {4062, 3352, 42},
  {4070, 3755, 22},
  {4080, 4085, 0}
};

static struct PID pid[NUMBER_OF_SENSORS];
static int16_t  current_temp [NUMBER_OF_SENSORS] = {0};
static int16_t  target_temp  [NUMBER_OF_SENSORS] = {0};
static double  output_temp  [NUMBER_OF_SENSORS] = {0};
static uint32_t adc_filtered [NUMBER_OF_SENSORS] = {4095, 4095}; // variable must have the higher value of ADC for filter start at the lowest temperature

#ifndef	ABSDELTA
#define	ABSDELTA(a, b)	(((a) >= (b))?((a) - (b)):((b) - (a)))
#endif

static int16_t read_temp(uint8_t sensor_number);

static uint8_t last_extruder_state[NUMBER_OF_SENSORS];
static uint8_t temp_pattern_pos[NUMBER_OF_SENSORS];
static bool temp_pattern[NUMBER_OF_SENSORS][TEMP_ELEMENTS];

static char debug_str[NUMBER_OF_SENSORS][1024];

int get_temp_pattern_pos(uint8_t sensor_number)
{
  return temp_pattern_pos[sensor_number];
}

char* temp_debug(uint8_t sensor_number)
{
  int i;
  
  for(i = 0; i < TEMP_ELEMENTS; i++) {
    if (temp_pattern[sensor_number][i] == 0)
	  debug_str[sensor_number][i] = '0';
	else
	  debug_str[sensor_number][i] = '1';
  }

  if (temp_pattern[sensor_number][temp_pattern_pos[sensor_number]] == 0)
    debug_str[sensor_number][temp_pattern_pos[sensor_number]] = '-';
  else
    debug_str[sensor_number][temp_pattern_pos[sensor_number]] = '+';
  debug_str[sensor_number][TEMP_ELEMENTS] = '\0';
  
  return debug_str[sensor_number];
}

void extruderDriverCallback()
{
  uint8_t sensor_number = 0;
  for(sensor_number = 0; sensor_number < NUMBER_OF_SENSORS; sensor_number++) {
    temp_pattern_pos[sensor_number]++;
    if (temp_pattern_pos[sensor_number] == TEMP_ELEMENTS)
      temp_pattern_pos[sensor_number] = 0;
	
    if (temp_pattern[sensor_number][temp_pattern_pos[sensor_number]] != last_extruder_state[sensor_number]) {
      last_extruder_state[sensor_number] = !last_extruder_state[sensor_number];
	  if (last_extruder_state[sensor_number]) {
	    if (sensor_number == EXTRUDER_0)
          extruder_heater_on();
		if (sensor_number == HEATED_BED_0)
          heated_bed_on();
      } else {
        if (sensor_number == EXTRUDER_0)
          extruder_heater_off();
		if (sensor_number == HEATED_BED_0)
          heated_bed_off();
      }
	}
  }
}

void temp_reset(int sensor_id)
{
  set_heater_pattern(sensor_id, 0);
}

void temp_init_sensor(uint8_t sensor_id, unsigned int sampletime, unsigned int logduration)
{  
   sersendf("- init - %u\r\n", sensor_id); /* for RepRap software */

   last_extruder_state[sensor_id] = 0;
   last_extruder_state[sensor_id] = LOW;
   set_heater_pattern(sensor_id, 0);
   
   if (sensor_id == EXTRUDER_0) {   
     PID_PID(&pid[sensor_id], &current_temp[sensor_id], &output_temp[sensor_id], &target_temp[sensor_id],
                              config.p_factor_extruder_1, config.i_factor_extruder_1, config.d_factor_extruder_1, DIRECT, sampletime, logduration * 1000 / sampletime);  	 
     PID_SetOutputLimits(&pid[sensor_id], config.min_extruder_1, config.max_extruder_1);
   }
   
   if (sensor_id == HEATED_BED_0) {   
     PID_PID(&pid[sensor_id], &current_temp[sensor_id], &output_temp[sensor_id], &target_temp[sensor_id],
                              config.p_factor_heated_bed_0, config.i_factor_heated_bed_0, config.d_factor_heated_bed_0, DIRECT, sampletime, logduration * 1000 / sampletime);  
     PID_SetOutputLimits(&pid[sensor_id], config.min_heated_bed_0, config.max_heated_bed_0);							  
   }
      
   PID_SetMode(&pid[sensor_id], AUTOMATIC);
}

double temp_get_output_temp(uint8_t sensor_id)
{
  return output_temp[sensor_id];
}

void temp_init(unsigned int sampletime, unsigned int logduration)
{
  int i;
  for(i = 0; i < NUMBER_OF_SENSORS; i++) {
    temp_init_sensor(i, sampletime, logduration);
  }
}

void temp_set(uint8_t sensor_number,int16_t t)
{
  if (t)
  {
    steptimeout = 0;  // Initiate timer
//?    power_on(); // Turn on ATX, not needed on R2C2
  }

  target_temp[sensor_number] = t;  

}

int16_t temp_get(uint8_t sensor_number)
{
  return current_temp[sensor_number];
}

int16_t temp_get_target(uint8_t sensor_number)
{
  return target_temp[sensor_number];
}

int8_t	temp_achieved(uint8_t sensor_number)
{
  if (current_temp[sensor_number] >= (target_temp[sensor_number] - config.temp_margin))
    return 255;

  return 0;
}

int8_t temps_achieved (void)
{
  if ((current_temp[EXTRUDER_0] >= (target_temp[EXTRUDER_0] - config.temp_margin)) && (current_temp[HEATED_BED_0] >= (target_temp[HEATED_BED_0] - config.temp_margin)))
    return 255;

  return 0;
}

void temp_print()
{
  sersendf("ok T:%u.0 B:%u.0\r\n", current_temp[EXTRUDER_0], current_temp[HEATED_BED_0]); /* for RepRap software */
}

void set_heater_pattern(uint8_t sensor_number, float power)
{  
  double level, step, position;

  output_temp[sensor_number] = power;   
  level = power * TEMP_ELEMENTS / 100;  
  step = TEMP_ELEMENTS / level;
  
  memset (&temp_pattern[sensor_number], 0, sizeof(temp_pattern[sensor_number]));    
  
  if (power > 0) {
    position = 0;
    while(position < TEMP_ELEMENTS) {
  	  temp_pattern[sensor_number][(int)position] = 1;
	  position += step;
    }
  }
}

double get_pid_val(uint8_t sensor_number)
{
  return *pid[sensor_number].myOutput;
}

void temp_tick(__attribute__((unused)) tTimer *pTimer)
{
  bool result = 0;
  
  //sersendf("- temp_tick\r\n");
  
  /* Read and average temperatures */
  current_temp[EXTRUDER_0] = read_temp(EXTRUDER_0);  

  if (target_temp[EXTRUDER_0] == 0) {
    set_heater_pattern(EXTRUDER_0,0);
  } else {
    if (current_temp[EXTRUDER_0] > config.safeguard_extruder_1) { // security above limit
	  sersendf("- E Safeguard value exceded [%d], turning off extruder_1\r\n", config.safeguard_extruder_1);
      set_heater_pattern(EXTRUDER_0,0);
    } else {
      result = PID_Compute(&pid[EXTRUDER_0]);	  
	  if (result) {
	    // sersendf("- E + %g\r\n", *pid[EXTRUDER_0].myOutput);
	    set_heater_pattern(EXTRUDER_0,*pid[EXTRUDER_0].myOutput);
      }
    }
  }
  
  current_temp[HEATED_BED_0] = read_temp(HEATED_BED_0);
  
  if (target_temp[HEATED_BED_0] == 0) {
    set_heater_pattern(HEATED_BED_0,0);
  } else {
    if (current_temp[HEATED_BED_0] > config.safeguard_heated_bed_0) { // security above limit
	  sersendf("- H Safeguard value exceded [%d], turning off heated_bed_0\r\n", config.safeguard_heated_bed_0);
      set_heater_pattern(HEATED_BED_0,0);	  
    } else {
	
      result = PID_Compute(&pid[HEATED_BED_0]);	  
	  if (result) {
	    // sersendf("- H + %g\r\n", *pid[HEATED_BED_0].myOutput);
	    set_heater_pattern(HEATED_BED_0,*pid[HEATED_BED_0].myOutput);
      }
    }
  }
  
  /* Manage heater using simple ON/OFF logic, no PID */  
  
  //if (current_temp[EXTRUDER_0] < target_temp[EXTRUDER_0])
  /*
  if (temp_turn_on(EXTRUDER_0))
  {
    extruder_heater_on();
  }
  else
  {
    extruder_heater_off();
  }
  */

  
  /* Manage heater using simple ON/OFF logic, no PID */
  
  /*
  if (current_temp[HEATED_BED_0] < target_temp[HEATED_BED_0])
  //if (temp_turn_on(HEATED_BED_0))
  {
    heated_bed_on();
  }
  else
  {
    heated_bed_off();
  }
  */
}

/* Read and average the ADC input signal */
static int16_t read_temp(uint8_t sensor_number)
{
  int32_t raw = 4095; // initialize raw with value equal to lowest temperature.
  static int32_t raw_correct = 4095;
  int16_t celsius = 0;
  uint8_t i;

  if (sensor_number == EXTRUDER_0)
  {
    raw = analog_read(EXTRUDER_0_SENSOR_ADC_CHANNEL);
  }
  else if (sensor_number == HEATED_BED_0)
  {
    raw = analog_read(HEATED_BED_0_SENSOR_ADC_CHANNEL);

    /* There is a problem with LPC1768 ADC being overdrive with > 3.3V on ADC extruder channel and that makes
     * error readings on ADC bed channel (only when extruder is cold less than ~15ºC).
     * Try to avoid the bad readings that usually are raw =< 300.
     */
    if (raw < 300) // error, assume last correct value
    {
      raw = raw_correct;
    }
    else // no error, save current raw
    {
      raw_correct = raw;
    }
  }
  
  // filter the ADC values with simple IIR
  adc_filtered[sensor_number] = ((adc_filtered[sensor_number] * 15) + raw) / 16;
  
  raw = adc_filtered[sensor_number];

  /* Go and use the temperature table to math the temperature value... */
  if (raw < temptable[0][sensor_number]) /* Limit the smaller value... */
  {
    celsius = temptable[0][2];
  }
  else if (raw >= temptable[NUMTEMPS-1][sensor_number]) /* Limit the higher value... */
  {
    celsius = temptable[NUMTEMPS-1][2];
  }
  else
  {
    for (i=1; i<NUMTEMPS; i++)
    {
      if (raw < temptable[i][sensor_number])
      {
        celsius = temptable[i-1][2] +
            (raw - temptable[i-1][sensor_number]) *
            (temptable[i][2] - temptable[i-1][2]) /
            (temptable[i][sensor_number] - temptable[i-1][sensor_number]);

        break;
      }
    }
  }

  return celsius;
}

bool temp_set_table_entry (uint8_t sensor_number, int16_t temp, int16_t adc_val)
{
  if (sensor_number < NUMBER_OF_SENSORS)
  {
    for (int entry=0; entry < NUMTEMPS; entry++)
    {
      if (temptable[entry][2] == temp)
      {
        temptable[entry][sensor_number] = adc_val;
        return true;
      }
    }
    return false;
  }
  else
    return false;
}

int16_t temp_get_table_entry (uint8_t sensor_number, int16_t temp)
{
  int16_t result = 0xffff;
  
  if (sensor_number < NUMBER_OF_SENSORS)
  {
    for (int entry=0; entry < NUMTEMPS; entry++)
    {
      if (temptable[entry][2] == temp)
      {
        result = temptable[entry][sensor_number];
        break;
      }
    }
  }
  return result;
}
