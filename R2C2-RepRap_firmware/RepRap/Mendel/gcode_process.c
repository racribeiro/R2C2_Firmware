/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
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

#include <string.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include "gcode_process.h"
#include "gcode_parse.h"
#include "serial.h"
#include "sermsg.h"
#include "sersendf.h"
#include "temp.h"
#include "timer.h"
#include "pinout.h"
#include "config.h"
#include "ff.h"
#include "buzzer.h"
#include "rtttl.h"
//#include "debug.h"
#include "planner.h"
#include "stepper.h"
#include "geometry.h"

#define MOTION_MODE_SEEK 0 // G0
#define MOTION_MODE_LINEAR 1 // G1
#define MOTION_MODE_CW_ARC 2 // G2
#define MOTION_MODE_CCW_ARC 3 // G3
#define MOTION_MODE_CANCEL 4 // G80

FATFS     fs;       /* Work area (file system object) for logical drive */
FIL       file;
uint32_t  filesize = 0;
uint32_t  sd_pos = 0;
bool      sd_printing = false;      // printing from SD file
bool      sd_active = false;        // SD card active
bool      sd_writing_file = false;  // writing to SD file

#define EXTRUDER_NUM_1  1
#define EXTRUDER_NUM_2  2
#define EXTRUDER_NUM_3  4

uint8_t   extruders_on;
double    extruder_1_speed;         // in RPM

uint32_t  auto_prime_steps = 0;
uint32_t  auto_reverse_steps = 0;
const double auto_prime_feed_rate = 18000;
const double auto_reverse_feed_rate = 18000;
double auto_prime_factor = 640;
double auto_reverse_factor = 640;

#define MAX_TARGET_STACK 20
tTarget targetStack[MAX_TARGET_STACK];
uint8_t targetStackPos = 0;

#define BED_LEVELING_POINTS 3
tTarget bedLeveling[BED_LEVELING_POINTS];
uint8_t bedLevelingDefined[BED_LEVELING_POINTS];
uint8_t bedLevelingActive = false;
double bedLevelingMatrix[3][3];

#define __SAFE_TRAVEL__

double calculate_teta()
{
  return 0;  
}

void set_IdentityBedLevelingMatrix()
{
  for(int i = 0; i < 3; i++) 
	for(int j = 0; j < 3; j++)		    
	  bedLevelingMatrix[i][j] = (i == j?1:0);  
}

void set_BedLevelingMatrix()
{
  double rX = calculate_teta();
  double rY = calculate_teta();
  double rZ = 0;
  
  set_IdentityBedLevelingMatrix();  
}

void debug_tTarget(tTarget *t)
{
  sersendf(".");
  sersendf(" - X:%g Y:%g Z:%g E:%g F:%g INV:%d REL:%d\r\n", t->x, t->y, t->z, t->e, t->feed_rate, t->invert_feed_rate);
  sersendf(".\r\n");
}

void read_gcode_file(char *filename)
{
  FRESULT res;    /* FatFs function common result code */
  char *pLine;
  
  // debug("- Opening %s\r\n", filename);
  res = f_open(&file, filename, FA_OPEN_EXISTING | FA_READ);
  if (res == FR_OK)
    {
      tLineBuffer line_buf;

      pLine = f_gets(line_buf.data, sizeof(line_buf.data), &file); 
      while (pLine)
      {
        line_buf.len = strlen(pLine);
		// debug("- Parsing [%s] %d\r\n", &line_buf.data, line_buf.len);
        gcode_parse_line (&line_buf);
		// debug("- executed [%s]\r\n");
        pLine = f_gets(line_buf.data, sizeof(line_buf.data), &file);
      }
      
      res = f_close(&file);
      if (res)
        debug("- Error closing %s\r\n", filename);
      else
	    debug("- %s processed\r\n", filename);
    } else {
	  debug("- Error opening %s\r\n");
	}
}


static void enqueue_moved (tTarget *pTarget)
{
  // grbl
  tActionRequest request;

  if (pTarget->x != startpoint.x || pTarget->y != startpoint.y ||
      pTarget->z != startpoint.z || pTarget->e != startpoint.e
  )
  {
    request.ActionType = AT_MOVE;
    request.target= *pTarget;
    request.target.invert_feed_rate =  false;

    if (config.enable_extruder_1 == 0)
      request.target.e = startpoint.e;


#ifdef __SAFE_TRAVEL__
  if (pTarget->x > config.printing_vol_x || pTarget->y > config.printing_vol_y ||
      pTarget->x < 0 || pTarget->y < 0) {
      debug ("- Reached limits, not traveling to: ");	  
	  debug_tTarget(pTarget);
  } else {
    plan_buffer_action (&request);
  }
#else
    plan_buffer_action (&request);
#endif
  }
  else
  {
    // no move, just set feed rate
    plan_set_feed_rate (pTarget);
  }
}

static void enqueue_wait_temp (void)
{
  tActionRequest request;

  request.ActionType = AT_WAIT_TEMPS;  
  plan_buffer_action (&request);
}

static void enqueue_fan (int speed)
{
  tActionRequest request;

  request.ActionType = AT_FAN_SET;
  request.wait_param = speed;
  plan_buffer_action (&request);
  
}

static void enqueue_wait (void)
{
  tActionRequest request;

  request.ActionType = AT_WAIT;
  plan_buffer_action (&request);
}

// wait for move queue to be empty
static void synch_queue (void)
{
  st_synchronize();
}

static void SpecialMoveXY(double x, double y, double f) 
{
  tActionRequest request;

  request.ActionType = AT_MOVE_ENDSTOP;
  request.target.x = x;
  request.target.y = y;
  request.target.z = startpoint.z;
  request.target.e = startpoint.e;
  request.target.feed_rate = f; 
  request.target.invert_feed_rate =  false;
  plan_buffer_action (&request);
}

static void SpecialMoveZ(double z, double f) 
{
  tActionRequest request;

  request.ActionType = AT_MOVE_ENDSTOP;
  request.target.x = startpoint.x;
  request.target.y = startpoint.y;
  request.target.z = z;
  request.target.e = startpoint.e;
  request.target.feed_rate = f; 
  request.target.invert_feed_rate =  false;
  plan_buffer_action (&request);
}

static void SpecialMoveE (double e, double feed_rate) 
{
  tTarget next_targetd;

  if (config.enable_extruder_1)
  {
    next_targetd = startpoint;
    next_targetd.e = startpoint.e + e;
    next_targetd.feed_rate = feed_rate;
    enqueue_moved(&next_targetd);
  }
}

static void zero_x(void)
{
  int dir;
  int max_travel;

  if (config.home_direction_x < 0)
  {
    dir = -1;
  }
  else
  {
    dir = 1;
  }
  max_travel = max (300, config.printing_vol_x);

  // move to endstop
  SpecialMoveXY(startpoint.x + dir * max_travel, startpoint.y, config.homing_feedrate_x);
  synch_queue();

  // move forward a bit
  SpecialMoveXY(startpoint.x - dir * 3, startpoint.y, config.search_feedrate_x);
  // move back in to endstop slowly
  SpecialMoveXY(startpoint.x + dir * 6, startpoint.y, config.search_feedrate_x);

  synch_queue();

  // this is our home point
  tTarget new_pos = startpoint;
  new_pos.x = config.home_pos_x;
  plan_set_current_position (&new_pos);
}

static void zero_y(void)
{
  int dir;
  int max_travel;

  if (config.home_direction_y < 0)
  {
    dir = -1;
  }
  else
  {
    dir = 1;
  }
  max_travel = max (300, config.printing_vol_y);

  // move to endstop
  SpecialMoveXY(startpoint.x, startpoint.y + dir * max_travel, config.homing_feedrate_y);
  synch_queue();

  // move forward a bit
  SpecialMoveXY(startpoint.x, startpoint.y - dir * 3, config.search_feedrate_y);
  // move back in to endstop slowly
  SpecialMoveXY(startpoint.x, startpoint.y + dir * 6, config.search_feedrate_y);

  synch_queue();

  // this is our home point
  tTarget new_pos = startpoint;
  new_pos.y = config.home_pos_y;
  plan_set_current_position (&new_pos);
}

static void zero_z(void)
{
  int dir;
  int max_travel;

  if (config.home_direction_z < 0)
  {
    dir = -1;
  }
  else
  {
    dir = 1;
  }
  max_travel = max (300, config.printing_vol_z);

  // move to endstop
  SpecialMoveZ(startpoint.z + dir * max_travel, config.homing_feedrate_z);  
  synch_queue();

  // move forward a bit
  SpecialMoveZ(startpoint.z - dir * 1, config.search_feedrate_z);
  synch_queue();

  // move back in to endstop slowly
  SpecialMoveZ(startpoint.z + dir * 6, config.search_feedrate_z);
  synch_queue();

  // this is our home point
  tTarget new_pos = startpoint;
  new_pos.z = config.home_pos_z;

  plan_set_current_position (&new_pos);
}

double probe_z()
{
  int dir;
  int max_travel;
  double z_error;

  if (config.home_direction_z < 0)
  {
    dir = -1;
  }
  else
  {
    dir = 1;
  }
  max_travel = max (300, config.printing_vol_z);

  tTarget original_pos = startpoint;
  
  // move to endstop
  SpecialMoveZ(startpoint.z + dir * max_travel, config.homing_feedrate_z);  
  synch_queue();

  z_error = current_block_situation_at_endstop.step_event_count - current_block_situation_at_endstop.steps_z;
  sersendf(" z_error 1 : %g \r\n", z_error);
  
  // move forward a bit
  SpecialMoveZ(startpoint.z - dir * 1, config.search_feedrate_z);
  synch_queue();

  // move back in to endstop slowly
  SpecialMoveZ(startpoint.z + dir * 6, config.search_feedrate_z);
  synch_queue();

  z_error = current_block_situation_at_endstop.step_event_count - current_block_situation_at_endstop.steps_z + z_error;
  sersendf(" z_error 2 : %g \r\n", z_error);
  
  plan_set_current_position (&original_pos);
    
  return z_error;  
}

static void zero_e(void)
{
  // extruder only runs one way and we have no "endstop", just set this point as home
  //startpoint.E = current_position.E = 0;
  tTarget new_pos = startpoint;
  new_pos.e = 0;
  plan_set_current_position (&new_pos);
}

void sd_initialise(void)
{
  f_mount(0, &fs);
  sd_active = true;
}

void sd_unitialise(void)
{
  f_mount(0, NULL);
  sd_active = false;
}


FRESULT sd_list_dir_sub (char *path, int level)
{
  FRESULT res;
  FILINFO fno;
  DIR dir;
  int i;
  char *fn;

  if (level == 8) {
    debug("- maximum fs depth reached\n\r", level);  
    return FR_OK;
  }
  
  // inibhit_slow_timer = 1;
  
#if _USE_LFN
  static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1];
  fno.lfname = lfn;
  fno.lfsize = sizeof(lfn);
#endif

  debug("- opendir [%s]\n\r", path);
  res = f_opendir(&dir, path);
  debug("- opendir = %d\n\r", res);  
  if (res == FR_OK)
  {
    i = strlen(path);
    for (;;)
    {
	  debug(".\n\r");  
      res = f_readdir(&dir, &fno);
      debug(" - readdir = %d\n\r", res);  	  
      if (res != FR_OK || fno.fname[0] == 0) break;
      if (fno.fname[0] == '.') continue;
#if _USE_LFN
      fn = *fno.lfname ? fno.lfname : fno.fname;
#else
      fn = fno.fname;
#endif
      if (fno.fattrib & AM_DIR)
      {
        sersendf("%s/%s/\r\n", path, fn);

        strcat (path, "/");
        strcat (path, fn);
        // sprintf(&path[i], "/%s", fn);
        res = sd_list_dir_sub(path, level+1);
        if (res != FR_OK)
        {
          break;
        }
        path[i] = 0;
      }
      else
      {
        sersendf("%s/%s\r\n", path, fn);
      }
    }	
  }

  // inibhit_slow_timer = 0;
  
  return res;
}

char search_path[256];

void sd_list_dir (void)
{
  search_path[0] = '\0';
  sd_list_dir_sub(search_path, 0);
}

unsigned sd_open(FIL *pFile, char *path, uint8_t flags)
{
  char *reason[] = {
	" (0) Succeeded ",
	" (1) A hard error occured in the low level disk I/O layer ",
	" (2) Assertion failed ",
	" (3) The physical drive cannot work ",
	" (4) Could not find the file ",
	" (5) Could not find the path ",
	" (6) The path name format is invalid ",
	" (7) Acces denied due to prohibited access or directory full ",
	" (8) Acces denied due to prohibited access ",
	" (9) The file/directory object is invalid ",
	" (10) The physical drive is write protected ",
	" (11) The logical drive number is invalid ",
	" (12) The volume has no work area ",
	" (13) There is no valid FAT volume on the physical drive ",
	" (14) The f_mkfs() aborted due to any parameter error ",
	" (15) Could not get a grant to access the volume within defined period ",
	" (16) The operation is rejected according to the file shareing policy ",
	" (17) LFN working buffer could not be allocated ",
	" (18) Number of open files > _FS_SHARE " };

  FRESULT res;

  debug(" opening file!");
  res = f_open (pFile, path, flags);
  debug(" opened file!");

  if (res == FR_OK)
  {
    debug(" file read!");
    return 1;
  }
  else
  {
    //debug
    sersendf (" failed: reason: %d %s\r\n", res, reason[res]);
    return 0;
  }
}

void sd_close(FIL *pFile)
{
  f_close (pFile);
}

bool sd_read_file(tLineBuffer *pLine)
{
  char *ptr;

  ptr = f_gets(pLine->data, MAX_LINE, &file);

  if (ptr != NULL)
  {
    pLine->len = strlen(ptr);
    sd_pos += pLine->len;
    return true;
  }
  else
  {
    return false;
  }
}

bool sd_write_to_file(char *pStr, unsigned bytes_to_write)
{
  UINT bytes_written;
  FRESULT result;

  result = f_write (&file, pStr, bytes_to_write, &bytes_written);

  return result == FR_OK;
}

unsigned sd_filesize (FIL *pFile)
{
  return f_size(pFile);
}

void sd_seek(FIL *pFile, unsigned pos)
{
  f_lseek (pFile, pos);
}


void append_arc_movements(tTarget *next_targetd, double center_offset_x, double center_offset_y, double radius, bool is_clockwise)
{
    //debug_tTarget(next_targetd);
	
    // Scary math
    double center_x = startpoint.x + center_offset_x;
    double center_y = startpoint.y + center_offset_y;
    double vertical_travel = next_targetd->z - startpoint.z;
    double r_axis_x = -center_offset_x; // Radius vector from center to current location
    double r_axis_y = -center_offset_y;
    double rt_axis_x = next_targetd->x - center_x;
    double rt_axis_y = next_targetd->y - center_y;

    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    double angular_travel = atan2(r_axis_x * rt_axis_y - r_axis_y * rt_axis_x, r_axis_x * rt_axis_x + r_axis_y * rt_axis_y);
    if (angular_travel < 0) {
        angular_travel += 2 * M_PI;
    }
    if (is_clockwise) {
        angular_travel -= 2 * M_PI;
    }

    // Find the distance for this gcode
    double millimeters_of_travel = hypotf(angular_travel * radius, fabs(vertical_travel));

    // We don't care about non-XYZ moves ( for example the extruder produces some of those )
    if( millimeters_of_travel < 0.0001F ) {
        return;
    }

    // Figure out how many segments for this gcode
    uint16_t segments = floor(millimeters_of_travel / config.mm_per_arc_segment);

    double theta_per_segment = angular_travel / segments;
    double linear_per_segment = vertical_travel / segments;	
	
	/* 
	
	Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
    and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
    r_T = [cos(phi) -sin(phi);
    sin(phi) cos(phi] * r ;
    For arc generation, the center of the circle is the axis of rotation and the radius vector is
    defined from the circle center to the initial position. Each line segment is formed by successive
    vector rotations. This requires only two cos() and sin() computations to form the rotation
    matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
    all float numbers are single precision on the Arduino. (True float precision will not have
    round off issues for CNC applications.) Single precision error can accumulate to be greater than
    tool precision in some cases. Therefore, arc path correction is implemented.

    Small angle approximation may be used to reduce computation overhead further. This approximation
    holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
    theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
    to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
    numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
    issue for CNC machines with the single precision Arduino calculations.
    This approximation also allows mc_arc to immediately insert a line segment into the planner
    without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
    a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
    This is important when there are successive arc motions.
    
	*/
	
    // Vector rotation matrix values
	
    double cos_T = 1 - 0.5F * theta_per_segment * theta_per_segment; // Small angle approximation
    double sin_T = theta_per_segment;
	
	tTarget arc_target;
    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    int8_t count = 0;
	
	// Initialize the linear axis
    arc_target.z = startpoint.z;
	
	double d = millimeters_of_travel / segments;
    double e_per_segment = d * extruder_1_speed / next_targetd->feed_rate * 24.0;

    arc_target.e = startpoint.e; 
	arc_target.feed_rate = next_targetd->feed_rate;

    for (i = 1; i < segments; i++) { // Increment (segments-1)

        if (count < config.arc_correction ) {
            // Apply vector rotation matrix
            r_axisi = r_axis_x * sin_T + r_axis_y * cos_T;
            r_axis_x = r_axis_x * cos_T - r_axis_y * sin_T;
            r_axis_y = r_axisi;
            count++;
        } else {
            // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
            // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
            cos_Ti = cosf(i * theta_per_segment);
            sin_Ti = sinf(i * theta_per_segment);
            r_axis_x = -center_offset_x * cos_Ti + center_offset_y * sin_Ti;
            r_axis_y = -center_offset_x * sin_Ti - center_offset_y * cos_Ti;
            count = 0;
        }

        // Update arc_target location
        arc_target.x = center_x + r_axis_x;
        arc_target.y = center_y + r_axis_y;
        arc_target.z += linear_per_segment;		
		arc_target.e += e_per_segment;

        //debug_tTarget(&arc_target);
		
        // Append this segment to the queue
        enqueue_moved(&arc_target);
	  
    }
	
	// Set e movement to 0, the arc is done. Just position head at the target location.
	next_targetd->e = arc_target.e;
	
	// Ensure last segment arrives at target location.
    enqueue_moved(next_targetd);

}

void prepare_arc_movement(tTarget *next_targetd, double center_offset_x, double center_offset_y, int motion_mode)
{
   // Find the radius
   float radius = hypotf(center_offset_x, center_offset_y);

    // Set clockwise/counter-clockwise sign for mc_arc computations
    bool is_clockwise = false;
    if( motion_mode == MOTION_MODE_CW_ARC ) {
        is_clockwise = true;
    }

    // Append arc
    append_arc_movements(next_targetd, center_offset_x, center_offset_y, radius, is_clockwise );
}

void append_circle_movements(tTarget *next_targetd, double radius, double teta, int motion_mode)
{
  tTarget tt_targetd;

  // Find the distance for this gcode
  double millimeters_of_travel = 2 * M_PI * radius;
  
  // We don't care about non-XYZ moves ( for example the extruder produces some of those )
  if( millimeters_of_travel < 0.1F ) {
    return;
  }

  // Figure out how many segments for this gcode
  uint16_t segments = floor(millimeters_of_travel / config.mm_per_arc_segment);
  double segment_arc = 2 * M_PI / segments;
  
  if (motion_mode == MOTION_MODE_CCW_ARC) {
    segment_arc = -segment_arc;
  }

  double center_x = next_targetd->x;
  double center_y = next_targetd->y;
  // Setup interpolated mid points temp structure		  
  memcpy(&tt_targetd, next_targetd, sizeof(tTarget));

  // Move to circle edge
  tt_targetd.feed_rate = config.maximum_feedrate_x;
  tt_targetd.x = center_x + cos(teta) * radius;
  tt_targetd.y = center_y + sin(teta) * radius;
  tt_targetd.e = startpoint.e;
  enqueue_moved (&tt_targetd);
	
  // Set feed_rate	
  tt_targetd.feed_rate = next_targetd->feed_rate;
  
  // Print circle
  int segment = 0;
  while(segment != segments) {
  
    // Next segment
    segment++;
	
    teta += segment_arc;
	
	//debug_tTarget(next_targetd);
	
	if (next_target.option_relative) {
	  // sersendf(" - rel - %g\r\n", tt_targetd.e / segments);
	  tt_targetd.e = next_targetd->e / segments;	 
	} else {
	  // sersendf(" - abs - %g\r\n", startpoint.e + next_targetd->e / segments);
	  tt_targetd.e = startpoint.e + next_targetd->e / segments;	 
	}

    // Move to circle edge    
    tt_targetd.x = center_x + cos(teta) * next_target.R;
    tt_targetd.y = center_y + sin(teta) * next_target.R;		      
	enqueue_moved (&tt_targetd);    
  }	
}

void prepare_circle_movement(tTarget *next_targetd, double radius, int motion_mode)
{
  double teta;
		  
  if (config.circle_start_random == 1) {
	srand(millis());
	int v = rand();
	teta = 2 * M_PI * ((double) v / RAND_MAX);
  } else {
	teta = config.circle_start_angle * M_PI / 180.0;
  }
		
  // Make the circle
  append_circle_movements(next_targetd, radius, teta, motion_mode);
}

/****************************************************************************
 *                                                                           *
 * Command Received - process it                                             *
 *                                                                           *
 ****************************************************************************/

double feed_rate_extruder_multiplier = 1;
double feed_rate_movement_multiplier = 1;

bool enqueue_fan_mode = false;
	
eParseResult process_gcode_command()
{
  double backup_f;
  uint8_t axisSelected = 0;
  eParseResult result = PR_OK;
  bool reply_sent = false; // 'ok' already sent?
  
  tTarget next_targetd = startpoint;

  // convert relative to absolute
  if (next_target.option_relative)
  {
    next_targetd.x = startpoint.x + next_target.target.x;
    next_targetd.y = startpoint.y + next_target.target.y;
    next_targetd.z = startpoint.z + next_target.target.z;
    next_targetd.e = startpoint.e + next_target.target.e;
    if (next_target.seen_F)
      next_targetd.feed_rate = next_target.target.feed_rate * feed_rate_movement_multiplier;
  }
  else
  {
    // absolute
    if (next_target.seen_X)
      next_targetd.x = next_target.target.x;
    if (next_target.seen_Y)
      next_targetd.y = next_target.target.y;
    if (next_target.seen_Z)
      next_targetd.z = next_target.target.z;
    if (next_target.seen_E)
      next_targetd.e = next_target.target.e;
    if (next_target.seen_F)
      next_targetd.feed_rate = next_target.target.feed_rate * feed_rate_movement_multiplier;
  }

  //  sersendf(" X:%ld Y:%ld Z:%ld E:%ld F:%ld\r\n", (int32_t)next_target.target.X, (int32_t)next_target.target.Y, (int32_t)next_target.target.Z, (int32_t)next_target.target.E, (uint32_t)next_target.target.F);
  //  sersendf(" X:%g Y:%g Z:%g E:%g F:%g\r\n", next_targetd.x, next_targetd.y, next_targetd.z, next_targetd.e, next_targetd.feed_rate);

  // E ALWAYS absolute 
  // host should periodically reset E with "G92 E0", otherwise we overflow our registers after only a few layers

  if (next_target.seen_G)
  {
    switch (next_target.G)
    {
    // G0 - rapid, unsynchronised motion
    // since it would be a major hassle to force the dda to not synchronise, just provide a fast feedrate and hope it's close enough to what host expects
      case 0:
        backup_f = next_targetd.feed_rate;
        next_targetd.feed_rate = config.maximum_feedrate_x * 2;
        enqueue_moved (&next_targetd);
        next_targetd.feed_rate = backup_f;
      break;

      // G1 - synchronised motion
      case 1:
      if ( (extruders_on == EXTRUDER_NUM_1) && !next_target.seen_E)
      {
        // approximate translation for 3D code. distance to extrude is move distance times extruder speed factor
        //TODO: extrude distance for Z moves
        double d = calc_distance (ABS(next_targetd.x - startpoint.x), ABS(next_targetd.y - startpoint.y));

        next_targetd.e = startpoint.e + d * extruder_1_speed / next_targetd.feed_rate * 24.0;
      }
      enqueue_moved(&next_targetd);
      break;

      //	G2 - Arc Clockwise
	  case 2:
	    if (next_target.seen_R) {
		  prepare_circle_movement(&next_targetd, next_target.R, MOTION_MODE_CW_ARC);
		} 
		
		if (next_target.seen_I && next_target.seen_J) {
	      prepare_arc_movement(&next_targetd, next_target.I, next_target.J, MOTION_MODE_CW_ARC);
		}
	  break;      

      //	G3 - Arc Counter-clockwise
	  case 3:
	    if (next_target.seen_R) {
		  prepare_circle_movement(&next_targetd, next_target.R, MOTION_MODE_CCW_ARC);
		} 
		
		if (next_target.seen_I && next_target.seen_J) {
	      prepare_arc_movement(&next_targetd, next_target.I, next_target.J, MOTION_MODE_CCW_ARC);
		}
	  break;      


      //	G4 - Dwell
      case 4:
      // wait for all moves to complete
      synch_queue();

      // delay
      delay_s(next_target.P);
      break;

      //	G20 - inches as units
      case 20:
      next_target.option_inches = 1;
      break;

      //	G21 - mm as units
      case 21:
      next_target.option_inches = 0;
      break;

      //	G30 - go home via point
      case 30:
        enqueue_moved(&next_targetd);
      // no break here, G30 is move and then go home

      //	G28 - go home
      case 28:
      if (next_target.seen_X)
      {
        zero_x();
        axisSelected = 1;
      }

      if (next_target.seen_Y)
      {
        zero_y();
        axisSelected = 1;
      }

      if (next_target.seen_Z)
      {
        zero_z();
        axisSelected = 1;
      }

      if (next_target.seen_E)
      {
        zero_e();
        axisSelected = 1;
      }

      if(!axisSelected)
      {
        if (config.machine_model == MM_RAPMAN)
        {
          // move stage down to clear Z endstop
          // Rapman only?
          next_targetd = startpoint;
          next_targetd.z += 3;
          next_targetd.feed_rate = config.homing_feedrate_z;
          enqueue_moved(&next_targetd);
        }

        zero_x();
        zero_y();
        zero_z();
        zero_e();
      }

      //!      startpoint.F = config.homing_feedrate_x;  //?
      break;

      // G90 - absolute positioning
      case 90:
      next_target.option_relative = 0;
      break;

      // G91 - relative positioning
      case 91:
        next_target.option_relative = 1;
      break;

      //	G92 - set current position
      case 92:
      {
        tTarget new_pos;

        // must have no moves pending if changing position
        synch_queue();

        new_pos = startpoint;

        if (next_target.seen_X)
        {
          new_pos.x = next_target.target.x;
          axisSelected = 1;
        }

        if (next_target.seen_Y)
        {
          new_pos.y = next_target.target.y;
          axisSelected = 1;
        }

        if (next_target.seen_Z)
        {
          new_pos.z = next_target.target.z;
          axisSelected = 1;
        }

        if (next_target.seen_E)
        {
          new_pos.e = 0;
          axisSelected = 1;
        }

        if(!axisSelected)
        {
          new_pos.x = 0;
          new_pos.y = 0;
          new_pos.z = 0;
          new_pos.e = 0;
        }

        plan_set_current_position (&new_pos);
      }
      break;

      // unknown gcode: spit an error
      default:
        sersendf("E: Bad G-code - G");
        serwrite_uint8(next_target.G);
        sersendf("\r\n");	  
    }
  }
  else if (next_target.seen_M)
  {
    switch (next_target.M)
    {
      // SD File functions
      case 20: // M20 - list SD Card files
	  
	  if (!sd_active)
	  {
        sd_initialise();
      }
	  
      sersendf("Begin file list\r\n"); // This is part of the protocol!
      // list files in root folder
      sd_list_dir();
      sersendf("End file list\r\n"); // This is also part of the protocol!
			
      break;

      case 21: // M21 - init SD card
        sd_printing = false;
        sd_initialise();
        // NB : assume that the disk has been mounted in config.c
        // TODO: mount volume here and change config.c
      break;

      case 22: // M22 - release SD card
        sd_printing = false;                
		sd_unitialise();
      break;

      case 23: // M23 <filename> - Select file
      if (!sd_active)
      {
        sd_initialise();
      }
      if(sd_active)
      {
        sd_printing = false;
        sd_close(&file);
        if (sd_open(&file, next_target.filename, FA_READ))
        {
          filesize = sd_filesize(&file);
          sersendf("File opened: %s Size: %d\r\n", next_target.filename, filesize);
          sd_pos = 0;
          sersendf("File selected\r\n");
        }
        else
        {
          sersendf("file.open failed\r\n");
        }		
      }
      break;

      case 24: //M24 - Start SD print
      if(sd_active)
      {
        sd_printing = true;
      }
      break;

      case 25: //M25 - Pause SD print
      if(sd_printing)
      {
        sd_printing = false;
      }
      break;

      case 26: //M26 - Set SD file pos
      if(sd_active && next_target.seen_S)
      {
        sd_pos = next_target.S;  // 16 bit
        sd_seek(&file, sd_pos);
      }
      break;

      case 27: //M27 - Get SD status
      if(sd_active)
      {
        sersendf("SD printing byte %d/%d\r\n", sd_pos, filesize);
		debug("SD printing byte %d/%d\r\n", sd_pos, filesize);
      }
      else
      {
        serial_writestr("Not SD printing\r\n");
      }      
      break;

      case 30: //M30 <filename>
	    if ((sd_active) && (!sd_printing)) { // Don't try to delete file if sd not active or during printing
		  FRESULT res;
		  
		  res = f_unlink(next_target.filename);
		  
		  if (res == FR_OK) {
		    serial_writestr("File deleted\r\n"); // Repetier expects this tring upon file removal
		    debug("File deleted: %s\r\n", next_target.filename);
		  } else {
		    serial_writestr("Deletion failed\r\n"); // Repetier expects this tring upon file removal
		    debug("Deletion failed: %s\r\n", next_target.filename);
		  }
		}
	  break;
	  
      case 28: //M28 <filename> - Start SD write
      if (!sd_active)
      {
        sd_initialise();
      }
      if(sd_active)
      {
        sd_close(&file);
        sd_printing = false;

        if (!sd_open(&file, next_target.filename, FA_CREATE_ALWAYS | FA_WRITE))
        {
          sersendf("open failed, File: %s.\r\n", next_target.filename);
		  debug("open failed, File: %s.\r\n", next_target.filename);
        }
        else
        {
          sd_writing_file = true;
          sersendf("Writing to file: %s\r\n", next_target.filename);
		  debug("Writing to file: %s\r\n", next_target.filename);
		  written_lines = 0;
        }
      }	  
      break;

      case 29: //M29 - Stop SD write
      // processed in gcode_parse_char()
	  // This is never reached!
      break;

      case 80: // M80 - Turn on ATX
	  // no-op, not used on R2C2
	  break;
	  
	  case 81: // M80 - Turn off ATX
	  // no-op, not used on R2C2
	  break;
	  
      case 82: // M82 - use absolute distance for extrusion
      // no-op, we always do absolute
      break;

      // M101- extruder on
      case 101:
      extruders_on = EXTRUDER_NUM_1;
      if (auto_prime_steps != 0)
      {
        SpecialMoveE ((double)auto_prime_steps / auto_prime_factor, auto_prime_feed_rate);
      }

      break;

      // M102- extruder reverse

      // M103- extruder off
      case 103:
      extruders_on = 0;
      if (auto_reverse_steps != 0)
      {
        SpecialMoveE (-(double)auto_reverse_steps / auto_reverse_factor, auto_reverse_feed_rate);
      }
      break;

      // M104- set temperature
      case 104:

      if (config.enable_extruder_1)
      {
        temp_set(EXTRUDER_0, next_target.S);

        if (config.wait_on_temp)
        {
          enqueue_wait_temp();
        }		
      }

      break;

      // M105- get temperature
      case 105:
        temp_print();
        reply_sent = true;
      break;

      // M106- fan on
      case 106:	    
	    
	    if (next_target.seen_S) { // Parameter S? // Yes, use parameter
		  if (enqueue_fan_mode) { 
		    enqueue_fan(next_target.S); // here
		  } else {
		    synch_queue();
  	        extruder_fan_set(next_target.S); // and here    
		  }
		} else { // no! // set to maximum
	      if (enqueue_fan_mode) {	        
		    enqueue_fan(255); 
		  } else {
            synch_queue();
		    extruder_fan_set(255);			
		  }
		}
      break;

      // M107- fan off
      case 107:
	    if (enqueue_fan_mode) {
		  enqueue_fan(0);      	
		} else {
	      synch_queue();
		  extruder_fan_set(0);
		}
      break;

      // M108 - set extruder speed
      // S = RPM * 10
      case 108:
      if (next_target.seen_S)
      {
        extruder_1_speed = (double)next_target.S / 10.0 * feed_rate_extruder_multiplier;
      }
      break;

      // M109- set temp and wait
      case 109:
      if (config.enable_extruder_1)
      {	  
        temp_set(EXTRUDER_0, next_target.S);
        enqueue_wait_temp();
      }
      break;

      // M110- set line number
      case 110:	    
	    if (next_target.seen_S) {
          next_target.N_expected = next_target.S; //- 1;
		} else {
		  next_target.N_expected = 1;
		}		
      break;

      // M111- set debug level
      case 111:
      //debug_flags = next_target.S;
      break;

      // M112- immediate stop
      case 112:
		disableHwTimer(0); // disable stepper ?        
        //queue_flush(); // error: " undefined reference to `queue_flush'" ??

        // disable extruder and bed heaters
        temp_set(EXTRUDER_0,0);
        temp_set(HEATED_BED_0,0);
        power_off();		
      break;

      // M113- extruder PWM
      case 113:
      break;

      /* M114- report XYZE to host */
      case 114:
      
	    enqueue_wait();

        if (next_target.option_inches)
        {

        }
        else
        {
          sersendf("ok C: X:%g Y:%g Z:%g E:%g\r\n", startpoint.x, startpoint.y, startpoint.z, startpoint.e);
	  	  reply_sent = true;
        }
      
      break;

      // M115- report firmware version
      case 115:
		sersendf("FIRMWARE_NAME:Teacup_R2C2 FIRMWARE_URL:https://github.com/racribeiro/R2C2_Firmware PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel\r\n");
      break;

      // M116 - Wait for all temperatures and other slowly-changing variables to arrive at their set values.
      case 116:
      if (config.enable_extruder_1)
      {
        enqueue_wait();
      }
      break;

      case 119:
      // M119 - Get Endstop Status
#if (X_MIN_PIN > -1)
      serial_writestr ("x_min:");
      serial_writestr ( x_min() ? "H ":"L ");
#endif
#if (Y_MIN_PIN > -1)
      serial_writestr ("y_min:");
      serial_writestr ( y_min() ? "H ":"L ");
#endif
#if (Z_MIN_PIN > -1)
      serial_writestr ("z_min:");
      serial_writestr ( z_min() ? "H ":"L ");
#endif
      serial_writestr ("\r\n");
      break;

      // M130- heater P factor
      case 130:
	  
	  if ( (!next_target.seen_P) || (next_target.P == 0) ) { // if no P, off mode pid definition. if P = 0, off mode pid definition
		
		if (next_target.seen_S)
			config.p_factor_extruder_1 = next_target.S / 1000.0;
		if (next_target.seen_X)
			config.p_factor_extruder_1 = next_target.target.x;
		if (next_target.seen_Y)
			config.i_factor_extruder_1 = next_target.target.y;
		if (next_target.seen_Z)
			config.d_factor_extruder_1 = next_target.target.z;	
		if (next_target.seen_I)
			config.max_extruder_1 = next_target.I;
		if (next_target.seen_J)
			config.min_extruder_1 = next_target.J;			
			
	  } else {
	  
	    if (next_target.seen_S)
			config.p_factor_extruder_fan_1 = next_target.S / 1000.0;
		if (next_target.seen_X)
			config.p_factor_extruder_fan_1 = next_target.target.x;
		if (next_target.seen_Y)
			config.i_factor_extruder_fan_1 = next_target.target.y;
		if (next_target.seen_Z)
			config.d_factor_extruder_fan_1 = next_target.target.z;	
		if (next_target.seen_I)
			config.max_extruder_fan_1 = next_target.I;
		if (next_target.seen_J)
			config.min_extruder_fan_1 = next_target.J;			

	  }
		
	  temp_init_sensor(EXTRUDER_0, config.temp_sample_rate, config.temp_buffer_duration);
	  
      break;
      // M131- heater I factor
      case 131:
      if (next_target.seen_S)
        config.i_factor_extruder_1 = next_target.S  / 1000.0;
		
	  temp_init_sensor(EXTRUDER_0, config.temp_sample_rate, config.temp_buffer_duration);
		
      break;

      // M132- heater D factor
      case 132:
        if (next_target.seen_S)
          config.d_factor_extruder_1 = next_target.S  / 1000.0;
		
	    temp_init_sensor(EXTRUDER_0, config.temp_sample_rate, config.temp_buffer_duration);		
      break;

      // M133- heated bed P=X, I=Y, D=Z, MAX=I, MIN=J
      case 133:
	  
	    if (next_target.seen_X)
          config.p_factor_heated_bed_0 = next_target.target.x;
        if (next_target.seen_Y)
          config.i_factor_heated_bed_0 = next_target.target.y;
        if (next_target.seen_Z)
          config.d_factor_heated_bed_0 = next_target.target.z;		
        if (next_target.seen_I)
          config.max_heated_bed_0 = next_target.I;
        if (next_target.seen_J)
          config.min_heated_bed_0 = next_target.J;		
		
	    temp_init_sensor(HEATED_BED_0, config.temp_sample_rate, config.temp_buffer_duration);
	
      break;

      // M134- save PID settings to eeprom
      case 134:
        //heater_save_settings();
      break;

      /* M140 - Bed Temperature (Fast) */
      case 140:
        temp_set(HEATED_BED_0,next_target.S);
      break;

      /* M141 - Chamber Temperature (Fast) */
      case 141:
      break;

      /* M142 - Bed Holding Pressure */
      case 142:
      break;

      // M190- power on
      case 190:
        power_on();
        x_enable();
        y_enable();
        z_enable();
        e_enable();
        steptimeout = 0;
      break;

      // M191- power off
      case 191:
        x_disable();
        y_disable();
        z_disable();
        e_disable();
        power_off();
      break;

      // M200 - set steps per mm
      case 200:
      if ((next_target.seen_X | next_target.seen_Y | next_target.seen_Z | next_target.seen_E) == 0)
      {
        reply_sent = true;
        sersendf ("ok X%g Y%g Z%g E%g\r\n",
            config.steps_per_mm_x,
            config.steps_per_mm_y,
            config.steps_per_mm_z,
            config.steps_per_mm_e
        );
      }
      else
      {
        if (next_target.seen_X)
        {
          config.steps_per_mm_x = next_target.target.x;
        }
        if (next_target.seen_Y)
        {
          config.steps_per_mm_y = next_target.target.y;
        }
        if (next_target.seen_Z)
        {
          config.steps_per_mm_z = next_target.target.z;
        }
        if (next_target.seen_E)
        {
          config.steps_per_mm_e = next_target.target.e;
        }

        gcode_parse_init();
      }
      break;// M551 - Prime extruder 1
      // P : number of steps
      // S : RPM * 10

      // M202 - set max speed in mm/min
      case 202:
      if ((next_target.seen_X | next_target.seen_Y | next_target.seen_Z | next_target.seen_E) == 0)
      {
        reply_sent = true;
        sersendf ("ok X%d Y%d Z%d E%d\r\n",
            config.maximum_feedrate_x,
            config.maximum_feedrate_y,
            config.maximum_feedrate_z,
            config.maximum_feedrate_e
        );
      }
      else
      {
        if (next_target.seen_X)
        {
          config.maximum_feedrate_x = next_target.target.x;
        }
        if (next_target.seen_Y)
        {
          config.maximum_feedrate_y = next_target.target.y;
        }
        if (next_target.seen_Z)
        {
          config.maximum_feedrate_z = next_target.target.z;
        }
        if (next_target.seen_E)
        {
          config.maximum_feedrate_e = next_target.target.e;
        }
      }
      break;

      // M206 - set accel in mm/sec^2
      case 206:
      if ((next_target.seen_X | next_target.seen_Y | next_target.seen_Z | next_target.seen_E) == 0)
      {
        reply_sent = true;
        sersendf ("ok X%g\r\n",
            config.acceleration
        );
      }
      else
      {
        if (next_target.seen_X)
          config.acceleration = next_target.target.x;
      }
      break;

    case 220:
      if (next_target.seen_S)
      {        
	    next_target.target.feed_rate = next_target.target.feed_rate / feed_rate_movement_multiplier;
		feed_rate_movement_multiplier = (double)next_target.S / 100.0;
		next_target.target.feed_rate = next_target.target.feed_rate * feed_rate_movement_multiplier;
      }
      break;

    case 221:
      if (next_target.seen_S)
      {
	    extruder_1_speed = extruder_1_speed / feed_rate_extruder_multiplier;
        feed_rate_extruder_multiplier = (double)next_target.S / 100.0;
		extruder_1_speed = extruder_1_speed * feed_rate_extruder_multiplier;

      }
      break;
	  
      // M227 - Enable Auto-prime/reverse (steps)
      // P: prime on start (steps)
      // S: reverse on stop (steps)
      case 227:
      if (next_target.seen_S && next_target.seen_P)
      {
        auto_prime_steps = next_target.S;
        auto_reverse_steps = next_target.P;
      }
      break;

      // M228 - Disable Auto-prime/reverse
      case 228:
        auto_prime_steps = 0;
        auto_reverse_steps = 0;
      break;

      // M229 - Enable Auto-prime/reverse
      // P: prime on start (rotations)
      // S: reverse on stop (rotations)
      case 229:
      if (next_target.seen_S && next_target.seen_P)
      {
        auto_prime_steps = next_target.S * config.steps_per_revolution_e;
        auto_reverse_steps = next_target.P * config.steps_per_revolution_e;
      }
      break;

      // M300 - beep
      // S: frequency
      // P: duration
      case 300:
      {
        uint16_t frequency = 1000;  // 1kHz
        uint16_t duration = 1000; // 1 second

        if (next_target.seen_S)
        {
          frequency = next_target.S;
        }
        if (next_target.seen_P)
        {
          duration = next_target.P;
        }

        buzzer_play (frequency, duration);
      }
      break;

      // Plays Music from Static Library
      case 301:
      {
        //play_jingle_bell();
		rtttl_play_axelf();
      }
      break;

      // Plays Music from command line:
      // Usage:
      
      // Example:
      
      //
      case 302:
      {
	    /*
        if (next_target.seen_P) {
          set_whole_note_time(next_target.P);
        }
        play_music_string(next_target.filename);    
		*/
		
		rtttl_play(next_target.filename);
		sersendf ("ok [%s]\r\n", next_target.filename);
		reply_sent = true;
      }
      break;
	  
      // M500 - set/get adc value for temperature
      // S: temperature (degrees C, 0-300)
      // P: ADC val
      case 500:
      if (next_target.seen_S && next_target.seen_P)
      {
        temp_set_table_entry (EXTRUDER_0, next_target.S, next_target.P);
      }
      else if (next_target.seen_S)
      {
        reply_sent = true;
        sersendf ("ok [%d] = %d\r\n", next_target.S, temp_get_table_entry (EXTRUDER_0, next_target.S));
      }
      else
      {
        serial_writestr ("E: bad param\r\n");
      }
      break;

      // M501 - set/get adc value for temperature
      // S: temperature (degrees C, 0-300)
      // P: ADC val
      case 501:
      if (next_target.seen_S && next_target.seen_P)
      {
        temp_set_table_entry (HEATED_BED_0, next_target.S, next_target.P);
      }
      else if (next_target.seen_S)
      {
        reply_sent = true;
        sersendf ("ok [%d] = %d\r\n", next_target.S, temp_get_table_entry (HEATED_BED_0, next_target.S));
      }
      else
      {
        serial_writestr ("E: bad param\r\n");
      }
      break;

      // M542 - nozzle wipe/move to rest location
      case 542:
      // TODO: this depends on current origin being same as home position
      if (config.have_rest_pos || config.have_wipe_pos)
      {
        // move above bed if ncessary
        if (startpoint.z < 2)
        {
          next_targetd = startpoint;
          next_targetd.z = 2;
          next_targetd.feed_rate = config.maximum_feedrate_z;
          enqueue_moved(&next_targetd);
        }

        if (config.have_wipe_pos)
        {
          // move to start of wipe area
          next_targetd.x = config.wipe_entry_pos_x;
          next_targetd.y = config.wipe_entry_pos_y;
          next_targetd.z = startpoint.z;
          next_targetd.feed_rate = config.maximum_feedrate_x;
        }
        else
        {
          // move to rest position
          next_targetd.x = config.rest_pos_x;
          next_targetd.y = config.rest_pos_y;
          next_targetd.z = startpoint.z;
          next_targetd.feed_rate = config.maximum_feedrate_x;
        }

        enqueue_moved(&next_targetd);
      }
      break;

      // M543 - exit nozzle wipe/no op
      case 543:
      if (config.have_wipe_pos)
      {
        // move out of wipe area
        next_targetd.x = config.wipe_exit_pos_x;
        next_targetd.y = config.wipe_exit_pos_y;
        next_targetd.z = startpoint.z;
        next_targetd.feed_rate = config.maximum_feedrate_x;
        enqueue_moved(&next_targetd);

        next_targetd.x = config.wipe_entry_pos_x;
        next_targetd.y = config.wipe_entry_pos_y;
        next_targetd.z = startpoint.z;
        next_targetd.feed_rate = config.maximum_feedrate_x;
        enqueue_moved(&next_targetd);

      }
      break;

      // M551 - Prime extruder 1
      // P : number of steps
      // S : RPM * 10
      case 551:
      if (next_target.seen_S && next_target.seen_P)
      {
        // calc E distance, use approximate conversion to get distance, not critical
        // TODO: how to derive magic number
        // S is RPM*10, but happens to give about the right speed in mm/min
        SpecialMoveE ((double)next_target.P / 256.0, next_target.S);
      }
      break;

      // M600 print the values read from the config file
      case 600:
      {	    
        print_config();		
      }
      break;

      case 601:
      {
        write_config();
      }
      break;

      //set home position
      case 605:
      {
        if (next_target.seen_X)
        {
          config.home_pos_x = next_target.target.x;
          axisSelected = 1;
        }//no need for else

        if (next_target.seen_Y)
        {
          config.home_pos_y = next_target.target.y;
          axisSelected = 1;
        }//no need for else

        if (next_target.seen_Z)
        {
          config.home_pos_z = next_target.target.z;
          axisSelected = 1;
        }//no need for else

        if(!axisSelected)
        {
          config.home_pos_x = 0.0;
          config.home_pos_y = 0.0;
          config.home_pos_z = 0.0;
        }//no need for else
      }
      break;

      // M606 - wait for empty movement queue
      case 606:
        enqueue_wait();
      break;

      // M701 - Temperature PID debug
      case 701:
	    sersendf("debug [%s]\r\n", temp_debug(EXTRUDER_0));		
		sersendf("debug E current_pid = %g\r\n", get_pid_val(EXTRUDER_0));
		sersendf("debug E current_temp = %d\r\n", temp_get(EXTRUDER_0));
		sersendf("debug E target_temp = %d\r\n", temp_get_target(EXTRUDER_0));		
		sersendf("debug E P:I:D = %g : %g : %g\r\n", config.p_factor_extruder_1, config.i_factor_extruder_1, config.d_factor_extruder_1);  
		sersendf("debug E MIN= %g MAX=%g\r\n", config.min_extruder_1, config.max_extruder_1);  		
	  break;
				
      case 702:
		sersendf("debug [%s]\r\n", temp_debug(HEATED_BED_0));
		sersendf("debug H current_pid = %g\r\n", get_pid_val(HEATED_BED_0));
		sersendf("debug H current_temp = %d\r\n", temp_get(HEATED_BED_0));
		sersendf("debug H target_temp = %d\r\n", temp_get_target(HEATED_BED_0));
		sersendf("debug H P:I:D = %g : %g : %g\r\n", config.p_factor_heated_bed_0, config.i_factor_heated_bed_0, config.d_factor_heated_bed_0);
		sersendf("debug H MIN= %g MAX=%g\r\n", config.min_heated_bed_0, config.max_heated_bed_0);  				
	  break;
	  
	  // M703 - Extruder Fan Value debug
      case 703:
	    sersendf("debug [%s]\r\n", temp_debug(EXTRUDER_0_FAN));				
		sersendf("debug F current_power = %d\r\n", temp_get(EXTRUDER_0_FAN));
		sersendf("debug F target_power = %d\r\n", temp_get_target(EXTRUDER_0_FAN));		
	  break;
	  
	  // SERIAL COMMANDS
	  
	  case 710:
	    uart_sendf("%s\r\n", next_target.filename);
	  break;
	  
	  
	  // STACK COMMANDS
	  
	  case 720:
	    uart_sendf("Stack Height: %d\r\n", targetStackPos);
	  break;
	  
	  case 721:
	    
		enqueue_wait();
		if (targetStackPos < MAX_TARGET_STACK) {		          
		  uart_sendf("Push\r\n");
		  memcpy(&targetStack[targetStackPos++], &startpoint, sizeof(startpoint));
		} else {
		  uart_sendf("Stack full\r\n");
		}
	  break;
	  
	  case 722:	    
		if (targetStackPos > 0) {		  
		  enqueue_moved(&targetStack[--targetStackPos]);
		  uart_sendf("Pop\r\n");
		} else {
		  uart_sendf("Stack empty\r\n");
		}
	  break;

	  case 730:
	    enqueue_fan_mode = false;
	  break;

      case 731:
	    enqueue_fan_mode = true;
	  break;
	  
	  // BedLeveling 74x commands
	  
	  // 
	  case 740: // Reset bed leveling points
	    set_IdentityBedLevelingMatrix();
	    bedLevelingDefined[0] = false;
		bedLevelingDefined[1] = false;
		bedLevelingDefined[2] = false;
		bedLevelingActive = false;
	  break;

      case 741: // Sets Point S1, S2, S3
	    if (bedLevelingActive) {
		  sersendf("Can't set points while bedlevel is activated!\r\n");
		} else
	    if (next_target.seen_S) {
		  if ((next_target.S >= 1) && (next_target.S <= BED_LEVELING_POINTS)) {
		    int this_point = next_target.S - 1;
		    bedLevelingDefined[this_point] = true;			
		    // memcpy(&bedLeveling[this_point], &startpoint, sizeof(startpoint));
            bedLeveling[this_point] = startpoint;
			
			if (this_point == 0) {
			  zero_z();
			  bedLeveling[this_point].z = 0;
			} else {
			  sersendf("Probing...\r\n");
			  bedLeveling[this_point].z = probe_z();
			  sersendf("Done!\r\n");
			}
		  }
		}
	  break;
	  
	  case 742: // (re)activate BedLeveling
	    if (bedLevelingDefined[0] && bedLevelingDefined[1] && bedLevelingDefined[2]) {
		  sersendf("Bedleveling enabled!\r\n");
		  bedLevelingActive = true;
		  // Calculate and set transformation matrix 
		  set_IdentityBedLevelingMatrix();
		  sersendf("Bedleveling activated!\r\n");
		}
      break;
	  
	  case 743: // Deactivate BedLeveling
	    bedLevelingActive = false;
		// Update transformation matrix to identity matrix
		set_IdentityBedLevelingMatrix();		
		sersendf("Bedleveling deactivated!");		
      break;

	  case 749: // Prints debug information
	    		
		if (bedLevelingActive) {
		  sersendf("Bedleveling active!\r\n");		
		} else {
		  sersendf("Bedleveling not active!\r\n");		
		}
		
		for(int i = 0; i < BED_LEVELING_POINTS; i++) {
		  if (bedLevelingDefined[i]) {
		    sersendf("P%d : X=%g Y=%g Z=%g\r\n", i, bedLeveling[i].x, bedLeveling[i].y, bedLeveling[i].z);		  
	      } else {
		    sersendf("P%d : not defined!\r\n", i);
		  }		  
		}
		
		for(int i = 0; i < 3; i++) 
		  for(int j = 0; j < 3; j++) {{		    
		    sersendf(" %g ", bedLevelingMatrix[i][j]);
		  }		  
		  sersendf("\r\n");
		}
		
      break;
	  
	  
				
      // unknown mcode: spit an error
      default:
        serial_writestr("E: Bad M-code - M");
        serwrite_uint8(next_target.M);
        serial_writestr("\r\n");
    }
  }

  if (!reply_sent)
  {
    serial_writestr("ok\r\n");
    //sersendf("ok Q:%d\r\n", plan_queue_size());
  }

  return result;
}
