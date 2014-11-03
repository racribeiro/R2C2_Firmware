#include "pid.h"

/*----------------------------------------------------------------- -------
pid_init

DESCRIPTION   This function initializes the pointers in the _pid structure
              to the process variable and the setpoint.  *pv and *sp are
              integer pointers.
------------------------------------------------------------------- -----*/
void pid_init(struct _pid *a, int16_t *pv, int16_t *sp)
{
    a->pv = pv;
    a->sp = sp;
}


/*----------------------------------------------------------------- -------
pid_tune

DESCRIPTION   Sets the proportional gain (p_gain), integral gain (i_gain),
              derivitive gain (d_gain), and the dead band (dead_band) of
              a pid control structure _pid.
------------------------------------------------------------------- -----*/
void pid_tune(struct _pid *a, double p_gain, double i_gain, double d_gain, double dead_band)
{
    a->pgain = p_gain;
    a->igain = i_gain;
    a->dgain = d_gain;
    a->deadband = dead_band;
    a->integral=0.0;
    a->last_error=0;
}

/*----------------------------------------------------------------- -------
get_gains

DESCRIPTION   Returns the gains and dead band in a _pid control structure
              in the locations pointed to by the p_gain, i_gain, d_gain,
              and dead_band pointers.
              
ALSO SEE      pid_tune              
------------------------------------------------------------------- -----*/
void get_gains(struct _pid *a, double *p_gain, double *i_gain, double *d_gain, double *dead_band)

{
    *p_gain = a->pgain;
    *i_gain = a->igain;
    *d_gain = a->dgain;
    *dead_band = a->deadband;
}



/*----------------------------------------------------------------- -------
pid_setinteg

DESCRIPTION   Set a new value for the integral term of the pid equation.
              This is useful for setting the initial output of the
              pid controller at start up.
------------------------------------------------------------------- -----*/
void pid_setinteg(struct _pid *a, double new_integ)
{
    a->integral=new_integ;
    a->last_error=0;
}

/*----------------------------------------------------------------- -------
pid_bumpless

DESCRIPTION   Bumpless transfer algorithim.  When suddenly changing
              setpoints, or when restarting the PID equation after an
              extended pause, the derivative of the equation can cause 
              a bump in the controller output.  This function will help 
              smooth out that bump. The process value in *pv should
              be the updated just before this function is used.
------------------------------------------------------------------- -----*/
void pid_bumpless(struct _pid *a)
{
    a->last_error = *(a->sp) - *(a->pv);
}
    
/*----------------------------------------------------------------- -------
pid_calc

DESCRIPTION   Performs PID calculations for the _pid structure *a.  This
              function uses the positional form of the pid equation, and
              incorporates an integral windup prevention algorithim.
              Rectangular integration is used, so this function must be
              repeated on a consistent time basis for accurate control.

RETURN VALUE  The new output value for the pid loop.

USAGE         #include "control.h"
              main() {
                   struct _pid PID;
                   int process_variable, set_point;
                   pid_init(&PID, &process_variable, &set_point);
                   pid_tune(&PID, 4.3, 0.2, 0.1, 2);
                   set_point = 500;
                   pid_setinteg(&PID,30.0);
                   process_varialbe = read_temp();
                   pid_bumpless(&PID);
                   for(;;) {
                        process_variable = read_temp();
                        output( pid_calc(&PID) );
                        wait(1.0);
                   }
              }
------------------------------------------------------------------- -----*/

double math_abs(double a)
{
  if (a < 0) {
     return 0-a;
  }
  return a;
}

double pid_calc(struct _pid *pid)
{
  int16_t err;
  double pterm, dterm, result, ferror;

  err = (pid->sp) - (pid->pv);
  if (math_abs(err) > pid->deadband) 
  {
    ferror = (float) err; /*do integer to float conversion only once*/
    pterm = pid->pgain * ferror;
    if (pterm > 100 || pterm < -100) 
      pid->integral = 0.0;
    else
    {
      pid->integral += pid->igain * ferror;
      if (pid->integral > 100.0) 
	    pid->integral = 100.0;
      else 
	    if (pid->integral < 0.0) 
		  pid->integral = 0.0;
    }
    dterm = ((float)(err - pid->last_error)) * pid->dgain;
    result = pterm + pid->integral + dterm;
  }
  else 
    result = pid->integral;
	
  pid->last_error = err;
  return (result);
}

double pid_calc_2(struct _pid *a)
{
    int16_t err;
    double pterm, dterm, iterm, ferror;
    err = *(a->sp) - *(a->pv);
    
    ferror = (double) err;    /*do integer to float conversion only once*/
    pterm = a->pgain * ferror;
	iterm = a->igain * a->integral;
	dterm = a->dgain * (ferror - a->last_error);    
	a->integral += ferror;
	a->last_error = ferror;
    
    return (pterm + iterm + dterm) * a->deadband;
}

