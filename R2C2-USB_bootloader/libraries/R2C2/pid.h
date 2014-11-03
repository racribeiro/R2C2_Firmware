#ifndef _PID_
#define _PID_

#include "lpc17xx.h"

/*
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
*/			  
			  
struct  _pid {
    int16_t *pv;  	/*pointer to an integer that contains the process value*/
    int16_t *sp;  	/*pointer to an integer that contains the set point*/ 
    double integral;
    double pgain;
    double igain;
    double dgain;
    double deadband;
    int16_t last_error;
};

void pid_init(struct _pid *a, int16_t *pv, int16_t *sp);
void pid_tune(struct _pid *a, double p_gain, double i_gain, double d_gain, double dead_band);
void get_gains(struct _pid *a, double *p_gain, double *i_gain, double *d_gain, double *dead_band);
void pid_setinteg(struct _pid *a, double new_integ);
void pid_bumpless(struct _pid *a);
double pid_calc(struct _pid *pid);

#endif  /* _PID_ */

