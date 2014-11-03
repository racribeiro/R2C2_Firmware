/**********************************************************************************************
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include "pid2.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void PID_PID(struct PID *p, int16_t* Input, double* Output, int16_t* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
{
	
    p->myOutput = Output;
    p->myInput = Input;
    p->mySetpoint = Setpoint;
	p->inAuto = false;
	
	PID_SetOutputLimits(p, 0, 255);				//default output limit corresponds to 
												//the arduino pwm limits

    p->SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    PID_SetControllerDirection(p, ControllerDirection);
    PID_SetTunings(p, Kp, Ki, Kd);

    p->lastTime = millis() - p->SampleTime;				
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PID_Compute(struct PID *p)
{
   if(!p->inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - p->lastTime);
   if(timeChange>=p->SampleTime)
   {
      /*Compute all the working error variables*/
	  double input = *p->myInput;
      double error = *p->mySetpoint - input;
      p->ITerm+= (p->ki * error);
      if(p->ITerm > p->outMax) p->ITerm= p->outMax;
      else if(p->ITerm < p->outMin) p->ITerm= p->outMin;
      double dInput = (input - p->lastInput);
 
      /*Compute PID Output*/
      double output = p->kp * error + p->ITerm - p->kd * dInput;
      
	  if(output > p->outMax) output = p->outMax;
      else if(output < p->outMin) output = p->outMin;
	  *p->myOutput = output;
	  
      /*Remember some variables for next time*/
      p->lastInput = input;
      p->lastTime = now;
	  return true;
   }
   else return false;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID_SetTunings(struct PID *p, double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   p->dispKp = Kp; p->dispKi = Ki; p->dispKd = Kd;
   
   double SampleTimeInSec = ((double)p->SampleTime)/1000;  
   p->kp = Kp;
   p->ki = Ki * SampleTimeInSec;
   p->kd = Kd / SampleTimeInSec;
 
  if(p->controllerDirection == REVERSE)
   {
      p->kp = (0 - p->kp);
      p->ki = (0 - p->ki);
      p->kd = (0 - p->kd);
   }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID_SetSampleTime(struct PID *p, int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)p->SampleTime;
      p->ki *= ratio;
      p->kd /= ratio;
      p->SampleTime = (unsigned long)NewSampleTime;
   }
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_SetOutputLimits(struct PID *p, double Min, double Max)
{
   if(Min >= Max) return;
   p->outMin = Min;
   p->outMax = Max;
 
   if(p->inAuto)
   {
	   if(*p->myOutput > p->outMax) *p->myOutput = p->outMax;
	   else if(*p->myOutput < p->outMin) *p->myOutput = p->outMin;
	 
	   if(p->ITerm > p->outMax) p->ITerm= p->outMax;
	   else if(p->ITerm < p->outMin) p->ITerm= p->outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID_SetMode(struct PID *p, int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !p->inAuto)
    {  /*we just went from manual to auto*/
        PID_Initialize(p);
    }
   p->inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID_Initialize(struct PID *p)
{
   p->ITerm = *p->myOutput;
   p->lastInput = *p->myInput;
   if(p->ITerm > p->outMax) p->ITerm = p->outMax;
   else if(p->ITerm < p->outMin) p->ITerm = p->outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID_SetControllerDirection(struct PID *p, int Direction)
{
   if(p->inAuto && Direction !=p->controllerDirection)
   {
	  p->kp = (0 - p->kp);
      p->ki = (0 - p->ki);
      p->kd = (0 - p->kd);
   }   
   p->controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID_GetKp(struct PID *p){ return  p->dispKp; }
double PID_GetKi(struct PID *p){ return  p->dispKi;}
double PID_GetKd(struct PID *p){ return  p->dispKd;}
int PID_GetMode(struct PID *p){ return  p->inAuto ? AUTOMATIC : MANUAL;}
int PID_GetDirection(struct PID *p){ return p->controllerDirection;}

