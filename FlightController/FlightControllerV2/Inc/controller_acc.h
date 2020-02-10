
#ifndef _CONTROLLER_VEL_H_
#define _CONTROLLER_VEL_H_

#include <PID.h>

/*
 * Defines
 */
#define VEL_CONTROLLER_FREQUENCY (100)
#define VEL_CONTROLLER_DTIME     (0.01)

class AccController
{
public:
   AccController(int periodMs);
   float GetOutput(float velSetpoint, float curVel);
   bool SetPID(float kp, float kd, float ki);
   bool SetKp(float kp);
   bool SetKd(float kd);
   bool SetKi(float ki);
   bool SetOutputLimits(float min, float max);

private:
   PID mAccPID;
   float mAccSetpoint;
   float mCurAcc;
   float mOutput;
   float mKp;
   float mKd;
   float mKi;
   int mPeriodMs;
};



#endif // _CONTROLLER_VEL_H_