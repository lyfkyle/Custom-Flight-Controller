#ifndef _CONTROLLER_ATT_RATE_H_
#define _CONTROLLER_ATT_RATE_H_

#include <PID.h>

/*
 * Defines
 */
#define ATT_RATE_CONTROLLER_FREQUENCY (1000)
#define ATT_RATE_CONTROLLER_DTIME     (0.001)

class AttRateController
{
public:
   AttRateController();
   float GetDesiredMotorThrust(float attSetpoint, float curAtt);
   bool SetPID(float kp, float kd, float ki);
   bool SetKp(float kp);
   bool SetKd(float kd);
   bool SetKi(float ki);
private:
   PID mAttRatePID;
   float mAttRateSetpoint;
   float mCurAttRate;
   float mOutput;
   float mKp;
   float mKd;
   float mKi;
};



#endif // _CONTROLLER_ATT_RATE_H_