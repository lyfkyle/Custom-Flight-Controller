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
   AttRateController(int sampleTimeMs);
   float GetDesiredMotorThrust(float attSetpoint, float curAtt);
   bool SetPID(float kp, float kd, float ki);
   bool SetKp(float kp);
   bool SetKd(float kd);
   bool SetKi(float ki);
   bool SetPeriodMs(int periodMs);
private:
   PID mAttRatePID;
   float mAttRateSetpoint;
   float mCurAttRate;
   float mOutput;
   float mKp;
   float mKd;
   float mKi;
   int mPeriodMs;
};



#endif // _CONTROLLER_ATT_RATE_H_