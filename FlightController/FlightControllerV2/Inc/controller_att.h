
#ifndef _CONTROLLER_ATT_H_
#define _CONTROLLER_ATT_H_

#include <PID.h>

/*
 * Defines
 */
#define _ATT_CONTROLLER_FREQUENCY (1000)
#define _ATT_CONTROLLER_DTIME     (0.001)

class AttController
{
public:
   AttController(int periodMs);
   float GetDesiredAttRateSetpoint(float attSetpoint, float curAtt);
   bool SetPID(float kp, float ki, float kd);
   bool SetKp(float kp);
   bool SetKd(float kd);
   bool SetKi(float ki);
   float GetKp();
   float GetKd();
   float GetKi();
   bool SetPeriodMs(int periodMs);
private:
   PID mAttPID;
   float mAttSetpoint;
   float mCurAtt;
   float mAttRateOutput;
   float mKp;
   float mKd;
   float mKi;
   int mPeriodMs;
};

#endif // _CONTROLLER_ATT_H_