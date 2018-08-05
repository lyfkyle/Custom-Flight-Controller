
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
   AttController();
   float GetDesiredAttRateSetpoint(float attSetpoint, float curAtt);
   bool SetPID(float kp, float kd, float ki);
   bool SetKp(float kp);
   bool SetKd(float kd);
   bool SetKi(float ki);
private:
   PID mAttPID;
   float mAttSetpoint;
   float mCurAtt;
   float mAttRateOutput;
   float mKp;
   float mKd;
   float mKi;
};



#endif // _CONTROLLER_ATT_H_