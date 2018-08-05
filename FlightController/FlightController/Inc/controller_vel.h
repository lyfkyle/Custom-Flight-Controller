
#ifndef _CONTROLLER_VEL_H_
#define _CONTROLLER_VEL_H_

#include <PID.h>

/*
 * Defines
 */
#define VEL_CONTROLLER_FREQUENCY (100)
#define VEL_CONTROLLER_DTIME     (0.01)

class VelController
{
public:
   VelController();
   float GetDesiredAcc(float velSetpoint, float curVel);
   bool SetPID(float kp, float kd, float ki);
   bool SetKp(float kp);
   bool SetKd(float kd);
   bool SetKi(float ki);
private:
   PID mVelPID;
   float mVelSetpoint;
   float mCurVel;
   float mAccOutput;
   float mKp;
   float mKd;
   float mKi;
};



#endif // _CONTROLLER_VEL_H_