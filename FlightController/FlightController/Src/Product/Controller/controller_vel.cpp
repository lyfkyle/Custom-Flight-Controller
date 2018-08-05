
#include "controller_vel.h"

/*
 * Code
 */
    VelController::VelController() :
   mVelPID(&mCurVel, &mAccOutput, &mVelSetpoint, mKp, mKd, mKi, PID_CTRL_DIR_DIRECT)
{
   mVelSetpoint = 0;
   mCurVel = 0;
   mAccOutput = 0;

   // TODO read from preference manager/flash?
   mKp = 0;
   mKd = 0;
   mKi = 0;
}

float VelController::GetDesiredAcc(float velSetpoint, float curVel)
{
   mVelSetpoint = velSetpoint;
   mCurVel = curVel;

   // run PID, the output is automatically stored into mAccOutput
   mVelPID.Compute();

   // additional handling?
   // feedforward?

   return mAccOutput;
}

bool VelController::SetPID(float kp, float kd, float ki)
{
   mKp = kp;
   mKd = kd;
   mKi = ki;

   return true;
}

bool VelController::SetKp(float kp)
{
   mKp = kp;
   return true;
}

bool VelController::SetKd(float kd)
{
   mKd = kd;
   return true;
}

bool VelController::SetKi(float ki)
{
   mKi = ki;
   return true;
}