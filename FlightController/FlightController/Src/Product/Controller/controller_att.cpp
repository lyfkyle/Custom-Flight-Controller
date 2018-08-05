
#include "controller_att.h"

/*
 * Code
 */
AttController::AttController() :
   mAttPID(&mCurAtt, &mAttRateOutput, &mAttSetpoint, mKp, mKd, mKi, PID_CTRL_DIR_DIRECT)
{
   mAttSetpoint = 0;
   mCurAtt = 0;
   mAttRateOutput = 0;

   // TODO read from preference manager/flash?
   mKp = 0;
   mKd = 0;
   mKi = 0;
}

float AttController::GetDesiredAttRateSetpoint(float attSetpoint, float curAtt)
{
   mAttSetpoint = attSetpoint;
   mCurAtt = curAtt;

   // run PID, the output is automatically stored into mAccOutput
   mAttPID.Compute();

   // additional handling?
   // feedforward?

   return mAttRateOutput;
}

bool AttController::SetPID(float kp, float kd, float ki)
{
   mKp = kp;
   mKd = kd;
   mKi = ki;

   return true;
}

bool AttController::SetKp(float kp)
{
   mKp = kp;
   return true;
}

bool AttController::SetKd(float kd)
{
   mKd = kd;
   return true;
}

bool AttController::SetKi(float ki)
{
   mKi = ki;
   return true;
}