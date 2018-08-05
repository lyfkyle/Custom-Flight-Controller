
#include "controller_att_rate.h"

/*
 * Code
 */
AttRateController::AttRateController() :
   mAttRatePID(&mCurAttRate, &mOutput, &mAttRateSetpoint, mKp, mKd, mKi, PID_CTRL_DIR_DIRECT)
{
   mAttRateSetpoint = 0;
   mCurAttRate = 0;
   mOutput = 0;

   // TODO read from preference manager/flash?
   mKp = 0;
   mKd = 0;
   mKi = 0;
}

int AttRateController::GetDesiredMotorPWM(float attRateSetpoint, float curAttRate)
{
   mAttRateSetpoint = attRateSetpoint;
   mCurAttRate = curAttRate;

   // run PID, the output is automatically stored into mAccOutput
   mAttRatePID.Compute();

   // additional handling?
   // feedforward?

   int output = (int) mOutput;
   return output;
}

bool AttRateController::SetPID(float kp, float kd, float ki)
{
   mKp = kp;
   mKd = kd;
   mKi = ki;

   return true;
}

bool AttRateController::SetKp(float kp)
{
   mKp = kp;
   return true;
}

bool AttRateController::SetKd(float kd)
{
   mKd = kd;
   return true;
}

bool AttRateController::SetKi(float ki)
{
   mKi = ki;
   return true;
}