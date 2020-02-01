#include "UAV_Defines.h"

#include "controller_vel.h"

/*
 * Code
 */
VelController::VelController(int periodMs) :
    mKp(PID_VEL_KP),
    mKd(PID_VEL_KD),
    mKi(PID_VEL_KI),
    mPeriodMs(periodMs),
    mVelPID(&mCurVel, &mAccOutput, &mVelSetpoint, mKp, mKi, mKd, PID_P_ON_E, PID_CTRL_DIR_DIRECT)
{
    mVelSetpoint = 0;
    mCurVel = 0;
    mAccOutput = 0;
    mVelPID.SetTunings(mKp, mKi, mKd);
    mVelPID.SetSampleTime(mPeriodMs);
    mVelPID.SetOutputLimits(UAV_PID_OUT_MIN, UAV_PID_OUT_MAX);
    mVelPID.SetMode(1);

    // TODO read from preference manager/flash?
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