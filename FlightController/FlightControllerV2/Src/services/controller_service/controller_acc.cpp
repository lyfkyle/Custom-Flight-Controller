#include "UAV_Defines.h"

#include "controller_acc.h"

/*
 * Code
 */
AccController::AccController(int periodMs) :
    mKp(0),
    mKd(0),
    mKi(0),
    mPeriodMs(periodMs),
    mAccPID(&mCurAcc, &mOutput, &mAccSetpoint, mKp, mKi, mKd, PID_P_ON_E, PID_CTRL_DIR_DIRECT)
{
    mAccSetpoint = 0;
    mCurAcc = 0;
    mOutput = 0;
    mAccPID.SetTunings(mKp, mKi, mKd);
    mAccPID.SetSampleTime(mPeriodMs);
    mAccPID.SetOutputLimits(ACC_PID_OUT_MIN, ACC_PID_OUT_MAX);
    mAccPID.SetMode(1);

    // TODO read from preference manager/flash?
}

bool AccController::SetOutputLimits(float min, float max)
{
    mAccPID.SetOutputLimits(min, max);
    return true;
}

float AccController::GetOutput(float accSetpoint, float curAcc)
{
   mAccSetpoint = accSetpoint;
   mCurAcc = curAcc;

   // run PID, the output is automatically stored into mAccOutput
   mAccPID.Compute();

   // additional handling?
   // feedforward?

   return mOutput;
}

bool AccController::SetPID(float kp, float kd, float ki)
{
   mKp = kp;
   mKd = kd;
   mKi = ki;

   return true;
}

bool AccController::SetKp(float kp)
{
   mKp = kp;
   return true;
}

bool AccController::SetKd(float kd)
{
   mKd = kd;
   return true;
}

bool AccController::SetKi(float ki)
{
   mKi = ki;
   return true;
}