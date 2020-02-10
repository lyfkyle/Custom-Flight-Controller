#include "logging.h"
#include "UAV_Defines.h"

#include "controller_att_rate.h"

/*
 * Defines
 */

#define LOG_TAG ("CntrllerAttRate")

/*
 * Code
 */
AttRateController::AttRateController(int periodMs) :
    mKp(0),
    mKd(0),
    mKi(0),
    mPeriodMs(periodMs),
    mAttRatePID(&mCurAttRate, &mOutput, &mAttRateSetpoint, mKp, mKi, mKd, PID_P_ON_E, PID_CTRL_DIR_DIRECT)
{
    mAttRateSetpoint = 0;
    mCurAttRate = 0;
    mOutput = 0;
    mAttRatePID.SetTunings(mKp, mKi, mKd);
    mAttRatePID.SetSampleTime(mPeriodMs);
    mAttRatePID.SetOutputLimits(ATT_RATE_PID_OUT_MIN, ATT_RATE_PID_OUT_MAX);
    mAttRatePID.SetMode(1);
    // TODO read from preference manager/flash?
}

float AttRateController::GetDesiredMotorThrust(float attRateSetpoint, float curAttRate)
{
    mAttRateSetpoint = attRateSetpoint;
    mCurAttRate = curAttRate;

    // run PID, the output is automatically stored into mAccOutput
    mAttRatePID.Compute();
    LOGI("GetDesiredMotorThrust: setpoint: %f, cur val: %f, output: %f\r\n", mAttRateSetpoint, mCurAttRate, mOutput);

    // additional handling?
    // feedforward?

    return mOutput;
}

bool AttRateController::SetPID(float kp, float kd, float ki)
{
    mKp = kp;
    mKd = kd;
    mKi = ki;
    mAttRatePID.SetTunings(mKp, mKi, mKd);
    return true;
}

bool AttRateController::SetKp(float kp)
{
    mKp = kp;
    mAttRatePID.SetTunings(mKp, mKi, mKd);
    return true;
}

bool AttRateController::SetKd(float kd)
{
    mKd = kd;
    mAttRatePID.SetTunings(mKp, mKi, mKd);
    return true;
}

bool AttRateController::SetKi(float ki)
{
    mKi = ki;
    mAttRatePID.SetTunings(mKp, mKi, mKd);
    return true;
}

bool AttRateController::SetPeriodMs(int periodMs)
{
    mPeriodMs = periodMs;
    mAttRatePID.SetSampleTime(mPeriodMs);
    return true;
}

float AttRateController::GetKp()
{
    return mKp;
}

float AttRateController::GetKd()
{
    return mKd;
}

float AttRateController::GetKi()
{
    return mKi;
}
