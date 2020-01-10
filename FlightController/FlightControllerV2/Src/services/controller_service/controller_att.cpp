#include "logging.h"
#include "UAV_Defines.h"

#include "controller_att.h"

/*
 * Defines
 */

#define LOG_TAG ("CntrllerAtt")

/*
 * Code
 */
AttController::AttController(int periodMs) :
    mKp(0),
    mKd(0),
    mKi(0),
    mPeriodMs(periodMs),
    mAttPID(&mCurAtt, &mAttRateOutput, &mAttSetpoint, mKp, mKi, mKd, PID_P_ON_E, PID_CTRL_DIR_DIRECT)
{
    mAttSetpoint = 0;
    mCurAtt = 0;
    mAttRateOutput = 0;
    mAttPID.SetTunings(mKp, mKi, mKd);
    mAttPID.SetSampleTime(mPeriodMs);
    mAttPID.SetOutputLimits(-125, 125);
    mAttPID.SetMode(1);
    // TODO read from preference manager/flash?
}

float AttController::GetDesiredAttRateSetpoint(float attSetpoint, float curAtt)
{
    mAttSetpoint = attSetpoint;
    mCurAtt = curAtt;

    // run PID, the output is automatically stored into mAccOutput
    // LOG("Kp: %f, Kd: %f, Ki %f\r\n", mAttPID.GetKp(), mAttPID.GetKd(), mAttPID.GetKi());
    bool res = mAttPID.Compute();
    if (!res) {
        LOGE("mAttPID.Compute() failed\r\n");
    } else {
        LOGI("AttController::GetDesiredAttRateSetpoint: setpoint: %f, cur val: %f, mAttRateOutput %f\r\n", mAttSetpoint, mCurAtt, mAttRateOutput);
    }

    // additional handling?
    // feedforward?

    return mAttRateOutput;
}

bool AttController::SetPID(float kp, float ki, float kd)
{
    mKp = kp;
    mKd = kd;
    mKi = ki;
    mAttPID.SetTunings(mKp, mKi, mKd);
    return true;
}

bool AttController::SetKp(float kp)
{
    mKp = kp;
    mAttPID.SetTunings(mKp, mKi, mKd);
    return true;
}

bool AttController::SetKd(float kd)
{
    mKd = kd;
    mAttPID.SetTunings(mKp, mKi, mKd);
    return true;
}

bool AttController::SetKi(float ki)
{
    mKi = ki;
    mAttPID.SetTunings(mKp, mKi, mKd);
    return true;
}

bool AttController::SetPeriodMs(int periodMs)
{
    mPeriodMs = periodMs;
    mAttPID.SetSampleTime(mPeriodMs);
    return true;
}