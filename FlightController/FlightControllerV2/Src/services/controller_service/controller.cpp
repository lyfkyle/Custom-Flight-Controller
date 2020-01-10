#include "controller.h"

#include "logging.h"

#define LOG_TAG ("Controller")

Controller::Controller() :
    mVelController_X(DEFAULT_VEL_PERIOD_MS),
    mVelController_Y(DEFAULT_VEL_PERIOD_MS),
    mVelController_Z(DEFAULT_VEL_PERIOD_MS),
    mAttController_pitch(DEFAULT_ATT_PERIOD_MS),
    mAttController_roll(DEFAULT_ATT_PERIOD_MS),
    mAttController_yaw(DEFAULT_ATT_PERIOD_MS),
    mAttRateController_pitch(DEFAULT_ATT_RATE_PERIOD_MS),
    mAttRateController_roll(DEFAULT_ATT_RATE_PERIOD_MS),
    mAttRateController_yaw(DEFAULT_ATT_RATE_PERIOD_MS),
    mMotorCtrl(MotorCtrl::GetInstance())
{
}

Controller& Controller::GetInstance()
{
    static Controller controller;
    return controller;
}

bool Controller::Init()
{
    return ControllerUtil_Init();
}

bool Controller::SetPeriodMs(int periodMs)
{
    // right now all controller runs on the same frequency
    mPeriodMs = periodMs;
    mAttController_pitch.SetPeriodMs(periodMs);
    mAttController_roll.SetPeriodMs(periodMs);
    mAttController_yaw.SetPeriodMs(periodMs);
    mAttRateController_pitch.SetPeriodMs(periodMs);
    mAttRateController_roll.SetPeriodMs(periodMs);
    mAttRateController_yaw.SetPeriodMs(periodMs);
    return true;
}

bool Controller::Run()
{
    // vel setpoints
    // TODO

    // get acc
//    float xAccSetpoint = mVelController_X.GetDesiredAcc(velSetpoint_x, curVel_x);
//    float yAccSetpoint = mVelController_Y.GetDesiredAcc(velSetpoint_y, curVel_y);
//    float zAccSetpoint = mVelController_Z.GetDesiredAcc(velSetpoint_z, curVel_z);
    // vel controller not used, directly treat velocity as acc.
    mAccSetpoint.x = mVelSetpoint.x;
    mAccSetpoint.y = mVelSetpoint.y;
    mAccSetpoint.z = mVelSetpoint.z;

    // from accSetpoint to attSetpoint
    Controller_GetAttSetpointFromAccSetpoint(mAttSetpoint, mAccSetpoint, mAttSetpoint.yaw);

    // att controller
    mAttRateSetpoint.pitch = mAttController_pitch.GetDesiredAttRateSetpoint(mAttSetpoint.pitch, mCurAtt.pitch);
    mAttRateSetpoint.roll = mAttController_roll.GetDesiredAttRateSetpoint(mAttSetpoint.roll, mCurAtt.roll);
    // mAttRateSetpoint.yaw = mAttController_yaw.GetDesiredAttRateSetpoint(mAttSetpoint.yaw, mCurAtt.yaw); // no need to control yaw angle

    // attRate
    float pitchThrust = mAttRateController_pitch.GetDesiredMotorThrust(mAttRateSetpoint.pitch, mCurAttRate.pitch);
    float rollThrust = mAttRateController_roll.GetDesiredMotorThrust(mAttRateSetpoint.roll, mCurAttRate.roll);
    float yawThrust = mAttRateController_yaw.GetDesiredMotorThrust(mAttRateSetpoint.yaw, mCurAttRate.yaw);
    float heightThrust = GetHeightThrustFromVelSetpointZ(mVelSetpoint.z);

    LOGI("Thrust: pitch: %f roll: %f yaw: %f height: %f\r\n", pitchThrust, rollThrust, yawThrust, heightThrust);
    mMotorCtrl.OutputMotor(pitchThrust, rollThrust, yawThrust, heightThrust);

    return true;
}

bool Controller::SetAccSetpoint(FCAccDataType& accSetpoint)
{
    mAccSetpoint.x = accSetpoint.x;
    mAccSetpoint.y = accSetpoint.y;
    mAccSetpoint.z = accSetpoint.z;
    return true;
}

bool Controller::SetVelSetpoint(FCVelDataType& velSetpoint)
{
    mVelSetpoint.x = velSetpoint.x;
    mVelSetpoint.y = velSetpoint.y;
    mVelSetpoint.z = velSetpoint.z;
    return true;
}

bool Controller::SetCurAtt(FCAttType& att)
{
    mCurAtt.pitch = att.pitch;
    mCurAtt.roll = att.roll;
    mCurAtt.yaw = att.yaw;
    return true;
}

bool Controller::SetCurAttRate(FCAttType& attRate)
{
    mCurAttRate.pitch = attRate.pitch;
    mCurAttRate.roll = attRate.roll;
    mCurAttRate.yaw = attRate.yaw;
    return true;
}

bool Controller::SetYawRateSetpoint(float yawRate)
{
    mAttRateSetpoint.yaw = yawRate;
    return true;
}