
#include "controller.h"

Controller::Controller() :
    mVelController_X(),
    mVelController_Y(),
    mVelController_Z(),
    mAttController_pitch(),
    mAttController_roll(),
    mAttController_yaw(),
    mAttRateController_pitch(),
    mAttRateController_roll(),
    mAttRateController_yaw(),
    mMotorCtrl(MotorCtrl::GetInstance())
{
}

Controller& Controller::GetInstance()
{
    static Controller controller;
    return controller;
}

bool Controller::Run()
{
    // vel setpoints
    // TODO

    // get acc
//    float xAccSetpoint = mVelController_X.GetDesiredAcc(velSetpoint_x, curVel_x);
//    float yAccSetpoint = mVelController_Y.GetDesiredAcc(velSetpoint_y, curVel_y);
//    float zAccSetpoint = mVelController_Z.GetDesiredAcc(velSetpoint_z, curVel_z);

    // from acc to att
    Controller_GetAttSetpointFromAccSetpoint(mAttSetpoint, mAccSetpoint, mAttSetpoint.yaw);

    // att controller
    mAttRateSetpoint.pitch = mAttController_pitch.GetDesiredAttRateSetpoint(mAttSetpoint.pitch, mCurAtt.pitch);
    mAttRateSetpoint.roll = mAttController_roll.GetDesiredAttRateSetpoint(mAttSetpoint.roll, mCurAtt.roll);
    mAttRateSetpoint.yaw = mAttController_yaw.GetDesiredAttRateSetpoint(mAttSetpoint.yaw, mCurAtt.yaw);

    // attRate
    float pitchThrust = mAttRateController_pitch.GetDesiredMotorThrust(mAttRateSetpoint.pitch, mCurAttRate.pitch);
    float rollThrust = mAttRateController_roll.GetDesiredMotorThrust(mAttRateSetpoint.roll, mCurAttRate.roll);
    float yawThrust = mAttRateController_yaw.GetDesiredMotorThrust(mAttRateSetpoint.yaw, mCurAttRate.yaw);
    float heightThrust = GetHeightThrustFromAccSetpointZ(mAccSetpoint.z);

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

bool Controller::SetYawSetpoint(float yawAngle)
{
    mAttSetpoint.yaw = yawAngle;
    return true;
}