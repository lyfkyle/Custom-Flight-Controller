
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
    // mMotorCtrl = MotorCtrl::GetInstance();
}

bool Controller::Run()
{
    // vel setpoints
    // TODO
    float velSetpoint_x;
    float velSetpoint_y;
    float velSetpoint_z;
    float yawSetpoint;
    float curVel_x;
    float curVel_y;
    float curVel_z;
    float curPitch;
    float curRoll;
    float curYaw;
    float curAttRate_pitch;
    float curAttRate_roll;
    float curAttRate_yaw;

    // get acc
    float xAccSetpoint = mVelController_X.GetDesiredAcc(velSetpoint_x, curVel_x);
    float yAccSetpoint = mVelController_Y.GetDesiredAcc(velSetpoint_y, curVel_y);
    float zAccSetpoint = mVelController_Z.GetDesiredAcc(velSetpoint_z, curVel_z);

    // from acc to att
    FCAttType attSetpoint;
    Controller_GetAttSetpointFromAccSetpoint(&attSetpoint, xAccSetpoint, yAccSetpoint, zAccSetpoint, yawSetpoint);

    // att controller
    float attRateSetpoint_pitch = mAttController_pitch.GetDesiredAttRateSetpoint(attSetpoint.pitch, curPitch);
    float attRateSetpoint_roll = mAttController_roll.GetDesiredAttRateSetpoint(attSetpoint.roll, curRoll);
    float attRateSetpoint_yaw = mAttController_yaw.GetDesiredAttRateSetpoint(attSetpoint.yaw, curYaw);

    // attRate
    float pitchThrust = mAttRateController_pitch.GetDesiredMotorThrust(attRateSetpoint_pitch, curAttRate_pitch);
    float rollThrust = mAttRateController_roll.GetDesiredMotorThrust(attRateSetpoint_roll, curAttRate_roll);
    float yawThrust = mAttRateController_yaw.GetDesiredMotorThrust(attRateSetpoint_yaw, curAttRate_yaw);
    float heightThrust = zAccSetpoint;

    mMotorCtrl.OutputMotor(pitchThrust, rollThrust, yawThrust, heightThrust);

    return true;
}