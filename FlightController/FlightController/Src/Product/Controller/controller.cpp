
#include "controller.h"
#include "controller_att.h"
#include "controller_att_rate.h"
#include "controller_util.h"
#include "controller_vel.h"

Controller::Controller() :
   mVelController_X(),
   mVelController_Y(),
   mVelController_Z(),
   mAttController_pitch(),
   mAttController_roll(),
   mAttController_yaw(),
   mAttRateController_pitch(),
   mAttRateController_roll(),
   mAttRateController_yaw()
{
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
   int output1 = mAttRateController_pitch.GetDesiredMotorPWM(attRateSetpoint_pitch, curAttRate_pitch);
   int output2 = mAttRateController_roll.GetDesiredMotorPWM(attRateSetpoint_roll, curAttRate_roll);
   int output3 = mAttRateController_yaw.GetDesiredMotorPWM(attRateSetpoint_yaw, curAttRate_yaw);

   return true;
}