#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "controller_att.h"
#include "controller_att_rate.h"
#include "controller_util.h"
#include "controller_vel.h"
#include "motor_ctrl.h"

class Controller
{
public:
   Controller();
   bool Run();
private:
   VelController mVelController_X;
   VelController mVelController_Y;
   VelController mVelController_Z;
   AttController mAttController_pitch;
   AttController mAttController_roll;
   AttController mAttController_yaw;
   AttRateController mAttRateController_pitch;
   AttRateController mAttRateController_roll;
   AttRateController mAttRateController_yaw;
   MotorCtrl& mMotorCtrl;
};

#endif