#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "controller_att.h"
#include "controller_att_rate.h"
#include "controller_util.h"
#include "controller_acc.h"
#include "motor_ctrl.h"
#include "UAV_Defines.h"

/*
 * Defines
 */

#define DEFAULT_VEL_PERIOD_MS (200)
#define DEFAULT_ATT_PERIOD_MS (100)
#define DEFAULT_ATT_RATE_PERIOD_MS (100)
#define DEFAULT_HEIGHT_RATE_PERIOD_MS (100)

class Controller
{
public:
    static Controller& GetInstance();

    bool SetPeriodMs(int periodMs);
    bool SetAttPeriodMs(int periodMs);
    bool SetAttRatePeriodMs(int periodMs);
    bool Init();
    bool SetAccSetpoint(FCAccDataType& accSetpoint);
    bool SetVelSetpoint(FCVelDataType& velSetpoint);
    bool SetYawRateSetpoint(float yawRate);
    bool SetAttSetpoint(FCAttType& attSetpoint);
    bool SetAttRateSetpoint(FCAttRateType& attRateSetpoint);

    bool SetCurAtt(FCAttType& att);
    bool SetCurAttRate(FCAttType& att);

    bool RunAttCtrl();
    bool RunAttRateCtrl();
    bool RunAccCtrl();

#if UAV_CONTROL_ACC
    AccController mAccController_X;
    AccController mAccController_Y;
#endif
#if UAV_CONTROL_ATT
    AttController mAttController_pitch;
    AttController mAttController_roll;
    AttController mAttController_yaw;
#endif

    AttRateController mAttRateController_pitch;
    AttRateController mAttRateController_roll;
    AttRateController mAttRateController_yaw;
    AccController mAccController_Z;

private:
    Controller(); // private constructor, singleton

    MotorCtrl& mMotorCtrl;

    FCVelDataType mVelSetpoint;
    FCAccDataType mAccSetpoint;
    FCAttType mAttSetpoint;
    FCVelDataType mCurVel;
    FCAttType mCurAtt;
    FCAttRateType mAttRateSetpoint;
    FCAttRateType mCurAttRate;
    int mPeriodMs;
};

#endif