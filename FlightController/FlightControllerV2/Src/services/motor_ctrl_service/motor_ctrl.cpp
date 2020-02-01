#include "logging.h"
#include "pwm.h"
#include "UAV_Defines.h"

#include "motor_ctrl.h"

/*
 * Define
 */

#define LOG_TAG ("MotorCtrl")



/*
 * Code
 */

MotorCtrl::MotorCtrl()
{
    mToClampThrust = true;
}

MotorCtrl& MotorCtrl::GetInstance()
{
    static MotorCtrl motorCtrl;
    return motorCtrl;
}

bool MotorCtrl::StopMotor()
{
    PWM_Stop();
    return true;
}

bool MotorCtrl::StartMotor()
{
    PWM_Start();
    return true;
}

bool MotorCtrl::EnableThrustClamp(bool enable)
{
    mToClampThrust = enable;
    return true;
}

bool MotorCtrl::OutputMotor(float pitchThrust, float rollThrust, float yawThrust, float heightThrust)
{
    int motorPWM[4];
    if (mToClampThrust) {
        if (heightThrust > 800) heightThrust = 800; // clamp height thrust.
    }
    motorPWM[0] = (int) (-pitchThrust + rollThrust + yawThrust + heightThrust); // frontleft
    motorPWM[1] = (int) (-pitchThrust - rollThrust - yawThrust + heightThrust); // frontright
    motorPWM[2] = (int) (pitchThrust + rollThrust - yawThrust + heightThrust); // backleft
    motorPWM[3] = (int) (pitchThrust - rollThrust + yawThrust + heightThrust); // backright

    for (int i = 0; i < 4; ++i) {
        if (motorPWM[i] < UAV_MOTOR_MIN_DUTYCYCLE) {
            motorPWM[i] = UAV_MOTOR_MIN_DUTYCYCLE;
        } else if (motorPWM[i] > UAV_MOTOR_MAX_DUTYCYCLE) {
            motorPWM[i] = UAV_MOTOR_MAX_DUTYCYCLE;
        }
    }

    LOGI("motorPWM: 1 %d , 2 %d, 3 %d, 4 %d\r\n", motorPWM[0], motorPWM[1], motorPWM[2], motorPWM[3]);

#if UAV_ENABLE_MOTORS
    PWM_SetDutyCycle(PWM_CHANNEL_1, motorPWM[0]);
    PWM_SetDutyCycle(PWM_CHANNEL_2, motorPWM[1]);
    PWM_SetDutyCycle(PWM_CHANNEL_3, motorPWM[2]);
    PWM_SetDutyCycle(PWM_CHANNEL_4, motorPWM[3]);
#endif
    return true;
}