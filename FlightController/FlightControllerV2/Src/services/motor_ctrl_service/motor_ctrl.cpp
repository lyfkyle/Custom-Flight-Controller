#include "logging.h"
#include "motor_ctrl.h"

#include "pwm.h"

/*
 * Define
 */

#define LOG_TAG ("MotorCtrl")

#define MOTOR_MIN_DUTYCYCLE (0)
#define MOTOR_MAX_DUTYCYCLE (100)

/*
 * Code
 */

MotorCtrl::MotorCtrl()
{

}

MotorCtrl& MotorCtrl::GetInstance()
{
    static MotorCtrl motorCtrl;
    return motorCtrl;
}

bool MotorCtrl::StopMotor()
{
    PWM_SetDutyCycle(PWM_CHANNEL_1, 0);
    PWM_SetDutyCycle(PWM_CHANNEL_2, 0);
    PWM_SetDutyCycle(PWM_CHANNEL_3, 0);
    PWM_SetDutyCycle(PWM_CHANNEL_4, 0);
    return true;
}

bool MotorCtrl::OutputMotor(float pitchThrust, float rollThrust, float yawThrust, float heightThrust)
{
    int motorPWM[4];
    motorPWM[0] = (int) (pitchThrust + rollThrust + yawThrust + heightThrust); // frontleft
    motorPWM[1] = (int) (pitchThrust - rollThrust - yawThrust + heightThrust); // frontright
    motorPWM[2] = (int) (-pitchThrust + rollThrust - yawThrust + heightThrust); // backleft
    motorPWM[3] = (int) (-pitchThrust - rollThrust + yawThrust + heightThrust); // backright

    for (int i = 0; i < 4; ++i) {
        if (motorPWM[i] < MOTOR_MIN_DUTYCYCLE) {
            motorPWM[i] = MOTOR_MIN_DUTYCYCLE;
        } else if (motorPWM[i] > MOTOR_MAX_DUTYCYCLE) {
            motorPWM[i] = MOTOR_MAX_DUTYCYCLE;
        }
    }

    LOGI("motorPWM: 1 %d , 2 %d, 3 %d, 4 %d\r\n", motorPWM[0], motorPWM[1], motorPWM[2], motorPWM[3]);

    PWM_SetDutyCycle(PWM_CHANNEL_1, (uint8_t)motorPWM[0]);
    PWM_SetDutyCycle(PWM_CHANNEL_2, (uint8_t)motorPWM[1]);
    PWM_SetDutyCycle(PWM_CHANNEL_3, (uint8_t)motorPWM[2]);
    PWM_SetDutyCycle(PWM_CHANNEL_4, (uint8_t)motorPWM[3]);

    return true;
}