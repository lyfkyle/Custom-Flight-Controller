#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

class MotorCtrl {
private:
    bool mToClampThrust;
    MotorCtrl(); // private constructor, singleton

public:
    static MotorCtrl& GetInstance();
    bool StopMotor();
    bool StartMotor();
    bool EnableThrustClamp(bool enable);
    bool OutputMotor(float pitchThrust, float rollThrust, float yawThrust, float heightThrust);
};

#endif

