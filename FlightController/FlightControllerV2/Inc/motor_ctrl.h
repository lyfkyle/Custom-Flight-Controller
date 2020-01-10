#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

class MotorCtrl {
private:
    MotorCtrl(); // private constructor, singleton
public:
    static MotorCtrl& GetInstance();
    bool StopMotor();
    bool StartMotor();
    bool OutputMotor(float pitchThrust, float rollThrust, float yawThrust, float heightThrust);
};

#endif

