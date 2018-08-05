


class AttController
{
public:
   AttController();
   int GetDesiredMotorPWM(float attSetpoint, float curAtt);
   bool SetAttPID(float kp, float kd, float ki);
   bool SetKp(float kp);
   bool SetKd(float kd);
   bool SetKi(float ki);
private:
   PID mAttPID;
   float mAttSetpoint;
   float mCurAtt;
   float mOutput;
   float mKp;
   float mKd;
   float mKi;
}