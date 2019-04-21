#ifndef _UAV_DEFINES_
#define _UAV_DEFINES_

/*
 * This file defines global constants that all files need to use.
 */

/*
 * Defines
 */

#define UAV_G (9.81)
#define UAV_PI (3.1415926)

#define PID_ATT_KP (1.0f)
#define PID_ATT_KD (0.0f)
#define PID_ATT_KI (0.0f)
#define PID_ATT_RATE_KP (1.0f)
#define PID_ATT_RATE_KD (0.0f)
#define PID_ATT_RATE_KI (0.0f)
#define PID_VEL_KP (1.0f)
#define PID_VEL_KD (0.0f)
#define PID_VEL_KI (0.0f)

#define UAV_MAX_ACC_X (0.5)
#define UAV_MAX_ACC_Y (0.5)
#define UAV_MAX_ACC_Z (0.5)
#define UAV_MAX_VEL_X (0.5)
#define UAV_MAX_VEL_Y (0.5)
#define UAV_MAX_VEL_Z (0.5)

#define UAV_PWM_HOVER_DUTYCYCLE (40) // To Confirm
#define UAV_PWM_MIN_DUTYCYCLE (20) // to prevent propeller from stopping // To Confirm
#define UAV_PWM_MAX_DUTYCYCLE (80) // to prevent propeller from stopping // To Confirm
#define VEL_SETPOINT_TO_MOTOR_THRUST_KP (20)

#define CMD_ACC_MIN (-1.0f)
#define CMD_ACC_MAX (1.0f)

/*
 * Struct
 */

typedef struct {
   float yaw;
   float pitch;
   float roll;
} FCAttType;

typedef FCAttType FCAttRateType;

typedef struct {
   int motor1PWM;
   int motor2PWM;
   int motor3PWM;
   int motor4PWM;
} FCMotorPWMType;

typedef struct {
   float x;
   float y;
   float z;
} FCSensorDataType;

typedef FCSensorDataType FCAccDataType;
typedef FCSensorDataType FCVelDataType;

typedef struct {
   float q1;
   float q2;
   float q3;
   float q4;
} FCQuaternionType;

typedef struct {
    FCSensorDataType gyroData;
    FCSensorDataType accData;
    // FCSensorDataType magData;
} FCSensorMeasType;

typedef struct {
    FCAttType att;
    FCAttRateType attRate;
} FCStateType;

typedef struct {
    FCVelDataType desiredVel;
    float desiredYawRate;
} FCCmdType;

#endif