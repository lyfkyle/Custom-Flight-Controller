#ifndef _UAV_DEFINES_
#define _UAV_DEFINES_

/*
 * This file defines global constants that all files need to use.
 */

/*
 * Defines
 */

#define UAV_Debug (0)
#define UAV_ENABLE_MOTORS (1)

// toggle whether the RC value controls attitude or acceleration
#define UAV_CMD_ATT_RATE (0) // in this mode, uesr directly control UAV's attitude rate
#define UAV_CMD_ATT (1) // in this mode, user directly controls UAV's attitude
#define UAV_CMD_ACC (0) // in this mode, user controls UAV's movement in xyz direction according to map coordinate. TODO. this is not supported.

#if UAV_CND_ACC
#define UAV_CONTROL_ACC (1)
#define UAV_CONTROL_ATT (1)
#elif UAV_CMD_ATT
#define UAV_CONTROL_ACC (0)
#define UAV_CONTROL_ATT (1)
#endif

/*
 * Constants
 */

#define UAV_G (9.7760)
#define UAV_PI (3.1415926)
#define UAV_RADIANS_TO_DEGREE (57.2957805)
#define UAV_DEGREE_TO_RADIAN (0.01745329)

/* SensorBias */
#define DEFAULT_ACC_BIAS_X (0.024)
#define DEFAULT_ACC_BIAS_Y (0.01)
#define DEFAULT_ACC_BIAS_Z (0.045)

/* PID */
#define PID_ATT_KP_PITCH (11.0f)
#define PID_ATT_KD_PITCH (0.0f)
#define PID_ATT_KI_PITCH (0.1f)
#define PID_ATT_KP_ROLL (11.0f)
#define PID_ATT_KD_ROLL (0.0f)
#define PID_ATT_KI_ROLL (0.1f)
#define PID_ATT_RATE_KP_PITCH (0.215f)
#define PID_ATT_RATE_KD_PITCH (0.0004f)
#define PID_ATT_RATE_KI_PITCH (0.001f)
#define PID_ATT_RATE_KP_ROLL (0.215f)
#define PID_ATT_RATE_KD_ROLL (0.0004f)
#define PID_ATT_RATE_KI_ROLL (0.001f)
#define PID_ATT_RATE_KP_YAW (0.21f)
#define PID_ATT_RATE_KD_YAW (0.0f)
#define PID_ATT_RATE_KI_YAW (0.001f)
#define PID_ACC_KP_Z (1.0f)
#define PID_ACC_KD_Z (0.0f)
#define PID_ACC_KI_Z (0.0f)
#define ACC_SETPOINT_TO_MOTOR_THRUST_KP (400)

#define UAV_PWM_HOVER_DUTYCYCLE (400) // To Confirm
//#define UAV_PWM_MIN_DUTYCYCLE (0) // to prevent propeller from stopping // To Confirm
//#define UAV_PWM_MAX_DUTYCYCLE (100) // to prevent propeller from stopping // To Confirm

/* Limits */

#define ACC_PID_OUT_MIN (-10) // TODO not tested.
#define ACC_PID_OUT_MAX (10) // TODO not tested
#define ATT_PID_OUT_MIN (-200.0f)
#define ATT_PID_OUT_MAX (200.0f)
#define ATT_RATE_PID_OUT_MIN (-200)
#define ATT_RATE_PID_OUT_MAX (200)

// pwm output to motor. we want 1000 steps between 0ms to 1ms pulse width
#define UAV_MOTOR_MIN_DUTYCYCLE (0)
#define UAV_MOTOR_MAX_DUTYCYCLE (1000)

// unit m/s*2
#define CMD_ACC_MIN (-1.0f)
#define CMD_ACC_MAX (1.0f)

// unit radian
#define CMD_PITCH_MIN (-20.0f)
#define CMD_PITCH_MAX (20.0f)
#define CMD_ROLL_MIN (-20.0f)
#define CMD_ROLL_MAX (20.0f)

// unit dps
#define CMD_ROLL_RATE_MIN (-90.0f)
#define CMD_ROLL_RATE_MAX (90.0f)
#define CMD_PITCH_RATE_MIN (-90.0f)
#define CMD_PITCH_RATE_MAX (90.0f)
#define CMD_YAW_RATE_MIN (-90.0f)
#define CMD_YAW_RATE_MAX (90.0f)

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

#if UAV_CMD_ATT
typedef struct {
    float desiredAccZ;
    float desiredPitch;
    float desiredRoll;
    float desiredYawRate;
    bool toTunePID;
} FCCmdType;
#endif

#if UAV_CMD_ATT_RATE
typedef struct {
    float desiredAccZ;
    FCAttRateType desiredAttRate;
    bool toTunePID;
} FCCmdType;
#endif

#if UAV_CMD_ACC
typedef struct {
    FCAccDataType desiredAcc;
    float desiredYawRate;
    bool toTunePID;
} FCCmdType;
#endif

#endif