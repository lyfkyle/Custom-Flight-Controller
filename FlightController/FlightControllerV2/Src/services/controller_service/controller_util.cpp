#include <arm_math.h>
#include <math.h>

#include <logging.h>

#include "UAV_Defines.h"


/*
 * Defines
 */

#define LOG_TAG ("Ctrler_Util")

#define CTRL_UTIL_DEBUG (0)
#if CTRL_UTIL_DEBUG
#define LOG(...) LOGI //TODO here
#else
#define LOG(...)
#endif

/*
 * Struct
 */

/*
 * Static
 */

static arm_matrix_instance_f32 sMatrix_RotYawInv; // rotation matrix of yaw angle
static arm_matrix_instance_f32 sVector_DesiredAtt;
static arm_matrix_instance_f32 sVector_Temp;
static float32_t sMatrixData_RotYawInv[9];
static float32_t sVectorData_DesiredAtt[3];
static float32_t sVectorData_Temp[3];

/*
 * Code
 */

bool ControllerUtil_Init()
{
   arm_mat_init_f32(&sMatrix_RotYawInv,3,3,(float32_t*)sMatrixData_RotYawInv);
   arm_mat_init_f32(&sVector_DesiredAtt,3,1,(float32_t*)sVectorData_DesiredAtt);
   arm_mat_init_f32(&sVector_Temp,3,1,(float32_t*)sVectorData_Temp);

   return true;
}

bool Controller_GetAttSetpointFromAccSetpoint(FCAttType& att, FCAccDataType& accSetpoint, float yawSetpoint)
{
   LOG("accSetpoint.x %f, .y %f .z %f\r\n", accSetpoint.x, accSetpoint.y, accSetpoint.z);
   // add gravity to z
   accSetpoint.z += UAV_G;

   // construct u2 = sVector_DesiredAtt
   float32_t sqredSum = accSetpoint.x * accSetpoint.x + accSetpoint.y * accSetpoint.y + accSetpoint.z * accSetpoint.z;
   float32_t norm = 0;
   arm_sqrt_f32(sqredSum, &norm);
   sVectorData_DesiredAtt[0] = accSetpoint.x / norm;
   sVectorData_DesiredAtt[1] = accSetpoint.y / norm;
   sVectorData_DesiredAtt[2] = accSetpoint.z / norm;

   // construct rotYaw inverse.
   float32_t cosYaw = arm_cos_f32(yawSetpoint);
   float32_t sinYaw = arm_sin_f32(yawSetpoint);
   sMatrixData_RotYawInv[0] = cosYaw;
   sMatrixData_RotYawInv[1] = sinYaw;
   sMatrixData_RotYawInv[2] = 0;
   sMatrixData_RotYawInv[3] = -1 * sinYaw;
   sMatrixData_RotYawInv[4] = cosYaw;
   sMatrixData_RotYawInv[5] = 0;
   sMatrixData_RotYawInv[6] = 0;
   sMatrixData_RotYawInv[7] = 0;
   sMatrixData_RotYawInv[8] = 1;

   // temp = RotYaw' * u2
   arm_mat_mult_f32(&sMatrix_RotYawInv, &sVector_DesiredAtt, &sVector_Temp);

   // pitch = atan(z_b_temp(1) / z_b_temp(3));
   // roll = asin(z_b_temp(2)* (-1));
   att.pitch = atan(sVectorData_Temp[0] / sVectorData_Temp[2]);
   att.roll = asin(sVectorData_Temp[1] * (-1));
   att.yaw = yawSetpoint;

   return true;
}

float GetHeightThrustFromAccSetpointZ(float accSetpoint_z)
{
    // open loop
    float res = UAV_PWM_HOVER_DUTYCYCLE + accSetpoint_z * ACC_SETPOINT_TO_MOTOR_THRUST_KP;
//    if (res < UAV_MOTOR_MIN_DUTYCYCLE) res = UAV_MOTOR_MIN_DUTYCYCLE;
//    if (res > UAV_MOTOR_MAX_DUTYCYCLE) res = UAV_MOTOR_MAX_DUTYCYCLE;
    return res;

}