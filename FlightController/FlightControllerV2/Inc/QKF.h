/*
 * This libray defines a quaternion kalman fileter for attitude estimation based on IMU measurement
 * For linear velocity and position estimation, perhaps another filter should be used.
 */

#ifndef LIB_QKF_h
#define LIB_QKF_h

#include <arm_math.h>
#include <stdint.h>

#include <UAV_Defines.h>

/*
 * Struct
 */

typedef enum {
   QKF_MEASUREMENT_GYRO,
   QKF_MEASUREMENT_ACC,
   QKF_MEASUREMENT_MAG,
} QKFMeasurementType;

typedef struct {
   QKFMeasurementType measType;
   float* pData;
} QKFMeasDataType;

#define QKF_FREQUENCY (200)

class QKF
{
private:
   /*for Kalman Filter*/
   arm_matrix_instance_f32 Matrix_X;//state
   arm_matrix_instance_f32 Matrix_X_prev; // prev_state
   arm_matrix_instance_f32 Matrix_A; //state_transition matrix
   arm_matrix_instance_f32 Matrix_Q; //process noise covariacne matrix
   arm_matrix_instance_f32 Matrix_P; //state noise covariacne
   arm_matrix_instance_f32 Matrix_P_prev; //previous/updated state noise covariance
   arm_matrix_instance_f32 Matrix_R_Acc; // accel sensor nosie covariance
   arm_matrix_instance_f32 Matrix_R_Mag; // mag sesnor noise covariance
   arm_matrix_instance_f32 Matrix_R_Gyro; //gyro sensor noise covariance
   arm_matrix_instance_f32 Matrix_R; //meas noise covariance
   arm_matrix_instance_f32 Matrix_S;
   arm_matrix_instance_f32 Matrix_H; //meas model
   arm_matrix_instance_f32 Matrix_H_T; //meas model transpose
   arm_matrix_instance_f32 Matrix_K; //kalman gain
   arm_matrix_instance_f32 Matrix_I; // identity matrix
   arm_matrix_instance_f32 Matrix_skewX;
   arm_matrix_instance_f32 Matrix_skewX_T;
   arm_matrix_instance_f32 Matrix_temp_12; //temporary matrix
   arm_matrix_instance_f32 Matrix_temp_16;
   arm_matrix_instance_f32 Matrix_temp_16_1;
   arm_matrix_instance_f32 Matrix_temp_32;
   arm_matrix_instance_f32 Matrix_temp_64;

   float32_t data_matrix_X[4];
   float32_t data_matrix_X_prev[4];
   float32_t data_matrix_A[16];
   float32_t data_matrix_Q[16];
   float32_t data_matrix_P[16];
   float32_t data_matrix_P_prev[16];
   float32_t data_matrix_R_Acc[9];
   float32_t data_matrix_R_Mag[9];
   float32_t data_matrix_R_Gyro[9];
   float32_t data_matrix_R[64];
   float32_t data_matrix_S[64];
   float32_t data_matrix_H[32];
   float32_t data_matrix_H_T[32];
   float32_t data_matrix_K[32];
   float32_t data_matrix_I[16];
   float32_t data_matrix_skewX[12];
   float32_t data_matrix_skewX_T[12];
   float32_t temp_vector[3];
   float32_t temp_matrix_9[9];
   float32_t temp_matrix_12[12];
   float32_t temp_matrix_16[16];
   float32_t temp_matrix_16_1[16];
   float32_t temp_matrix_32[32];
   float32_t temp_matrix_64[64];
   float32_t temp_value;
   float32_t X_norm;
   float32_t magnitude;
   float32_t dt;
   uint8_t firstRun; //first run flag;

   float32_t gravity[3];
   float32_t magConst[3];
   void InitMatrix();

public:
   QKF();
   bool SetGravityVector(float* pGravity);
   bool SetMagConstVector(float* pMagConst);
   bool SetGyroNoise(FCSensorDataType* pNoise);
   bool SetMagNoise(FCSensorDataType* pNoise);
   bool SetAccelNoise(FCSensorDataType* pNoise);

   bool PredictState(FCSensorDataType* pGyroData);
   bool UpdateState(FCSensorDataType* pAccData, FCSensorDataType* pMagData);
   bool GetState(FCQuaternionType* pQuaternion);
};
#endif