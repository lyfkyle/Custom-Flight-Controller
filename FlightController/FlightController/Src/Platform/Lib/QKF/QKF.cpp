#include "QKF.h"

#include "logging.h"

#define LOG_TAG ("QKF")

#define QKF_DEBUG (1)
#if QKF_DEBUG
#define LOG(...) LOGI(__VA_ARGS__)
#else
#define LOG(...)
#endif

// TODO get noise from IMU module

/*
* Constants
*/

#define GYRO_NOISE_DEFAULT 0.1
#define ACC_NOISE_DEFAULT 0.008
#define MAG_NOISE_DEFAULT 0.05

/*
* Code
*/
#if QKF_DEBUG
static void DebugMatrix(const float32_t* pData, const int dataSize)
{
    PRINT("Matrix Value :");
    for (int i = 0; i < dataSize; ++i) {
        PRINT(" %.2f", pData[i]);
    }
    PRINT("\r\n");
}
#endif
/*
* Given a squre matrix, init it to become an identity matrix
*/
static void InitIdentityMatrix(float32_t* pData, const int rowNum)
{
    memset(pData, 0, rowNum * rowNum * sizeof(float));
    for (int i = 0; i < rowNum * rowNum; i += rowNum + 1) {
        pData[i] = 1;
    }
}

void QKF::InitMatrix()
{
    arm_mat_init_f32(&Matrix_X,4,1,(float32_t*)data_matrix_X);
    arm_mat_init_f32(&Matrix_X_prev,4,1,(float32_t*)data_matrix_X_prev);
    arm_mat_init_f32(&Matrix_A,4,4,(float32_t*)data_matrix_A);
    arm_mat_init_f32(&Matrix_Q,4,4,(float32_t*)data_matrix_Q);
    arm_mat_init_f32(&Matrix_P,4,4,(float32_t*)data_matrix_P);
    arm_mat_init_f32(&Matrix_P_prev,4,4,(float32_t*)data_matrix_P_prev);
    arm_mat_init_f32(&Matrix_R_Acc,3,3,(float32_t*)data_matrix_R_Acc);
    arm_mat_init_f32(&Matrix_R_Mag,3,3,(float32_t*)data_matrix_R_Mag);
    arm_mat_init_f32(&Matrix_R_Gyro,3,3,(float32_t*)data_matrix_R_Gyro);
    arm_mat_init_f32(&Matrix_R,8,8,(float32_t*)data_matrix_R);
    arm_mat_init_f32(&Matrix_S,8,8,(float32_t*)data_matrix_S);
    arm_mat_init_f32(&Matrix_H,8,4,(float32_t*)data_matrix_H);
    arm_mat_init_f32(&Matrix_H_T,4,8,(float32_t*)data_matrix_H_T);
    arm_mat_init_f32(&Matrix_K,4,8,(float32_t*)data_matrix_K);
    arm_mat_init_f32(&Matrix_I,4,4,(float32_t*)data_matrix_I);
    arm_mat_init_f32(&Matrix_skewX,4,3,(float32_t*)data_matrix_skewX);
    arm_mat_init_f32(&Matrix_skewX_T,3,4,(float32_t*)data_matrix_skewX_T);
    arm_mat_init_f32(&Matrix_temp_12,4,3,(float32_t*)temp_matrix_12);
    // arm_mat_init_f32(&Matrix_temp_12_1,4,3,(float32_t*)temp_matrix_12_1);
    arm_mat_init_f32(&Matrix_temp_16,4,4,(float32_t*)temp_matrix_16);
    arm_mat_init_f32(&Matrix_temp_16_1,4,4,(float32_t*)temp_matrix_16_1);
    // arm_mat_init_f32(&Matrix_temp_16_T,4,4,(float32_t*)temp_matrix_16_T);
    arm_mat_init_f32(&Matrix_temp_32,8,4,(float32_t*)temp_matrix_32); //8x4 matrix
    // arm_mat_init_f32(&Matrix_temp_32_T,4,8,(float32_t*)temp_matrix_32_T); //4x8 matrix
    arm_mat_init_f32(&Matrix_temp_64,8,8,(float32_t*)temp_matrix_64);
}

QKF::QKF()
{
    InitIdentityMatrix(data_matrix_I, 4);
    memset(data_matrix_R, 0, 64 * sizeof(float));
    memset(data_matrix_R_Gyro, 0, 9 * sizeof(float));
    data_matrix_R_Gyro[0] = GYRO_NOISE_DEFAULT;
    data_matrix_R_Gyro[4] = GYRO_NOISE_DEFAULT;
    data_matrix_R_Gyro[8] = GYRO_NOISE_DEFAULT;
    memset(data_matrix_R_Mag, 0, 9 * sizeof(float));
    data_matrix_R_Mag[0] = MAG_NOISE_DEFAULT;
    data_matrix_R_Mag[4] = MAG_NOISE_DEFAULT;
    data_matrix_R_Mag[8] = MAG_NOISE_DEFAULT;
    memset(data_matrix_R_Acc, 0, 9 * sizeof(float));
    data_matrix_R_Acc[0] = ACC_NOISE_DEFAULT;
    data_matrix_R_Acc[4] = ACC_NOISE_DEFAULT;
    data_matrix_R_Acc[8] = ACC_NOISE_DEFAULT;
    InitIdentityMatrix(data_matrix_P_prev, 4);
    memset(data_matrix_X_prev, 0, 4 * sizeof(float));
    data_matrix_X_prev[0] = 1;
    memset(data_matrix_X, 0, 4 * sizeof(float));
    data_matrix_X[0] = 1;
    memset(temp_matrix_16, 0, 16 * sizeof(float));
    memset(temp_matrix_16_1, 0, 16 * sizeof(float));

    // gravity
    memset(gravity, 0, 3 * sizeof(float));
    gravity[2] = UAV_G;// by default
    // magConst
    memset(magConst, 0, 3 * sizeof(float)); // by default all 0

    dt = 1.0f / QKF_FREQUENCY;
    firstRun = 1;

    InitMatrix();
}

bool QKF::SetGyroNoise(FCSensorDataType* pNoise)
{
    if (!pNoise) return false;
    data_matrix_R_Gyro[0] = pNoise->x;
    data_matrix_R_Gyro[4] = pNoise->y;
    data_matrix_R_Gyro[8] = pNoise->z;
    return true;
}

bool QKF::SetMagNoise(FCSensorDataType* pNoise)
{
    if (!pNoise) return false;
    data_matrix_R_Mag[0] = pNoise->x;
    data_matrix_R_Mag[4] = pNoise->y;
    data_matrix_R_Mag[8] = pNoise->z;
    return true;
}

bool QKF::SetAccelNoise(FCSensorDataType* pNoise)
{
    if (!pNoise) return false;
    data_matrix_R_Acc[0] = pNoise->x;
    data_matrix_R_Acc[4] = pNoise->y;
    data_matrix_R_Acc[8] = pNoise->z;
    return true;
}

bool QKF::SetGravityVector(float* pGravity)
{
    if (!pGravity) return false;
    gravity[0] = pGravity[0];
    gravity[1] = pGravity[1];
    gravity[2] = pGravity[2];
    return true;
}

bool QKF::SetMagConstVector(float* pMagConst)
{
    if (!pMagConst) return false;
    magConst[0] = pMagConst[0];
    magConst[1] = pMagConst[1];
    magConst[2] = pMagConst[2];
    return true;
}

bool QKF::PredictState(FCSensorDataType* pGyroData)
{
    // Input validity check
    if (pGyroData == NULL) {
        LOGI("pGyroData == NULL, no need prediction", __func__);
        return true;
    }
    float Gxyz[3];
    Gxyz[0] = pGyroData->x * UAV_PI / 180;
    Gxyz[1] = pGyroData->y * UAV_PI / 180;
    Gxyz[2] = pGyroData->z * UAV_PI / 180;

    int i;

    /*step 1-> calculate A ->state transition matrix*/
    /*temp1 = [gyro(1)*pi/180;gyro(2)*pi/180;gyro(3)*pi/180];*/
    /*magnitude = norm(temp1);*/
    temp_value = Gxyz[0]*Gxyz[0] + Gxyz[1]*Gxyz[1] + Gxyz[2]*Gxyz[2];
    arm_sqrt_f32(temp_value, &magnitude);

    // LOG("magnitude = %f\r\n", magnitude);

    /*  if (magnitude<1e-4)
    magnitude = 0;
    temp1 = zeros(3,1);
    else
    temp1 = temp1/magnitude*sin(magnitude*dt/2);*/
    if (magnitude<0.0001) {
        magnitude = 0;
        for(i = 0; i < 3; ++i){
            Gxyz[i] = 0.0f;
        }
    } else {
        float32_t tmp = arm_sin_f32(magnitude * dt / 2);
        for(int i = 0; i < 3; ++i){
            temp_vector[i] = Gxyz[i] / magnitude * tmp;
        }
    }
    /*a = cos(magnitude/2*dt);*/
    temp_value = arm_cos_f32(magnitude*dt/2);

    /*skew = skewSymmetric(a,temp1);*/
    /*function [Matrix]=skewSymmetric(a,X)
    Matrix = [a X(3)*(-1) X(2);X(3) a X(1)*(-1);X(2)*(-1) X(1) a];*/
    /*value of A*/
    /*  A_top = [a,(temp1')*(-1)];
    A_btm = [temp1,skew];
    A = [A_top;A_btm];
    */
    data_matrix_A[0] = temp_value;
    data_matrix_A[1] = temp_vector[0]*(-1);
    data_matrix_A[2] = temp_vector[1]*(-1);
    data_matrix_A[3] = temp_vector[2]*(-1);
    data_matrix_A[4] = temp_vector[0];
    data_matrix_A[8] = temp_vector[1];
    data_matrix_A[12] = temp_vector[2];
    data_matrix_A[5] = temp_value;
    data_matrix_A[6] = temp_vector[2]*(-1);
    data_matrix_A[7] = temp_vector[1];
    data_matrix_A[9] = temp_vector[2];
    data_matrix_A[10] = temp_value;
    data_matrix_A[11] = temp_vector[0]*(-1);
    data_matrix_A[13] = temp_vector[1]*(-1);
    data_matrix_A[14] = temp_vector[0];
    data_matrix_A[15] = temp_value;

#if QKF_DEBUG
    // DebugMatrix(data_matrix_A, 16);
#endif

    /*Step 2 ---------------->  predict X */
    /*X = A*X_prev;*/
    arm_mat_mult_f32(&Matrix_A, &Matrix_X_prev, &Matrix_X);

    /*Step 3 ----------------> calculate Q*/
    /*skewQ = skewSymmetric(X(1),X(2:4));*/
    /*skewX = [X(2)*(-1) X(3)*(-1) X(4)*(-1);skewQ];*/
    data_matrix_skewX[0] = data_matrix_X_prev[1]*(-1);
    data_matrix_skewX[1] = data_matrix_X_prev[2]*(-1);
    data_matrix_skewX[2] = data_matrix_X_prev[3]*(-1);
    data_matrix_skewX[3] = data_matrix_X_prev[0];
    data_matrix_skewX[4] = data_matrix_X_prev[3]*(-1);
    data_matrix_skewX[5] = data_matrix_X_prev[2];
    data_matrix_skewX[6] = data_matrix_X_prev[3];
    data_matrix_skewX[7] = data_matrix_X_prev[0];
    data_matrix_skewX[8] = data_matrix_X_prev[1]*(-1);
    data_matrix_skewX[9] = data_matrix_X_prev[2]*(-1);
    data_matrix_skewX[10] = data_matrix_X_prev[1];
    data_matrix_skewX[11] = data_matrix_X_prev[0];

    /*Q = dt*dt/4*skewX*rg*(skewX');*/
    temp_value = dt*dt/4;
    arm_mat_mult_f32(&Matrix_skewX, &Matrix_R_Gyro, &Matrix_temp_12);
    arm_mat_trans_f32(&Matrix_skewX, &Matrix_skewX_T);
    arm_mat_mult_f32(&Matrix_temp_12, &Matrix_skewX_T, &Matrix_Q);
    for(i = 0; i < 16; ++i){
        data_matrix_Q[i] *= temp_value;
    }

    /*Step4 ----------------> predict P */
    /*P = A*P_prev*A'+ Q;*/
    if(firstRun){
        data_matrix_P[0] = 1;
        data_matrix_P[5] = 1;
        data_matrix_P[10] = 1;
        data_matrix_P[15] = 1;
    }else{
        arm_mat_trans_f32(&Matrix_A, &Matrix_temp_16);
        arm_mat_mult_f32(&Matrix_A, &Matrix_P_prev, &Matrix_temp_16_1);
        arm_mat_mult_f32(&Matrix_temp_16_1, &Matrix_temp_16, &Matrix_P);
    }
    for(i = 0; i < 16; ++i){
        data_matrix_P_prev[i] = data_matrix_P[i] + data_matrix_Q[i];
    }

    // copy to prev state
    for (i = 0; i < 4; ++i) {
        data_matrix_X_prev[i] = data_matrix_X[i];
    }

    return true;
}

bool QKF::UpdateState(FCSensorDataType* pAccData, FCSensorDataType* pMagData)
{
    // Input validity check
    if (pAccData == NULL) {
        LOGI("pAccData == NULL", __func__);
        return false;
    }
    if (pMagData == NULL) {
        LOGI("pMagData == NULL", __func__);
        return false;
    }
    // float* Axyz = (float*) pAccData; //TODO
    float Axyz[3];
    Axyz[0] = pAccData->x;
    Axyz[1] = pAccData->y;
    Axyz[2] = pAccData->z;
    // float* Mxyz = (float*) pMagData; //TODO
    float Mxyz[3];
    Mxyz[0] = pMagData->x;
    Mxyz[1] = pMagData->y;
    Mxyz[2] = pMagData->z;

    int i;

    /*Step 5 --------------> Calculate H*/
    /*tmp = Za-G;*/
    for(i = 0; i < 3; ++i){
        temp_vector[i] = Axyz[i] - gravity[i];
    }

    /*Hleft = [0;tmp]; + Hright = [(-1)*tmp';skewH*(-1)];*/
    data_matrix_H[0] = 0;
    data_matrix_H[1] = temp_vector[0]*(-1);
    data_matrix_H[2] = temp_vector[1]*(-1);
    data_matrix_H[3] = temp_vector[2]*(-1);
    data_matrix_H[4] = temp_vector[0];
    data_matrix_H[8] = temp_vector[1];
    data_matrix_H[12] = temp_vector[2];

    /*tmp1 = Za+G;*/
    for(i = 0; i < 3; ++i){
        temp_vector[i] = Axyz[i]+gravity[i];
    }

    /*skewH = skewSymmetric(0,tmp1);*/
    /*Hright = [(-1)*tmp';skewH*(-1)];*/
    /*Htop = [Hleft,Hright];*/
    data_matrix_H[5] = 0;
    data_matrix_H[6] = temp_vector[2];
    data_matrix_H[7] = temp_vector[1] * (-1);
    data_matrix_H[9] = temp_vector[2] * (-1);
    data_matrix_H[10] = 0;
    data_matrix_H[11] = temp_vector[0];
    data_matrix_H[13] = temp_vector[1];
    data_matrix_H[14] = temp_vector[0] * (-1);
    data_matrix_H[15] = 0;

    /*tmp = Zm-M;*/
    for(i = 0; i < 3; ++i){
        temp_vector[i] = Mxyz[i] - magConst[i];
    }

    /*Hleft = [0;tmp]; + Hright = [(-1)*tmp';skewH*(-1)];*/
    data_matrix_H[16] = 0;
    data_matrix_H[17] = temp_vector[0]*(-1);
    data_matrix_H[18] = temp_vector[1]*(-1);
    data_matrix_H[19] = temp_vector[2]*(-1);
    data_matrix_H[20] = temp_vector[0];
    data_matrix_H[24] = temp_vector[1];
    data_matrix_H[28] = temp_vector[2];

    /*tmp1 = Zm+M;*/
    for(i = 0; i < 3; ++i){
        temp_vector[i] = Mxyz[i] + magConst[i];
    }

    /*skewH = skewSymmetric(0,tmp1);*/
    /*Hright = [(-1)*tmp';skewH*(-1)];
    Hbtm = [Hleft,Hright];
    H = [Htop;Hbtm];*/
    data_matrix_H[21] = 0;
    data_matrix_H[22] = temp_vector[2];
    data_matrix_H[23] = temp_vector[1]*(-1);
    data_matrix_H[25] = temp_vector[2]*(-1);
    data_matrix_H[26] = 0;
    data_matrix_H[27] = temp_vector[0];
    data_matrix_H[29] = temp_vector[1];
    data_matrix_H[30] = temp_vector[0]*(-1);
    data_matrix_H[31] = 0;

    /*Step 6 --------------> Calculate R*/
    /*Ra = 0.25*skewX*ra*(skewX');*/
    data_matrix_skewX[0] = data_matrix_X_prev[1]*(-1);
    data_matrix_skewX[1] = data_matrix_X_prev[2]*(-1);
    data_matrix_skewX[2] = data_matrix_X_prev[3]*(-1);
    data_matrix_skewX[3] = data_matrix_X_prev[0];
    data_matrix_skewX[4] = data_matrix_X_prev[3]*(-1);
    data_matrix_skewX[5] = data_matrix_X_prev[2];
    data_matrix_skewX[6] = data_matrix_X_prev[3];
    data_matrix_skewX[7] = data_matrix_X_prev[0];
    data_matrix_skewX[8] = data_matrix_X_prev[1]*(-1);
    data_matrix_skewX[9] = data_matrix_X_prev[2]*(-1);
    data_matrix_skewX[10] = data_matrix_X_prev[1];
    data_matrix_skewX[11] = data_matrix_X_prev[0];
    arm_mat_mult_f32(&Matrix_skewX, &Matrix_R_Acc, &Matrix_temp_12);
    arm_mat_trans_f32(&Matrix_skewX, &Matrix_skewX_T);
    arm_mat_mult_f32(&Matrix_temp_12, &Matrix_skewX_T, &Matrix_temp_16);
    for(i = 0; i < 16; ++i){
        temp_matrix_16[i] *= 0.25;
    }

    /*Rm = 0.25*skewX*rm*(skewX');*/
    arm_mat_mult_f32(&Matrix_skewX, &Matrix_R_Mag, &Matrix_temp_12);
    arm_mat_mult_f32(&Matrix_temp_12, &Matrix_skewX_T, &Matrix_temp_16_1);
    for(i = 0; i < 16; i++){
        temp_matrix_16_1[i] *= 0.25;
    }

    /*Rtop = [Ra,zeros(4)];
    Rbtm = [zeros(4),Rm];
    R = [Rtop;Rbtm];*/
    data_matrix_R[0] = temp_matrix_16[0];
    data_matrix_R[1] = temp_matrix_16[1];
    data_matrix_R[2] = temp_matrix_16[2];
    data_matrix_R[3] = temp_matrix_16[3];
    data_matrix_R[8] = temp_matrix_16[4];
    data_matrix_R[9] = temp_matrix_16[5];
    data_matrix_R[10] = temp_matrix_16[6];
    data_matrix_R[11] = temp_matrix_16[7];
    data_matrix_R[16] = temp_matrix_16[8];
    data_matrix_R[17] = temp_matrix_16[9];
    data_matrix_R[18] = temp_matrix_16[10];
    data_matrix_R[19] = temp_matrix_16[11];
    data_matrix_R[24] = temp_matrix_16[12];
    data_matrix_R[25] = temp_matrix_16[13];
    data_matrix_R[26] = temp_matrix_16[14];
    data_matrix_R[27] = temp_matrix_16[15];
    data_matrix_R[36] = temp_matrix_16_1[0];
    data_matrix_R[37] = temp_matrix_16_1[1];
    data_matrix_R[38] = temp_matrix_16_1[2];
    data_matrix_R[39] = temp_matrix_16_1[3];
    data_matrix_R[44] = temp_matrix_16_1[4];
    data_matrix_R[45] = temp_matrix_16_1[5];
    data_matrix_R[46] = temp_matrix_16_1[6];
    data_matrix_R[47] = temp_matrix_16_1[7];
    data_matrix_R[52] = temp_matrix_16_1[8];
    data_matrix_R[53] = temp_matrix_16_1[9];
    data_matrix_R[54] = temp_matrix_16_1[10];
    data_matrix_R[55] = temp_matrix_16_1[11];
    data_matrix_R[60] = temp_matrix_16_1[12];
    data_matrix_R[61] = temp_matrix_16_1[13];
    data_matrix_R[62] = temp_matrix_16_1[14];
    data_matrix_R[63] = temp_matrix_16_1[15];
    //the other values should be 0 as initialized.

    /*Step 7 -------------->  update */
    /*S = H*P*H'+R;*/
    arm_mat_trans_f32(&Matrix_H, &Matrix_H_T);
    arm_mat_mult_f32(&Matrix_H, &Matrix_P_prev, &Matrix_temp_32);
    arm_mat_mult_f32(&Matrix_temp_32, &Matrix_H_T, &Matrix_S);
    if (firstRun){
        data_matrix_S[3] = 0;
        data_matrix_S[10] = 0;
        data_matrix_S[17] = 0;
        data_matrix_S[24] = 0;
        data_matrix_S[39] = 0;
        data_matrix_S[46] = 0;
        data_matrix_S[53] = 0;
        data_matrix_S[60] = 0;
    }
    for (i = 0; i < 64; ++i) {
        data_matrix_S[i] += data_matrix_R[i];
    }

    /*K = (P*H')/S;*/
    arm_mat_inverse_f32(&Matrix_S, &Matrix_temp_64);
    arm_mat_mult_f32(&Matrix_P_prev, &Matrix_H_T, &Matrix_temp_32);
    arm_mat_mult_f32(&Matrix_temp_32, &Matrix_temp_64, &Matrix_K);

    /*X_updated = (I-K*H)*X;*/
    arm_mat_mult_f32(&Matrix_K, &Matrix_H, &Matrix_temp_16);
    for (i = 0; i < 16; ++i) {
        temp_matrix_16[i] = data_matrix_I[i] - temp_matrix_16[i];
    }
    arm_mat_mult_f32(&Matrix_temp_16, &Matrix_X_prev, &Matrix_X);

    /*X_updated = X_updated/norm(X_updated);*/
    temp_value = data_matrix_X[0]*data_matrix_X[0]
        + data_matrix_X[1]*data_matrix_X[1]
            + data_matrix_X[2]*data_matrix_X[2]
                + data_matrix_X[3]*data_matrix_X[3];
    arm_sqrt_f32(temp_value, &X_norm);
    for (i = 0; i < 4; i++) {
        data_matrix_X[i] = data_matrix_X[i] / X_norm;
    }

    /*P_updated = (I-K*H)*P;*/
    arm_mat_mult_f32(&Matrix_temp_16, &Matrix_P_prev, &Matrix_P);

    /*arm_mat_trans_f32(&Matrix_temp_16_1,&Matrix_temp_16_T);
    arm_mat_mult_f32(&Matrix_temp_16_1,&Matrix_P,&Matrix_temp_16);
    arm_mat_mult_f32(&Matrix_temp_16,&Matrix_temp_16_T,&Matrix_temp_16_1);
    arm_mat_trans_f32(&Matrix_K,&Matrix_temp_32);
    arm_mat_mult_f32(&Matrix_K,&Matrix_R,&Matrix_temp_32_T);
    arm_mat_mult_f32(&Matrix_temp_32_T,&Matrix_temp_32,&Matrix_temp_16);
    arm_mat_add_f32(&Matrix_temp_16_1,&Matrix_temp_16,&Matrix_P_prev);*/

    if(firstRun){
        firstRun = 0;
    }

    // copy to CurState
    for (int i = 0; i < 4; ++i) {
        data_matrix_X[i] = data_matrix_X_prev[i];
    }
    for(i = 0; i < 16; ++i){
        data_matrix_P_prev[i] = data_matrix_P[i];
    }

    return true;
}

bool QKF::GetState(FCQuaternionType* pQuaternion)
{
    if (pQuaternion == NULL) {
        LOGE("%s, input invalid", __func__);
        return false;
    }
    pQuaternion->q1 = data_matrix_X_prev[0];
    pQuaternion->q2 = data_matrix_X_prev[1];
    pQuaternion->q3 = data_matrix_X_prev[2];
    pQuaternion->q4 = data_matrix_X_prev[3];
    return true;
}
