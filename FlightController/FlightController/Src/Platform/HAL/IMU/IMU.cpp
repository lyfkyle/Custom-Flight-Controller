#include <stdlib.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#include "IMU.h"

#include "logging.h"

/*
* Defines
*/

#define LOG_TAG ("IMU")

#define DEBUG_IMU (0)
#if DEBUG_IMU
#define LOG(...)  LOGI(__VA_ARGS__)
#else
#define LOG(...)
#endif

/*
* Constants
*/

// GPIO interrupt definition
#define MPU9250_ID                   0x73
#if USE_INTERRUPT
#define MPU9250_Interrupt_Pin        GPIO_PIN_12
#define MPU9250_Interrupt_GPIO_Port  GPIOF
#endif

/*
 * Code
 */
IMU::IMU()
{
    /*Configure GPIO pin : MPU9250_Interrupt_Pin */
#if USE_INTERRUPT
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = MPU9250_Interrupt_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MPU9250_Interrupt_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
#endif

    mGyroAccDataRdyFlag = true;
    mSensorBiasCalibrateFlag = false;
    mMagCalibrateFlag = false;
    mReadyToStart = false;
    mMagEnabled = false;
#if USE_INTERRUPT
    mDataReadyCb = NULL;
#endif

    for (int i = 0; i < 3; ++i) {
        magConst[i] = 0.0f;
        gravity[i] = 0.0f;
        gyroBias[i] = 0.0f;
        mMagOffset[i] = 0.0f;
        mMagScale[i] = 1.0f;
    }
}

IMU& IMU::GetInstance()
{
    static IMU imu;
    return imu;
}

/*------------------------------------------------*
 * Interrupt Handler
 *------------------------------------------------*/

#if USE_INTERRUPT
inline void IMU::SetGyroAccDataReadyFlg()
{
    mGyroAccDataRdyFlag = true;
}

void IMU::OnGyroAccDataReady()
{

    // LOG("%s\r\n", __func__);
    SetGyroAccDataReadyFlg();

    if (mDataReadyCb != NULL) {
        mDataReadyCb();
    }

}
#endif

/*------------------------------------------------*
 * Public API
 *------------------------------------------------*/
bool IMU::Init()
{
    LOG("%s\r\n", __func__);
    // sanity check whoami register
    uint8_t deviceID = mIMU.getDeviceID();
    if (deviceID == MPU9250_ID) {
        mIMU.Init(); //initialzie
        mReadyToStart = true;
    } else {
        LOGE("mIMU ID wrong, ID = %x\r\n", deviceID);
        return false;
    }

    return true;
}

bool IMU::Start()
{
    LOG("%s\r\n", __func__);
    if (mReadyToStart) {
#if USE_INTERRUPT
        mIMU.enableInterrupt();
#endif
    } else {
        LOGE("failed to start IMU \r\n");
        return false;
    }

    return true;
}

bool IMU::EnableMag(bool enable)
{
    mMagEnabled = enable;
    return true;
}

#if USE_INTERRUPT
bool IMU::SetDataReadyCb(DataReadyCb cb)
{
    if (cb == NULL) {
        return false;
    }

    mDataReadyCb = cb;
    return true;
}
#endif

void IMU::GetGravityVector(float* pGravity)
{
    for (int i = 0; i < 3; i++) {
        pGravity[i] = gravity[i];
    }
}

void IMU::GetMagConstVector(float* pMagConst)
{
    for (int i = 0; i < 3; i++) {
        pMagConst[i] = magConst[i];
    }
}

void IMU::GetGyroData(FCSensorDataType* pGyroData)
{
    int16_t gx, gy, gz;
    mIMU.getRotation(&gx, &gy, &gz);
    float Gxyz[3];
    Gxyz[0] = (float) gx * mIMU.mGyroSensitivity;
    Gxyz[1] = (float) gy * mIMU.mGyroSensitivity;
    Gxyz[2] = (float) gz * mIMU.mGyroSensitivity;
    Gxyz[0] = Gxyz[0] - gyroBias[0];
    Gxyz[1] = Gxyz[1] - gyroBias[1];
    Gxyz[2] = Gxyz[2] - gyroBias[2];
    //High Pass Filter -> remove all values that are less than 0.05dps.
    for (int i = 0; i < 3; ++i){
        if (-1 < Gxyz[i] && Gxyz[i] < 1){
            Gxyz[i]=0.0f;
        }
    }

    pGyroData->x = Gxyz[0];
    pGyroData->y = Gxyz[1];
    pGyroData->z = Gxyz[2];
}

void IMU::GetRawGyroData(FCSensorDataType* pGyroData)
{
    int16_t gx, gy, gz;
    mIMU.getRotation(&gx, &gy, &gz);
    pGyroData->x = (float) gx * mIMU.mGyroSensitivity;
    pGyroData->y = (float) gy * mIMU.mGyroSensitivity;
    pGyroData->z = (float) gz * mIMU.mGyroSensitivity;
}

void IMU::GetAccelData(FCSensorDataType* pAccData)
{
    int16_t ax, ay, az;
    mIMU.getAcceleration(&ax,&ay,&az);
    pAccData->x = (float) ax * mIMU.mAccSensitivity * 0.001f;
    pAccData->y = (float) ay * mIMU.mAccSensitivity * 0.001f;
    pAccData->z = (float) az * mIMU.mAccSensitivity * 0.001f;
}

bool IMU::GetCompassData(FCSensorDataType* pMagData)
{
    if (!mMagEnabled) {
        LOGE("Mag is not enabled\r\n");
        return false;
    }

    uint8_t dataReady = mIMU.getCompassDataReady();
    if (dataReady == 1){
        int16_t mx, my, mz;
        float Mxyz[3];
        mIMU.getMagData(&mx,&my,&mz);
        //14 bit output.
        Mxyz[0] = (float) mx * mIMU.mMagSensitivity * mIMU.mMagSensAdjData[0];
        Mxyz[1] = (float) my * mIMU.mMagSensitivity * mIMU.mMagSensAdjData[1];
        Mxyz[2] = (float) mz * mIMU.mMagSensitivity * mIMU.mMagSensAdjData[2];
        Mxyz[0] = (Mxyz[0] - mMagOffset[0]) * mMagScale[0];
        Mxyz[1] = (Mxyz[1] - mMagOffset[1]) * mMagScale[1];
        Mxyz[2] = (Mxyz[2] - mMagOffset[2]) * mMagScale[2];

        /*frame transformation -> coz mag is mounted on different axies with gyro and accel*/
        pMagData->x = Mxyz[1];
        pMagData->y = Mxyz[0];
        pMagData->z = Mxyz[2]*(-1);
        return true;
    } else {
        LOGI("Mag data not ready, skip\r\n");
        return false;
    }
}

bool IMU::GetRawCompassData(FCSensorDataType* pMagData)
{
    if (!mMagEnabled) {
        LOGE("Mag is not enabled\r\n");
        return false;
    }

    uint8_t dataReady = mIMU.getCompassDataReady();
    if (dataReady == 1) {
        int16_t mx, my, mz;
        mIMU.getMagData(&mx,&my,&mz);
        pMagData->x = (float) mx * mIMU.mMagSensitivity * mIMU.mMagSensAdjData[0];
        pMagData->y = (float) my * mIMU.mMagSensitivity * mIMU.mMagSensAdjData[1];
        pMagData->z = (float) mz * mIMU.mMagSensitivity * mIMU.mMagSensAdjData[2];
        return true;
    }else{
        LOGI("Mag data not ready, skip\r\n");
        return false;
    }
}

void IMU::CalibrateMag()
{
    if (!mMagEnabled) {
        LOGE("Mag is not enabled\r\n");
        return;
    }

    uint16_t ii = 0, sample_count = 0;
    float mag_max[3];
    float mag_min[3];
    float mag_delta[3] = {0.0f};
    for (int i = 0; i < 3; ++i) {
        mag_max[i] = mIMU.mMagRange * (-1);
        mag_min[i] = mIMU.mMagRange;
    }

    LOGI("Mag Calibration: Wave device in a figure eight until done! \r\n");
    HAL_Delay(2000);

    sample_count = 1000;
    FCSensorDataType magData;
    for(ii = 0; ii < sample_count; ii++) {
        GetRawCompassData(&magData);  // Read the mag data
        if(magData.x > mag_max[0]) mag_max[0] = magData.x;
        if(magData.x < mag_min[0]) mag_min[0] = magData.x;
        if(magData.y > mag_max[1]) mag_max[1] = magData.y;
        if(magData.y < mag_min[1]) mag_min[1] = magData.y;
        if(magData.z > mag_max[2]) mag_max[2] = magData.z;
        if(magData.z < mag_min[2]) mag_min[2] = magData.z;
        HAL_Delay(5); // read at 200 Hz
    }

    // Get hard iron correction
    mMagOffset[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mMagOffset[1]   = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mMagOffset[2]   = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    // Get soft iron correction estimate
    mag_delta[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_delta[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_delta[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = (mag_delta[0] + mag_delta[1] + mag_delta[2]) / 3.0f;

    mMagScale[0] = avg_rad / mag_delta[0];
    mMagScale[1] = avg_rad / mag_delta[1];
    mMagScale[2] = avg_rad / mag_delta[2];

    mMagCalibrateFlag = true;
}

void IMU::CalibrateSensorBias()
{
    int counter = 0;

    FCSensorDataType gyroData;
    FCSensorDataType accData;
    FCSensorDataType magData;

#if USE_INTERRUPT
    ClearInterrupt();
#endif
    while (counter < 100) {
        if (mGyroAccDataRdyFlag) {
            GetRawGyroData(&gyroData);
            GetAccelData(&accData);
            if (mMagEnabled) {
                GetCompassData(&magData);
            } else {
                magData.x = 0.0f;
                magData.y = 0.0f;
                magData.z = 0.0f;
            }
            if (counter < 100) {
                magConst[0] += magData.x;
                magConst[1] += magData.y;
                magConst[2] += magData.z;
                gravity[0] += accData.x;
                gravity[1] += accData.y;
                gravity[2] += accData.z;
                gyroBias[0] += gyroData.x;
                gyroBias[1] += gyroData.y;
                gyroBias[2] += gyroData.z;

                ++counter;
            }
            if (counter == 100) {
                for (int i = 0; i < 3; i++) {
                    magConst[i] = magConst[i] / 100;
                    gravity[i] = gravity[i] / 100;
                    gyroBias[i] = gyroBias[i] / 100;
                }
                mSensorBiasCalibrateFlag = true;
            }
#if USE_INTERRUPT
            mGyroAccDataRdyFlag = false;
            ClearInterrupt();
#else
            HAL_Delay(5); // read at 200Hz
#endif
        } else {
            /*data not ready*/
            // LOG("Data not Ready \r\n");
        }
        HAL_Delay(1);
    }
    LOGI("gyroBias: %.2f, %.2f, %.2f, MagConst: %.2f, %.2f, %.2f\r\n",
         gyroBias[0], gyroBias[1], gyroBias[2],
         magConst[0], magConst[1], magConst[2]);

}

#if USE_INTERRUPT
void IMU::ClearInterrupt()
{
    mIMU.readIntStatus();
}
#endif

bool IMU::GetDataReady(uint8_t* pDataReady)
{
    return mIMU.GetDataReady(pDataReady);
}

