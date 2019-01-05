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
#define MPU9250_Interrupt_Pin        GPIO_PIN_12
#define MPU9250_Interrupt_GPIO_Port  GPIOF

IMU::IMU() :
    mIMU()
{
    /*Configure GPIO pin : MPU9250_Interrupt_Pin */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = MPU9250_Interrupt_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MPU9250_Interrupt_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    mGyroAccDataRdyFlag = false;
    mSensorBiasCalibrateFlag = false;
    mMagCalibrateFlag = false;
    mReadyToStart = false;
    mDataReadyCb = NULL;

    for (int i = 0; i < 3; i++) {
        magConst[i] = 0.0f;
        gravity[i] = 0.0f;
        gyroBias[i] = 0.0f;
    }
}

IMU& IMU::GetInstance()
{
    static IMU imu;
    return imu;
}

/*
* Interrupt Handler
*/
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
        mIMU.enableInterrupt();
    } else {
        LOGE("failed to start IMU \r\n");
        return false;
    }

    return true;
}

bool IMU::SetDataReadyCb(DataReadyCb cb)
{
    if (cb == NULL) {
        return false;
    }

    mDataReadyCb = cb;
    return true;
}

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
    Gxyz[0] = (float) gx * 500 / 32768;//131 LSB(??/s)
    Gxyz[1] = (float) gy * 500 / 32768;
    Gxyz[2] = (float) gz * 500 / 32768;
    Gxyz[0] = Gxyz[0] - gyroBias[0];
    Gxyz[1] = Gxyz[1] - gyroBias[1];
    Gxyz[2] = Gxyz[2] - gyroBias[2];
    //High Pass Filter -> remove all values that are less than 0.05dps.
    for (int i=0; i < 3; ++i){
        if (-0.05 < Gxyz[i] &&  Gxyz[i] < 0.05){
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
    pGyroData->x = (float) gx * 500 / 32768;//131 LSB(??/s)
    pGyroData->y = (float) gy * 500 / 32768;
    pGyroData->z = (float) gz * 500 / 32768;
}

void IMU::GetAccelData(FCSensorDataType* pAccData)
{
    int16_t ax, ay, az;
    mIMU.getAcceleration(&ax,&ay,&az);
    pAccData->x = (float) ax / 16384;//16384  LSB/g
    pAccData->y = (float) ay / 16384;
    pAccData->z = (float) az / 16384;
}

bool IMU::GetCompassData(FCSensorDataType* pMagData)
{
    uint8_t dataReady = mIMU.getCompassDataReady();
    if (dataReady == 1){
        int16_t mx, my, mz;
        float Mxyz[3];
        mIMU.getMagData(&mx,&my,&mz);
        //14 bit output.
        Mxyz[0] = (float) mx * 4912 / 8192;
        Mxyz[1] = (float) my * 4912 / 8192;
        Mxyz[2] = (float) mz * 4912 / 8192;
        Mxyz[0] = Mxyz[0] - mx_centre;
        Mxyz[1] = Mxyz[1] - my_centre;
        Mxyz[2] = Mxyz[2] - mz_centre;

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
    uint8_t dataReady = mIMU.getCompassDataReady();
    if (dataReady == 1) {
        int16_t mx, my, mz;
        mIMU.getMagData(&mx,&my,&mz);
        // 14 bit output. // TODO frame transformation here??
        pMagData->x = (float) mx * 4912 / 8192;
        pMagData->y = (float) my * 4912 / 8192;
        pMagData->z = (float) mz * 4912 / 8192;
        return true;
    }else{
        LOGI("Mag data not ready, skip\r\n");
        return false;
    }
}

void IMU::CalibrateMag()
{
    uint16_t ii = 0, sample_count = 0;
    float mag_max[3] = {1,1,1};
    float mag_min[3] = {-1,-1,-1};

    LOGI("Mag Calibration: Wave device in a figure eight until done! \r\n");
    HAL_Delay(2000);

    sample_count = 100;
    float Mxyz[3];
    for(ii = 0; ii < sample_count; ii++) {
        GetRawCompassData((FCSensorDataType*)&Mxyz);  // Read the mag data
        for (int jj = 0; jj < 3; jj++) {
            if(Mxyz[jj] > mag_max[jj]) mag_max[jj] = Mxyz[jj];
            if(Mxyz[jj] < mag_min[jj]) mag_min[jj] = Mxyz[jj];
        }
        HAL_Delay(200);
    }

    // Get hard iron correction
    mx_centre  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    my_centre  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mz_centre  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts


    // Get soft iron correction estimate
    /*
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
    */
    mMagCalibrateFlag = true;
}

void IMU::CalibrateSensorBias()
{
    int counter = 0;

    FCSensorDataType gyroData;
    FCSensorDataType accData;
    FCSensorDataType magData;

    ClearInterrupt();
    while (counter < 100) {
        if (mGyroAccDataRdyFlag) {
            GetRawGyroData(&gyroData);
            GetAccelData(&accData);
            GetCompassData(&magData);
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
            mGyroAccDataRdyFlag = false;
            ClearInterrupt();
        } else {
            /*data not ready*/
            // LOG("Data not Ready \r\n");
        }
    }
    LOGI("gyroBias: %.2f, %.2f, %.2f, MagConst: %.2f, %.2f, %.2f\r\n",
         gyroBias[0], gyroBias[1], gyroBias[2],
         magConst[0], magConst[1], magConst[2]);

}

void IMU::ClearInterrupt()
{
    mIMU.readIntStatus();
}

