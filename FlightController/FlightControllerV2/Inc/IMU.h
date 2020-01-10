#ifndef HAL_IMU_H_
#define HAL_IMU_H_

#include <UAV_Defines.h>

#include "MPU9250.h"

/*
 * Defines
 */

#define USE_INTERRUPT (0)

typedef void (*DataReadyCb)(void);

class IMU
{
private:
    MPU9250 mIMU;

    float gravity[3];
    float magConst[3];
    float gyroBias[3];
    float mMagOffset[3];
    float mMagScale[3];

    // flags
    bool mReadyToStart;
    bool mSensorBiasCalibrateFlag;
    bool mGyroAccDataRdyFlag;
    bool mMagCalibrateFlag;
    bool mMagEnabled;

    //FCSensorDataType gyroData;
    //FCSensorDataType accData;
    //FCSensorDataType magData;
    //FCSensorDataType rawGyroData;
    //FCSensorDataType rawAccData;
    //FCSensorDataType rawMagData;

    // private constructor, singleton paradigm
    IMU();
public:
    static IMU& GetInstance();

    bool Init();
    bool EnableMag(bool enable);
    bool Start();
    void CalibrateSensorBias();
    void CalibrateMag();
    void GetGravityVector(float* pGravity);
    void GetMagConstVector(float* pMagConst);
    bool GetCompassData(FCSensorDataType* pMagData);
    void GetAccelData(FCSensorDataType* pAccData);
    void GetGyroData(FCSensorDataType* pGyroData);
    bool GetDataReady(uint8_t* pDataReady);

    bool GetRawCompassData(FCSensorDataType* pMagData);
    void GetRawGyroData(FCSensorDataType* pGyroData);

#if USE_INTERRUPT
    // interrupt
    void OnGyroAccDataReady();
    //cb
    DataReadyCb mDataReadyCb;
    bool SetDataReadyCb(DataReadyCb cb);
    void ClearInterrupt();
    void SetGyroAccDataReadyFlg();
#endif
};



#endif