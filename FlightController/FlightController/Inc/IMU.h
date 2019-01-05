#ifndef HAL_IMU_H_
#define HAL_IMU_H_

#include <UAV_Defines.h>

#include "MPU9250.h"

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

    // interrupt
    void OnGyroAccDataReady();

     //cb
    DataReadyCb mDataReadyCb;
    void SetGyroAccDataReadyFlg();
    bool Init();
    bool Start();
    bool SetDataReadyCb(DataReadyCb cb);
    void CalibrateSensorBias();
    void CalibrateMag();
    void GetGravityVector(float* pGravity);
    void GetMagConstVector(float* pMagConst);
    bool GetRawCompassData(FCSensorDataType* pMagData);
    bool GetCompassData(FCSensorDataType* pMagData);
    void GetAccelData(FCSensorDataType* pAccData);
    void GetRawGyroData(FCSensorDataType* pGyroData);
    void GetGyroData(FCSensorDataType* pGyroData);
    void ClearInterrupt();
};



#endif