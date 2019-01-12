#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <stdint.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"

typedef struct {
    uint8_t activeLvl;
    uint8_t intMode;
    uint8_t latch;
    uint8_t clearMethod;
} MPU9250IntConfigType;

class MPU9250 {
public:
    float mGyroSensitivity;
    float mAccSensitivity;
    float mMagSensitivity;
    float mMagSensAdjData[3];
    int mGyroFreq;
    int mAccFreq;
    int mMagFreq;
    int mGyroRange;
    int mAccRange;
    int mMagRange;

    MPU9250();

    void Init();

    // CONFIG register
    uint8_t getGyroDLPFMode();
    void setGyroDLPFMode(uint8_t bandwidth);

    // GYRO_CONFIG register
    uint8_t getFullScaleGyroRange();
    void setFullScaleGyroRange(uint8_t range);

    // ACCEL_CONFIG register
    uint8_t getFullScaleAccelRange();
    void setFullScaleAccelRange(uint8_t range);

    //ACCEL_CONFIG2 register
    uint8_t getAccDLPFMode();
    void setAccDLPFMode(uint8_t bandwidth);

    // INT_PIN_CFG register

    // INT_STATUS register

    // ACCEL_*OUT_* registers
    void getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);
    void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
    void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
    int16_t getAccelerationX();
    int16_t getAccelerationY();
    int16_t getAccelerationZ();

    // TEMP_OUT_* registers
    int16_t getTemperature();

    // GYRO_*OUT_* registers
    void getRotation(int16_t* x, int16_t* y, int16_t* z);
    int16_t getRotationX();
    int16_t getRotationY();
    int16_t getRotationZ();

    // PWR_MGMT_1 register
    void setSleepEnabled(bool enabled);
    void setClockSource(uint8_t source);

    // WHO_AM_I register
    uint8_t getDeviceID();

    //INT_PIN_CONFIG register
    //set I2c Bypass enable to access magnetometer and configure interrupt
    bool setBypassEnable(uint8_t enable);

    bool configInterrupt(MPU9250IntConfigType* pConfig);

    //INT_ENABLE register
    void enableInterrupt();

    // set Mag output bit
    bool setMagMeasOutputBit(uint8_t outputBit);

    //set Mag Measuremnt Mode;
    bool setMagContMeasMode(uint8_t mode);

    //read Mag Data
    void getMagData(int16_t* mx, int16_t* my, int16_t* mz);

    //check whether mag data is ready.
    uint8_t getCompassDataReady();

    // get Mag Sensitivity Adjustment Data
    bool GetMagSensitivityAdjData(float* pAdj);

    //read Int_Status register to clear interrupt
    void readIntStatus();
    bool GetDataReady(uint8_t* pDataReady);

private:
    uint8_t devAddr;
    uint8_t ID;
    uint8_t buffer[14];
};

#endif /* _MPU9250_H_ */

