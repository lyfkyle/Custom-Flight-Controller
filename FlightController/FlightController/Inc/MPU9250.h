#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <stdint.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"

class MPU9250 {
    public:
        MPU9250();
        //MPU9250(uint8_t address,I2C_HandleTypeDef hi2c);

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

        // INT_ENABLE register

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
        void setBypassEnableAndInterrupt();

        //INT_ENABLE register
        void enableInterrupt();

        //set Mag into Continuous Measuremnt Mode;
        void setMagContMeasMode();

        //read Mag Data
        void getMagData(int16_t* mx,int16_t* my, int16_t* mz);

        //check whether mag data is ready.
        uint8_t getCompassDataReady();

        //read Int_Status register to clear interrupt
        void readIntStatus();

    private:
        I2C_HandleTypeDef* hi2c;
        uint8_t devAddr;
        uint8_t ID;
        uint8_t buffer[14];
};

#endif /* _MPU9250_H_ */

