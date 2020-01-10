#include "sensor_reader.h"

#include "IMU.h"
#include "logging.h"

#define LOG_TAG ("SensorReader")

SensorReader::SensorReader()
{
}

SensorReader& SensorReader::GetInstance()
{
    static SensorReader sensorReader;
    return sensorReader;
}

bool SensorReader::Init()
{
    IMU& imu = IMU::GetInstance();
    if (!imu.Init()) {
        LOGE("IMU init failed\r\n");
        return false;
    }

    // start IMU
    if (!imu.Start()) {
        LOGE("Failed to start IMU\r\n");
        return false;
    }
    LOGI("IMU start success\r\n");

    imu.CalibrateSensorBias();
    LOGI("IMU calibrate success\r\n");
    return true;
}

bool SensorReader::GetSensorMeas(FCSensorMeasType& meas)
{
    IMU& imu = IMU::GetInstance();
    imu.GetGyroData(&(meas.gyroData));
    imu.GetAccelData(&(meas.accData));
    meas.accData.z *= (-1.0); // flip z value as IMU is mounted upside down
    return true;
}