#include "sensor_reader.h"

#include "IMU.h"
#include "logging.h"

SensorReader::SensorReader()
{
}

SensorReader& SensorReader::GetInstance()
{
    static SensorReader sensorReader;
    return sensorReader;
}

bool SensorReader::GetSensorMeas(FCSensorMeasType& meas)
{
    IMU& imu = IMU::GetInstance();
    imu.GetGyroData(&(meas.gyroData));
    imu.GetAccelData(&(meas.accData));
    return true;
}