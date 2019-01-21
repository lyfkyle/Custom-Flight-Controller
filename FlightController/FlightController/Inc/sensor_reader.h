#ifndef _SENSOR_READER_H_
#define _SENSOR_READER_H_

#include "UAV_Defines.h"

class SensorReader {
private:
    SensorReader(); // private constructor, singleton

    FCSensorDataType mSensorData;
public:
    static SensorReader& GetInstance();

    bool GetSensorMeas(FCSensorMeasType& cmd);
};

#endif