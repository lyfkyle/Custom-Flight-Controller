#include <IMU.h>
#include <logging.h>
#include <QKF.h>

#include "stm32f4xx_hal.h"

/*
 * Defines
 */

#define LOG_TAG ("TEST_IMU")

/*
 * Statics
 */

//static IMU sIMU; // for testing only. App should not hold IMU object
static bool sIMUDataReady = false;

/*flags*/


/*mag calibration data*/


float Axyz[3];
float Gxyz[3];
float Mxyz[3];



/*
 * Prototypes
 */

static void IMUDataReadyCb();
// static void TestIMU_Init();

/*
 * Code
 */

static void IMUDataReadyCb()
{
    sIMUDataReady = true;
}

void TestIMU_Main()
{
    LOGI("%s", __func__);

    IMU* pIMU = new IMU();
    if (!pIMU->Init()) {
        LOGE("IMU init failed\r\n");
        return;
    }

    LOGI("%s, calibrating mag", __func__);
    pIMU->CalibrateMag();
    LOGI("Mag Calibration done! \r\n");
    HAL_Delay(4000);
    LOGI("Put the device to rest!! \r\n");
    HAL_Delay(4000);

    return;

    // Register dataReady Cb
    pIMU->SetDataReadyCb(IMUDataReadyCb);

    // start IMU
    if (!pIMU->Start()) {
        LOGE("Failed to start IMU\r\n");
        return;
    }

    QKF* pQKF = new QKF();
    float gravityVector[3];
    float magConst[3];
    pIMU->GetGravityVector(gravityVector);
    pIMU->GetMagConstVector(magConst);
    pQKF->SetGravityVector(gravityVector);
    pQKF->SetMagConstVector(magConst);

    uint8_t counter = 0;

    FCSensorDataType magData;
    FCSensorDataType accData;
    FCSensorDataType gyroData;

    while (1)
    {
        if (sIMUDataReady) {
            // Get Data
            pIMU->GetAccelData(&accData);
            pIMU->GetGyroData(&gyroData);
            while (!pIMU->GetCompassData(&magData)) {
                LOGI("magData not ready, retry");
            }

            /*run KalmanFilter*/
            pQKF->PredictState(&gyroData);
            pQKF->UpdateState(&accData, &magData);
            FCQuaternionType quat;
            pQKF->GetState(&quat);

            /*print the resultant quaternion to serial terminal*/
            LOGI("Q: %f %f %f %f \r\n", quat.q1, quat.q2, quat.q3, quat.q4); // this will send through UART as well
        } else {
            LOGI("Data not ready, sleep");
            HAL_Delay(1);
        }
   }

   delete pIMU;
   delete pQKF;
}

