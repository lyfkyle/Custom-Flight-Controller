#include "stm32f4xx_hal.h"

#include "TestIMU.h"

#include "IMU.h"
#include "logging.h"
#include "QKF.h"

/*
 * Defines
 */

#define LOG_TAG ("TEST_IMU")
#define LOG(...)
/*
 * Statics
 */

//static IMU sIMU; // for testing only. App should not hold IMU object
static volatile bool sIMUDataReady = false;

/*flags*/


/*mag calibration data*/


//float Axyz[3];
//float Gxyz[3];
//float Mxyz[3];



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
    LOGI("%s\r\n", __func__);

    IMU& imu = IMU::GetInstance();
    if (!imu.Init()) {
        LOGE("IMU init failed\r\n");
        return;
    }
    LOGI("IMU init success\r\n");

    // Register dataReady Cb
    imu.SetDataReadyCb(IMUDataReadyCb);

    // start IMU
    if (!imu.Start()) {
        LOGE("Failed to start IMU\r\n");
        return;
    }
    LOGI("IMU start success\r\n");

    LOGI("calibrating mag\r\n", __func__);
    imu.CalibrateMag();
    LOGI("Mag Calibration done! \r\n");
    LOGI("Put the device to rest!! \r\n");
    HAL_Delay(4000);

    imu.CalibrateSensorBias();
    LOGI("IMU calibrate success\r\n");

    QKF* pQKF = new QKF();
    float gravityVector[3];
    float magConst[3];
    imu.GetGravityVector(gravityVector);
    imu.GetMagConstVector(magConst);
    pQKF->SetGravityVector(gravityVector);
    pQKF->SetMagConstVector(magConst);

    FCSensorDataType magData;
    FCSensorDataType accData;
    FCSensorDataType gyroData;
    FCQuaternionType quat;
    pQKF->GetState(&quat);

    /*print the resultant quaternion to serial terminal*/
    LOGI("Q: %f %f %f %f \r\n", quat.q1, quat.q2, quat.q3, quat.q4); // this will send through UART as well

    imu.ClearInterrupt();
    while (1)
    {
        if (sIMUDataReady) {
            // Get Data
            imu.GetAccelData(&accData);
            imu.GetGyroData(&gyroData);
            while (!imu.GetCompassData(&magData)) {
                LOGI("magData not ready, retry");
            }

            LOG("Gyro: %.2f %.2f %.2f, Acc: %.2f %.2f %.2f, Mag: %.2f %.2f %.2f\r\n",
                 gyroData.x, gyroData.y, gyroData.z,
                 accData.x, accData.y, accData.z,
                 magData.x, magData.y, magData.z);
            sIMUDataReady = false;
            imu.ClearInterrupt();

            /*run KalmanFilter*/
            pQKF->PredictState(&gyroData);
            pQKF->UpdateState(&accData, &magData);
            pQKF->GetState(&quat);

            /*print the resultant quaternion to serial terminal*/
            // PRINT("tick = %u\r\n", HAL_GetTick());
            PRINT("Q: %f %f %f %f %u\r\n", quat.q1, quat.q2, quat.q3, quat.q4, HAL_GetTick()); // this will send through UART as well
        } else {
            // LOGI("Data not ready, sleep\r\n");
            HAL_Delay(1);
        }


   }

   delete pQKF;
}

