#include "stm32f4xx_hal.h"

#include "TestIMU.h"

#include "IMU.h"
#include "led.h"
#include "logging.h"
#include "QKF.h"
#include "SparkFunMPU9250-DMP.h"
#include "MadgwickAHRS.h"
#include "cmd_listener.h"
#include "pwm.h"
#include "sbus.h"

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

// static void IMUDataReadyCb();
// static void TestIMU_Init();

/*
* Code
*/
#if 0
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
#endif

void TestMPU9250DMP_Main()
{
    MPU9250_DMP imu;
    if (imu.begin() != INV_SUCCESS) {
        LOGE("Unable to communicate with MPU-9250");
        return;
    }

    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                 DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                 MAX_DMP_SAMPLE_RATE); // Set DMP FIFO rate to 200 Hz

    while (1) {
        // Check for new data in the FIFO
        if (imu.fifoAvailable()) {
            // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
            if ( imu.dmpUpdateFifo() == INV_SUCCESS) {
                // computeEulerAngles can be used -- after updating the
                // quaternion values -- to estimate roll, pitch, and yaw
                imu.computeEulerAngles();
                FCQuaternionType quat;
                quat.q1 = imu.calcQuat(imu.qw);
                quat.q2 = imu.calcQuat(imu.qx);
                quat.q3 = imu.calcQuat(imu.qy);
                quat.q4 = imu.calcQuat(imu.qz);
                PRINT("Q: %f %f %f %f %u\r\n", quat.q1, quat.q2, quat.q3, quat.q4, HAL_GetTick()); // this will send through UART as well
            }
        }
    }
}

void TestMadgwick_Main()
{
    LOGI("%s\r\n", __func__);

    IMU& imu = IMU::GetInstance();
    if (!imu.Init()) {
        LOGE("IMU init failed\r\n");
        return;
    }
    LOGI("IMU init success\r\n");

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

    FCSensorDataType magData;
    FCSensorDataType accData;
    FCSensorDataType gyroData;
    FCQuaternionType quat;
    Madgwick filter;
    filter.begin(50);

    // from magConst and gravity, calculate initial orientation quaternion.
    float magConst[3];
    imu.GetMagConstVector(magConst);
    float theta = atan2(magConst[1], magConst[0]) * (-0.5f); // calculate angle
    quat.q1 = arm_cos_f32(theta);
    quat.q4 = arm_sin_f32(theta);
    quat.q2 = 0.0f;
    quat.q3 = 0.0f;
    //TODO gravity
    filter.SetInitialOrientation(quat);

    while (1) {
        uint8_t imuDataReady = 1;
        // imu.GetDataReady(&imuDataReady);
        if (imuDataReady) {
            // Get Data
            imu.GetAccelData(&accData);
            imu.GetGyroData(&gyroData);
            while (!imu.GetCompassData(&magData)) {
                LOGI("magData not ready, set to 0\r\n");
                magData.x = 0.0f;
                magData.y = 0.0f;
                magData.z = 0.0f;
            }

            LOGI("Gyro: %.2f %.2f %.2f, Acc: %.2f %.2f %.2f, Mag: %.2f %.2f %.2f\r\n",
                gyroData.x, gyroData.y, gyroData.z,
                accData.x, accData.y, accData.z,
                magData.x, magData.y, magData.z);

            /*run Madgwick Filter*/
            filter.update(gyroData.x, gyroData.y, gyroData.z, accData.x, accData.y, accData.z, magData.x * 10, magData.y * 10, magData.z * 10);
//            filter.updateIMU(gyroData.x, gyroData.y, gyroData.z, accData.x, accData.y, accData.z);

            filter.GetQuat(&quat);
            /*print the resultant quaternion to serial terminal*/
            // PRINT("tick = %u\r\n", HAL_GetTick());
            PRINT("Q: %f %f %f %f %u\r\n", quat.q1, quat.q2, quat.q3, quat.q4, HAL_GetTick()); // this will send through UART as well
            // PRINT("Orientation: %f %f %f\r\n", heading, pitch, roll);
        } else {
            PRINT("Data not ready, sleep\r\n");
        }
        HAL_Delay(5);
    }
}

void TestCmdListener()
{
//    LED_SetOn(LED_RED, true);
//    HAL_Delay(1000);
//    LED_SetOn(LED_RED, false);
//    LED_SetOn(LED_GREEN, true);
//    HAL_Delay(1000);
//    LED_SetOn(LED_GREEN, false);
//    LED_SetOn(LED_BLUE, true);
//    HAL_Delay(1000);
//    LED_SetOn(LED_BLUE, false);
//    HAL_Delay(1000);
    SBUS_Start();
    CmdListener& cmdListener = CmdListener::GetInstance();
    FCCmdType cmd;
    while (1) {
        cmdListener.GetCmd(cmd);
        PRINT("desiredVel.x = %f, desiredVel.y = %f, desiredVel.z = %f, desiredYawRate = %f\r\n",
              cmd.desiredVel.x, cmd.desiredVel.y, cmd.desiredVel.z, cmd.desiredYawRate);
        HAL_Delay(1000);
    }
}

void TestPWM()
{
    PWM_SetDutyCycle(PWM_CHANNEL_1, 20);
    PWM_SetDutyCycle(PWM_CHANNEL_2, 40);
    PWM_SetDutyCycle(PWM_CHANNEL_3, 60);
    PWM_SetDutyCycle(PWM_CHANNEL_4, 80);
    while (1) {}
}