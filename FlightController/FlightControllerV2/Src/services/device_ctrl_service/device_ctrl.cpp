#include "device_ctrl.h"

#include "i2c.h"
#include "IMU.h"
#include "logging.h"
#include "uart.h"
#include "sbus.h"
#include "pwm.h"
#include "led.h"

#include "state_estimator.h"
#include "cmd_listener.h"
#include "controller.h"
#include "sensor_reader.h"

#define LOG_TAG ("DeviceCtrl")

/*
* Static
*/

/*
* Code
*/

static bool DriversInit()
{
    // Init uart driver
    if (!UART_Init()) {
        LOGE("UART Init failed\r\n");
        return false;
    } else {
        LOGI("UART Init success\r\n");
    };
    // Init i2c driver
    if (!I2C_Init()) {
        LOGE("I2C Init failed\r\n");
        return false;
    } else {
        LOGI("I2C Init success\r\n");
    }
    // Init SBUS driver
//    if (!SBUS_Init()) {
//        LOGE("SBUS Init failed\r\n");
//        return false;
//    } else {
//        LOGI("SBUS Init success\r\n");
//    }
    // Init PWM driver
    if (!PWM_Init()) {
        LOGE("PWM Init failed\r\n");
        return false;
    } else {
        PWM_Start();
        LOGI("PWM Init success\r\n");
    }
    // Init LED driver
    if (!LED_Init()) {
        LOGE("LED Init failed\r\n");
        return false;
    } else {
        LOGI("PWM Init success\r\n");
    }

    return true;
}

static bool ProductInit()
{
    if (!StateEstimator::GetInstance().Init()) {
        LOGE("StateEstimator Init failed\r\n");
        return false;
    }
    if (!SensorReader::GetInstance().Init()) {
        LOGE("SensorReader Init failed\r\n");
        return false;
    }
    if (!Controller::GetInstance().Init()) {
        LOGE("Controller Init failed\r\n");
        return false;
    }
    if (!CmdListener::GetInstance().Init()) {
        LOGE("CmdListener Init failed\r\n");
        return false;
    }

    return true;
}

static bool ProductStart()
{
    if (!CmdListener::GetInstance().Start()) {
        LOGE("CmdListener Start failed\r\n");
        return false;
    }
    return true;
}

bool DeviceInit()
{
    if (!DriversInit()) {
        LOGE("DriversInit failed\r\n");
        return false;
    }
    if (!ProductInit()) {
        LOGE("ProductInit failed\r\n");
        return false;
    }
    if (!ProductStart()) {
        LOGE("ProductStart failed\r\n");
        return false;
    }

    return true;
}


