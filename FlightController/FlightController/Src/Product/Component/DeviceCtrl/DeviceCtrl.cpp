#include "stm32f4xx_hal.h"


#include <i2c.h>
#include <IMU.h>
#include <Interrupt.h>
#include <logging.h>
#include <uart.h>

#include "DeviceCtrl.h"


#define LOG_TAG ("DeviceCtrl")

/*
 * Static
 */

static IMU* spIMU = NULL;

/*
 * Code
 */
void DeviceInit()
{
    HAL_Init();

    DeviceClkConfig();
    DeviceGPIOInit();

    DriversInit();
    // UserHalInit(); TODO
}

void DriversInit()
{
    // Init interrupt drivers
    InterruptInit();
    // Init i2c drivers
    I2C_Init();
    // Init uart drivers
    UART_Init();
}

/** Configure pins as
* Analog
* Input
* Output
* EVENT_OUT
* EXTI
PC1   ------> ETH_MDC
PA1   ------> ETH_REF_CLK
PA2   ------> ETH_MDIO
PA7   ------> ETH_CRS_DV
PC4   ------> ETH_RXD0
PC5   ------> ETH_RXD1
PB13   ------> ETH_TXD1
PA8   ------> USB_OTG_FS_SOF
PA9   ------> USB_OTG_FS_VBUS
PA10   ------> USB_OTG_FS_ID
PA11   ------> USB_OTG_FS_DM
PA12   ------> USB_OTG_FS_DP
PG11   ------> ETH_TX_EN
PG13   ------> ETH_TXD0
*/
void DeviceGPIOInit()
{
   GPIO_InitTypeDef GPIO_InitStruct;

   /* GPIO Ports Clock Enable */
   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_GPIOH_CLK_ENABLE();
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_GPIOF_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();
   __HAL_RCC_GPIOG_CLK_ENABLE();

   /*Configure GPIO pin : User_Blue_Button_Pin */
   GPIO_InitStruct.Pin = User_Blue_Button_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(User_Blue_Button_GPIO_Port, &GPIO_InitStruct);

   /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
   GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
   GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /*Configure GPIO pins : PB0 LD3_Pin LD2_Pin */
   GPIO_InitStruct.Pin = GPIO_PIN_0|LD3_Pin|LD2_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   /*Configure GPIO pin : RMII_TXD1_Pin */
   GPIO_InitStruct.Pin = RMII_TXD1_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
   HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

   /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
   GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

   /*Configure GPIO pin : USB_OverCurrent_Pin */
   GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

   /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
   GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /*Configure GPIO pin : USB_VBUS_Pin */
   GPIO_InitStruct.Pin = USB_VBUS_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

   /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
   GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
   HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

   /* EXTI interrupt init*/
   HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/** System Clock Configuration
*/
void DeviceClkConfig(void)
{

   RCC_OscInitTypeDef RCC_OscInitStruct;
   RCC_ClkInitTypeDef RCC_ClkInitStruct;

   /**Configure the main internal regulator output voltage
   */
   __HAL_RCC_PWR_CLK_ENABLE();

   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

   /**Initializes the CPU, AHB and APB busses clocks
   */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
   RCC_OscInitStruct.HSICalibrationValue = 16;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
   RCC_OscInitStruct.PLL.PLLM = 16;
   RCC_OscInitStruct.PLL.PLLN = 360;
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
   RCC_OscInitStruct.PLL.PLLQ = 4;
   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
   {
      // Error_Handler();
   }

   /**Initializes the CPU, AHB and APB busses clocks
   */
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
      |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
   {
      // Error_Handler();
   }

   /**Configure the Systick interrupt time
   */
   HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

   /**Configure the Systick
   */
   HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

   /* SysTick_IRQn interrupt configuration */
   HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

bool UserHalInit()
{
    spIMU = new IMU();
    if (!spIMU) {
        LOGE("IMU init failed\r\n");
        return false;
    }
    return true;
}