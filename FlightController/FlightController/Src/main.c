/**
******************************************************************************
* File Name          : main.c
* Description        : Main program body
******************************************************************************
*
* COPYRIGHT(c) 2016 STMicroelectronics
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

#include <logging.h>

#include "myMPU9250.hpp"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
//I2C_HandleTypeDef hi2c1;
//
//UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/*flags*/
uint8_t MPU9250_DataRdyFlag = 0;
uint8_t initDataRdy = 0;
uint8_t magCalibrateFlag = 1;

/*mag calibration data*/
static float mx_centre;
static float my_centre;
static float mz_centre;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

float gravity[3];
float MagConst[3];
float gyroBias[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void getGyroData();
void getAccelData();
void getCompassData();
void getRawCompassData();
void calibrateMag();
void getRawGyroData();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//MPU9250 *myMPU = new MPU9250 (&hi2c1);
/* USER CODE END 0 */


/*
 * Code
 */

/************************
 * Initialization
 ***********************/

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
static void MX_GPIO_Init(void)
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

   /*Configure GPIO pin : MPU9250_Interrupt_Pin */
   GPIO_InitStruct.Pin = MPU9250_Interrupt_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(MPU9250_Interrupt_GPIO_Port, &GPIO_InitStruct);

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
void SystemClock_Config(void)
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
      Error_Handler();
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
      Error_Handler();
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

/* I2C1 init function */
//static void MX_I2C1_Init(void)
//{
//
//   hi2c1.Instance = I2C1;
//   hi2c1.Init.ClockSpeed = 100000;
//   hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//   hi2c1.Init.OwnAddress1 = 0;
//   hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//   hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//   hi2c1.Init.OwnAddress2 = 0;
//   hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//   hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//   if (HAL_I2C_Init(&hi2c1) != HAL_OK)
//   {
//      Error_Handler();
//   }
//
//}

/* USART3 init function */
//static void MX_USART3_UART_Init(void)
//{
//
//   huart3.Instance = USART3;
//   huart3.Init.BaudRate = 115200;
//   huart3.Init.WordLength = UART_WORDLENGTH_8B;
//   huart3.Init.StopBits = UART_STOPBITS_1;
//   huart3.Init.Parity = UART_PARITY_NONE;
//   huart3.Init.Mode = UART_MODE_TX_RX;
//   huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//   huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//   if (HAL_UART_Init(&huart3) != HAL_OK)
//   {
//      Error_Handler();
//   }
//}

int main(void)
{

   /* USER CODE BEGIN 1 */

   /* USER CODE END 1 */

   /* MCU Configuration----------------------------------------------------------*/

   /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

   /* Configure the system clock */
   SystemClock_Config();

   /* Initialize all configured peripherals */
   MX_GPIO_Init();
   I2C_Init();
   UART_Init();
   // MX_USART3_UART_Init();
   // MX_I2C1_Init();

   /* USER CODE BEGIN 2 */
   uint8_t deviceID;
   deviceID = myMPU->getDeviceID();
   if (deviceID == 0x71) {
      myMPU->initialize();//initialzie
   } else {
      printf("ID wrong");
      return 0;
   }

   printf("Device OK, reading data \r\n");

   /* USER CODE END 2 */

   /* Infinite loop */
   /* USER CODE BEGIN WHILE */
   if(magCalibrateFlag){
      calibrateMag();
      printf("Mag Calibration done! \r\n");
      HAL_Delay(4000);
      printf("Put the device to rest!! \r\n");
   }

   HAL_Delay(4000);

   //enable interrupt
   myMPU->enableInterrupt();

   uint8_t counter = 0;

   float sMag[3] = {0.0f,0.0f,0.0f};
   float sAcc[3] = {0.0f,0.0f,0.0f};
   float sGyro[3] = {0.0f,0.0f,0.0f};
   myMPU->readIntStatus();

   while (1)
   {
      if (!initDataRdy){
         if (MPU9250_DataRdyFlag){
            getRawGyroData();
            getAccelData();
            getCompassData();
            if(counter<100){
               for (int i=0;i<3;i++){
                  sMag[i] = sMag[i] + Mxyz[i];
                  sAcc[i] = sAcc[i] + Axyz[i];
                  sGyro[i] = sGyro[i] + Gxyz[i];
               }
               counter = counter+1;
            }
            if (counter==100){
               for (int i=0;i<3;i++){
                  MagConst[i] = sMag[i]/100;
                  gravity[i] = sAcc[i]/100;
                  gyroBias[i] = sGyro[i]/100;
               }
               initDataRdy = 1;
            }
         } else {
            /*data not ready*/
            printf("Data not Ready \r\n");
         }
      } else {
         if (MPU9250_DataRdyFlag){
            /*get data*/
            getGyroData();
            getAccelData();
            getCompassData();
            //printf("%f %f %f %f %f %f %f %f %f \r\n",Gxyz[0],Gxyz[1],Gxyz[2],Axyz[0],Axyz[1],Axyz[2],Mxyz[0],Mxyz[1],Mxyz[2]);

            /*clear flag and interrupt status*/
            MPU9250_DataRdyFlag = 0;
            myMPU->readIntStatus();

            /*collect steady state gyro measurement by averaging intial 50 gyromeasurement.*/

            /*mag initial data collected, lets go!!*/
            /*run KalmanFilter*/
            //QKF::PredictState(); TODO
            //QKF::UpdateState(); TODO
            //QKF::GetState();

            /*print the resultant quaternion to serial terminal*/
            // printf("Q: %f %f %f %f \r\n",data_matrix_X_prev[0],data_matrix_X_prev[1],data_matrix_X_prev[2],data_matrix_X_prev[3]);
         }else{
            /*data not ready*/
            printf("Data not Ready \r\n");
         }
      }
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */

      /* USER CODE END 3 */
   }
}

/* USER CODE BEGIN 4 */
void getGyroData()
{
   int16_t gx, gy, gz;
   myMPU->getRotation(&gx, &gy, &gz);
   Gxyz[0] = (float) gx * 500 / 32768;//131 LSB(??/s)
   Gxyz[1] = (float) gy * 500 / 32768;
   Gxyz[2] = (float) gz * 500 / 32768;
   Gxyz[0] = Gxyz[0] - gyroBias[0];
   Gxyz[1] = Gxyz[1] - gyroBias[1];
   Gxyz[2] = Gxyz[1] - gyroBias[2];
   //High Pass Filter -> remove all values that are less than 0.05dps.
   for (int i=0;i<3;i++){
      if(Gxyz[i]<0.05){
         Gxyz[i]=0;
      }
   }
}

void getRawGyroData()
{
   int16_t gx, gy, gz;
   myMPU->getRotation(&gx, &gy, &gz);
   Gxyz[0] = (float) gx * 500 / 32768;//131 LSB(??/s)
   Gxyz[1] = (float) gy * 500 / 32768;
   Gxyz[2] = (float) gz * 500 / 32768;
}

void getAccelData(){
   int16_t ax, ay, az;
   myMPU->getAcceleration(&ax,&ay,&az);
   Axyz[0] = (float) ax / 16384;//16384  LSB/g
   Axyz[1] = (float) ay / 16384;
   Axyz[2] = (float) az / 16384;
}

void getCompassData(){
   uint8_t dataReady = myMPU->getCompassDataReady();
   if (dataReady == 1){
      int16_t mx, my, mz;
      myMPU->getMagData(&mx,&my,&mz);
      //14 bit output.
      Mxyz[0] = (float) mx * 4912 / 8192;
      Mxyz[1] = (float) my * 4912 / 8192;
      Mxyz[2] = (float) mz * 4912 / 8192;
      Mxyz[0] = Mxyz[0] - mx_centre;
      Mxyz[1] = Mxyz[1] - my_centre;
      Mxyz[2] = Mxyz[2] - mz_centre;

      /*frame transformation -> coz mag is mounted on different axies with gyro and accel*/
      float temp = Mxyz[0];
      Mxyz[0] = Mxyz[1];
      Mxyz[1] = temp;
      Mxyz[2] = Mxyz[2]*(-1);
   }
}

void getRawCompassData(){
   uint8_t dataReady = myMPU->getCompassDataReady();
   if (dataReady == 1){
      int16_t mx, my, mz;
      myMPU->getMagData(&mx,&my,&mz);
      //14 bit output.
      Mxyz[0] = (float) mx * 4912 / 8192;
      Mxyz[1] = (float) my * 4912 / 8192;
      Mxyz[2] = (float) mz * 4912 / 8192;
   }else{
      printf("Mag data not ready, using original data");
   }
}

void calibrateMag(){
   uint16_t ii = 0, sample_count = 0;
   float mag_max[3] = {1,1,1};
   float mag_min[3] = {-1,-1,-1};

   printf("Mag Calibration: Wave device in a figure eight until done! \r\n");
   HAL_Delay(2000);

   sample_count = 100;
   for(ii = 0; ii < sample_count; ii++) {
      getRawCompassData();  // Read the mag data
      for (int jj = 0; jj < 3; jj++) {
         if(Mxyz[jj] > mag_max[jj]) mag_max[jj] = Mxyz[jj];
         if(Mxyz[jj] < mag_min[jj]) mag_min[jj] = Mxyz[jj];
      }
      HAL_Delay(200);
   }

   // Get hard iron correction
   mx_centre  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
   my_centre  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
   mz_centre  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts


   // Get soft iron correction estimate
   /*
   mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
   mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
   mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

   float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
   avg_rad /= 3.0;

   dest2[0] = avg_rad/((float)mag_scale[0]);
   dest2[1] = avg_rad/((float)mag_scale[1]);
   dest2[2] = avg_rad/((float)mag_scale[2]);
   */



}

/* USER CODE END 4 */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
* @brief  Retargets the C library printf function to the USART.
* @param  None
* @retval None
*/
PUTCHAR_PROTOTYPE
{
   /* Place your implementation of fputc here */
   /* e.g. write a character to the USART3 and Loop until the end of transmission */
   HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

   return ch;
}

/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
void Error_Handler(void)
{
   /* USER CODE BEGIN Error_Handler */
   /* User can add his own implementation to report the HAL error return state */
   while(1)
   {
   }
   /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
   /* USER CODE BEGIN 6 */
   /* User can add his own implementation to report the file name and line number,
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
   /* USER CODE END 6 */

}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
