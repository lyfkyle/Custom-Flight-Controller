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


/* USER CODE BEGIN Includes */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


#include "myMPU9250.hpp"
#include "arm_math.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

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

float32_t gravity[3];
float32_t MagConst[3];
float32_t gyroBias[3];

/*for Kalman Filter*/
arm_matrix_instance_f32 Matrix_X;//state
arm_matrix_instance_f32 Matrix_X_prev; // prev_state
arm_matrix_instance_f32 Matrix_A; //state_transition matrix
arm_matrix_instance_f32 Matrix_Q; //process noise covariacne matrix
arm_matrix_instance_f32 Matrix_P; //state noise covariacne
arm_matrix_instance_f32 Matrix_P_prev; //previous/updated state noise covariance
arm_matrix_instance_f32 Matrix_ra; // accel sensor nosie covariance
arm_matrix_instance_f32 Matrix_rm; // mag sesnor noise covariance
arm_matrix_instance_f32 Matrix_rg; //gyro sensor noise covariance
arm_matrix_instance_f32 Matrix_R; //meas noise covariance
arm_matrix_instance_f32 Matrix_S; 
arm_matrix_instance_f32 Matrix_H; //meas model 
arm_matrix_instance_f32 Matrix_H_T; //meas model transpose
arm_matrix_instance_f32 Matrix_K; //kalman gain
arm_matrix_instance_f32 Matrix_I; // identity matrix
arm_matrix_instance_f32 Matrix_skewX; 
arm_matrix_instance_f32 Matrix_skewX_T;
arm_matrix_instance_f32 Matrix_temp_12; //temporary matrix
arm_matrix_instance_f32 Matrix_temp_12_1;
arm_matrix_instance_f32 Matrix_temp_16;
arm_matrix_instance_f32 Matrix_temp_16_1;
arm_matrix_instance_f32 Matrix_temp_16_T;
arm_matrix_instance_f32 Matrix_temp_32;
arm_matrix_instance_f32 Matrix_temp_32_T;
arm_matrix_instance_f32 Matrix_temp_64;
float32_t accel[3];
float32_t mag[3];
float32_t temp_vector[3];
float32_t data_matrix_X[4] ={1,0,0,0};
float32_t data_matrix_X_prev[4] ={1,0,0,0};
float32_t data_matrix_A[16];
float32_t data_matrix_Q[16];
float32_t data_matrix_P[16];
float32_t data_matrix_P_prev[16] = {1,0,0,0,
                                    0,1,0,0,
                                    0,0,1,0,
                                    0,0,0,1};
float32_t data_matrix_ra[9] = {0.008,0,0,
                               0,0.008,0,
                               0,0,0.008};
float32_t data_matrix_rm[9] = {0.05,0,0,
                               0,0.05,0,
                               0,0,0.05};
float32_t data_matrix_rg[9] = {0.01,0,0,
                               0,0.01,0,
                               0,0,0.01};
float32_t data_matrix_R[64] = {0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0};
float32_t data_matrix_S[64];
float32_t data_matrix_H[32];
float32_t data_H_T[32];
float32_t data_matrix_K[32];
float32_t data_matrix_I[16] = {1,0,0,0,
                               0,1,0,0,
                               0,0,1,0,
                               0,0,0,1};
float32_t data_matrix_skewX[12];
float32_t data_skewX_T[12];
float32_t temp_matrix_9[9];
float32_t temp_matrix_12[12];
float32_t temp_matrix_12_1[12];
float32_t temp_matrix_16[16];
float32_t temp_matrix_16_1[16];
float32_t temp_matrix_16_T[16];
float32_t temp_matrix_32[32];
float32_t temp_matrix_32_T[32];
float32_t temp_matrix_64[64];
float32_t temp_value;
float32_t X_norm;
float32_t magnitude;
float32_t dt = 0.005;
uint8_t firstRun = 1; //first run flag;



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
void runKalmanFilter();
void initMatrix();
void getRawCompassData();
void calibrateMag();
void getRawGyroData();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
MPU9250 *myMPU = new MPU9250 (&hi2c1);
/* USER CODE END 0 */

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
  MX_USART3_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  data_matrix_P_prev[0] = 1;
  data_matrix_P_prev[1] = 0;
  data_matrix_P_prev[2] = 0;
  data_matrix_P_prev[3] = 0;
  data_matrix_P_prev[4] = 0;
  data_matrix_P_prev[5] = 1;
  data_matrix_P_prev[6] = 0;
  data_matrix_P_prev[7] = 0;
  data_matrix_P_prev[8] = 0;
  data_matrix_P_prev[9] = 0;
  data_matrix_P_prev[10] = 1;
  data_matrix_P_prev[11] = 0;
  data_matrix_P_prev[12] = 0;
  data_matrix_P_prev[13] = 0;
  data_matrix_P_prev[14] = 0;
  data_matrix_P_prev[15] = 1;
  
  data_matrix_X_prev[0] = 1;
  data_matrix_X_prev[1] = 0;
  data_matrix_X_prev[2] = 0;
  data_matrix_X_prev[3] = 0;
  data_matrix_X[0] = 1;
  data_matrix_X[1] = 0;
  data_matrix_X[2] = 0;
  data_matrix_X[3] = 0;
  
  uint8_t deviceID;
  deviceID = myMPU->getDeviceID();
  if (deviceID == 0x71){
    myMPU->initialize();//initialzie
  }else{
    printf("ID wrong");
    return 0;
  }
  
  printf("Device OK, reading data \r\n");
  
  /* KF step 0-> initialize*/
  initMatrix();
  
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
      }else{
        /*data not ready*/
        printf("Data not Ready \r\n");
      }
    }else{
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
        runKalmanFilter();
      
        /*print the resultant quaternion to serial terminal*/
        printf("Q: %f %f %f %f \r\n",data_matrix_X_prev[0],data_matrix_X_prev[1],data_matrix_X_prev[2],data_matrix_X_prev[3]);      
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
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

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

/* USER CODE BEGIN 4 */
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


void runKalmanFilter(){
      /*step 1-> calculate A ->state transition matrix*/
      /*temp1 = [gyro(1)*pi/180;gyro(2)*pi/180;gyro(3)*pi/180];*/
      for (int i=0;i<3;i++){
        temp_vector[i] = Gxyz[i]*3.1415926/180;
      }
      /*magnitude = norm(temp1);*/
      temp_value = temp_vector[0]*temp_vector[0] + temp_vector[1]*temp_vector[1] + temp_vector[2]*temp_vector[2];
      arm_sqrt_f32(temp_value, &magnitude);
  
      /*  if (magnitude<1e-4)
         magnitude = 0;
         temp1 = zeros(3,1);
      else
         temp1 = temp1/magnitude*sin(magnitude*dt/2);*/
      if(magnitude<0.0001){
        magnitude = 0;
        for(int i=0;i<3;i++){
          temp_vector[i] = 0;
        }
      }else{
        float32_t tmp = arm_sin_f32(magnitude*dt/2);
        for(int i=0;i<3;i++){
          temp_vector[i] = temp_vector[i]/magnitude*tmp;//not tested
        }
      }
      /*a = cos(magnitude/2*dt);*/
      temp_value = arm_cos_f32(magnitude*dt/2);
  
      /*skew = skewSymmetric(a,temp1);*/
      /*function [Matrix]=skewSymmetric(a,X)
        Matrix = [a X(3)*(-1) X(2);X(3) a X(1)*(-1);X(2)*(-1) X(1) a];*/
      temp_matrix_9[0] = temp_value;
      temp_matrix_9[1] = temp_vector[2]*(-1);
      temp_matrix_9[2] = temp_vector[1];
      temp_matrix_9[3] = temp_vector[2];
      temp_matrix_9[4] = temp_value;
      temp_matrix_9[5] = temp_vector[0]*(-1);
      temp_matrix_9[6] = temp_vector[1]*(-1);
      temp_matrix_9[7] = temp_vector[0];
      temp_matrix_9[8] = temp_value;
  
      /*value of A*/
      /*  A_top = [a,(temp1')*(-1)];
          A_btm = [temp1,skew];
          A = [A_top;A_btm];
      */
      data_matrix_A[0] = temp_value;
      data_matrix_A[1] = temp_vector[0]*(-1);
      data_matrix_A[2] = temp_vector[1]*(-1);
      data_matrix_A[3] = temp_vector[2]*(-1);
      data_matrix_A[4] = temp_vector[0];
      data_matrix_A[8] = temp_vector[1];
      data_matrix_A[12] = temp_vector[2];
      data_matrix_A[5] = temp_matrix_9[0];
      data_matrix_A[6] = temp_matrix_9[1];
      data_matrix_A[7] = temp_matrix_9[2];
      data_matrix_A[9] = temp_matrix_9[3];
      data_matrix_A[10] = temp_matrix_9[4];
      data_matrix_A[11] = temp_matrix_9[5];
      data_matrix_A[13] = temp_matrix_9[6];
      data_matrix_A[14] = temp_matrix_9[7];
      data_matrix_A[15] = temp_matrix_9[8];
      
      /*Step 2 ---------------->  predict X */
      /*X = A*X_prev;*/
      arm_mat_mult_f32(&Matrix_A,&Matrix_X_prev,&Matrix_X);
  
      /*Step 3 ----------------> calculate Q*/
      /*skewQ = skewSymmetric(X(1),X(2:4));*/
      temp_matrix_9[0] = data_matrix_X[0];
      temp_matrix_9[1] = data_matrix_X[3]*(-1);
      temp_matrix_9[2] = data_matrix_X[2];
      temp_matrix_9[3] = data_matrix_X[3];
      temp_matrix_9[4] = data_matrix_X[0];
      temp_matrix_9[5] = data_matrix_X[1]*(-1);
      temp_matrix_9[6] = data_matrix_X[2]*(-1);
      temp_matrix_9[7] = data_matrix_X[1];
      temp_matrix_9[8] = data_matrix_X[0];
  
      /*skewX = [X(2)*(-1) X(3)*(-1) X(4)*(-1);skewQ];*/
      data_matrix_skewX[0] = data_matrix_X[1]*(-1);
      data_matrix_skewX[1] = data_matrix_X[2]*(-1);
      data_matrix_skewX[2] = data_matrix_X[3]*(-1);
      data_matrix_skewX[3] = temp_matrix_9[0];
      data_matrix_skewX[4] = temp_matrix_9[1];
      data_matrix_skewX[5] = temp_matrix_9[2];
      data_matrix_skewX[6] = temp_matrix_9[3];
      data_matrix_skewX[7] = temp_matrix_9[4];
      data_matrix_skewX[8] = temp_matrix_9[5];
      data_matrix_skewX[9] = temp_matrix_9[6];
      data_matrix_skewX[10] = temp_matrix_9[7];
      data_matrix_skewX[11] = temp_matrix_9[8];
  
      /*Q = dt*dt/4*skewX*rg*(skewX');*/
      temp_value = dt*dt/4;
      arm_mat_mult_f32(&Matrix_skewX,&Matrix_rg,&Matrix_temp_12_1);
      arm_mat_trans_f32(&Matrix_skewX,&Matrix_skewX_T);
      arm_mat_mult_f32(&Matrix_temp_12_1,&Matrix_skewX_T,&Matrix_Q);
      for(int i=0;i<16;i++){
        data_matrix_Q[i] *= temp_value;
      }
  
      /*Step4 ----------------> predict P */ 
      /*P = A*P_prev*A'+ Q;*/
      if(firstRun){
        temp_matrix_16_1[0] = 1;
        temp_matrix_16_1[5] = 1;
        temp_matrix_16_1[10] = 1;
        temp_matrix_16_1[15] = 1;
      }else{
        arm_mat_trans_f32(&Matrix_A,&Matrix_temp_16_T);
        arm_mat_mult_f32(&Matrix_A,&Matrix_P_prev,&Matrix_temp_16);
        arm_mat_mult_f32(&Matrix_temp_16,&Matrix_temp_16_T,&Matrix_temp_16_1);
      }
      arm_mat_add_f32(&Matrix_temp_16_1,&Matrix_Q,&Matrix_P);
  
      /*Step 5 --------------> Calculate H*/
      /*tmp = Za-G;*/
      for(int i=0;i<3;i++){
        temp_vector[i] = Axyz[i]-gravity[i];
      }
  
      /*Hleft = [0;tmp]; + Hright = [(-1)*tmp';skewH*(-1)];*/
      data_matrix_H[0] = 0;
      data_matrix_H[1] = temp_vector[0]*(-1);
      data_matrix_H[2] = temp_vector[1]*(-1);
      data_matrix_H[3] = temp_vector[2]*(-1);
      data_matrix_H[4] = temp_vector[0];
      data_matrix_H[8] = temp_vector[1];
      data_matrix_H[12] = temp_vector[2];
  
      /*tmp1 = Za+G;*/
      for(int i=0;i<3;i++){
        temp_vector[i] = Axyz[i]+gravity[i];
      }
  
      /*skewH = skewSymmetric(0,tmp1);*/
      temp_matrix_9[0] = 0;
      temp_matrix_9[1] = temp_vector[2]*(-1);
      temp_matrix_9[2] = temp_vector[1];
      temp_matrix_9[3] = temp_vector[2];
      temp_matrix_9[4] = 0;
      temp_matrix_9[5] = temp_vector[0]*(-1);
      temp_matrix_9[6] = temp_vector[1]*(-1);
      temp_matrix_9[7] = temp_vector[0];
      temp_matrix_9[8] = 0;
  
      /*Hright = [(-1)*tmp';skewH*(-1)];*/
      /*Htop = [Hleft,Hright];*/
      data_matrix_H[5] = temp_matrix_9[0]*(-1);
      data_matrix_H[6] = temp_matrix_9[1]*(-1);
      data_matrix_H[7] = temp_matrix_9[2]*(-1);
      data_matrix_H[9] = temp_matrix_9[3]*(-1);
      data_matrix_H[10] = temp_matrix_9[4]*(-1);
      data_matrix_H[11] = temp_matrix_9[5]*(-1);
      data_matrix_H[13] = temp_matrix_9[6]*(-1);
      data_matrix_H[14] = temp_matrix_9[7]*(-1);
      data_matrix_H[15] = temp_matrix_9[8]*(-1);
  
      /*tmp = Zm-M;*/
      for(int i=0;i<3;i++){
        temp_vector[i] = Mxyz[i]-MagConst[i];
      }
  
      /*Hleft = [0;tmp]; + Hright = [(-1)*tmp';skewH*(-1)];*/
      data_matrix_H[16] = 0;
      data_matrix_H[17] = temp_vector[0]*(-1);
      data_matrix_H[18] = temp_vector[1]*(-1);
      data_matrix_H[19] = temp_vector[2]*(-1);
      data_matrix_H[20] = temp_vector[0];
      data_matrix_H[24] = temp_vector[1];
      data_matrix_H[28] = temp_vector[2];
  
      /*tmp1 = Zm+M;*/
      for(int i=0;i<3;i++){
        temp_vector[i] = Mxyz[i]+MagConst[i];
      }
  
      /*skewH = skewSymmetric(0,tmp1);*/
      temp_matrix_9[0] = 0;
      temp_matrix_9[1] = temp_vector[2]*(-1);
      temp_matrix_9[2] = temp_vector[1];
      temp_matrix_9[3] = temp_vector[2];
      temp_matrix_9[4] = 0;
      temp_matrix_9[5] = temp_vector[0]*(-1);
      temp_matrix_9[6] = temp_vector[1]*(-1);
      temp_matrix_9[7] = temp_vector[0];
      temp_matrix_9[8] = 0;
  
      /*Hright = [(-1)*tmp';skewH*(-1)];
      Hbtm = [Hleft,Hright];
      H = [Htop;Hbtm];*/
      data_matrix_H[21] = temp_matrix_9[0]*(-1);
      data_matrix_H[22] = temp_matrix_9[1]*(-1);
      data_matrix_H[23] = temp_matrix_9[2]*(-1);
      data_matrix_H[25] = temp_matrix_9[3]*(-1);
      data_matrix_H[26] = temp_matrix_9[4]*(-1);
      data_matrix_H[27] = temp_matrix_9[5]*(-1);
      data_matrix_H[29] = temp_matrix_9[6]*(-1);
      data_matrix_H[30] = temp_matrix_9[7]*(-1);
      data_matrix_H[31] = temp_matrix_9[8]*(-1);
  
      /*Step 6 --------------> Calculate R*/
      /*Ra = 0.25*skewX*ra*(skewX');*/
      arm_mat_mult_f32(&Matrix_skewX,&Matrix_ra,&Matrix_temp_12_1);
      arm_mat_trans_f32(&Matrix_skewX,&Matrix_skewX_T);
      arm_mat_mult_f32(&Matrix_temp_12_1,&Matrix_skewX_T,&Matrix_temp_16);
      for(int i=0;i<16;i++){
        temp_matrix_16[i] *= 0.25;
      }
  
      /*Rm = 0.25*skewX*rm*(skewX');*/
      arm_mat_mult_f32(&Matrix_skewX,&Matrix_rm,&Matrix_temp_12_1);
      arm_mat_trans_f32(&Matrix_skewX,&Matrix_skewX_T);
      arm_mat_mult_f32(&Matrix_temp_12_1,&Matrix_skewX_T,&Matrix_temp_16_1);
      for(int i=0;i<16;i++){
        temp_matrix_16_1[i] *= 0.25;
      }
  
      /*Rtop = [Ra,zeros(4)];
      Rbtm = [zeros(4),Rm];
      R = [Rtop;Rbtm];*/
      data_matrix_R[0] = temp_matrix_16[0];
      data_matrix_R[1] = temp_matrix_16[1];
      data_matrix_R[2] = temp_matrix_16[2];
      data_matrix_R[3] = temp_matrix_16[3];
      data_matrix_R[8] = temp_matrix_16[4];
      data_matrix_R[9] = temp_matrix_16[5];
      data_matrix_R[10] = temp_matrix_16[6];
      data_matrix_R[11] = temp_matrix_16[7];
      data_matrix_R[16] = temp_matrix_16[8];
      data_matrix_R[17] = temp_matrix_16[9];
      data_matrix_R[18] = temp_matrix_16[10];
      data_matrix_R[19] = temp_matrix_16[11];
      data_matrix_R[24] = temp_matrix_16[12];
      data_matrix_R[25] = temp_matrix_16[13];
      data_matrix_R[26] = temp_matrix_16[14];
      data_matrix_R[27] = temp_matrix_16[15];
  
      data_matrix_R[36] = temp_matrix_16_1[0];
      data_matrix_R[37] = temp_matrix_16_1[1];
      data_matrix_R[38] = temp_matrix_16_1[2];
      data_matrix_R[39] = temp_matrix_16_1[3];
      data_matrix_R[44] = temp_matrix_16_1[4];
      data_matrix_R[45] = temp_matrix_16_1[5];
      data_matrix_R[46] = temp_matrix_16_1[6];
      data_matrix_R[47] = temp_matrix_16_1[7];
      data_matrix_R[52] = temp_matrix_16_1[8];
      data_matrix_R[53] = temp_matrix_16_1[9];
      data_matrix_R[54] = temp_matrix_16_1[10];
      data_matrix_R[55] = temp_matrix_16_1[11];
      data_matrix_R[60] = temp_matrix_16_1[12];
      data_matrix_R[61] = temp_matrix_16_1[13];
      data_matrix_R[62] = temp_matrix_16_1[14];
      data_matrix_R[63] = temp_matrix_16_1[15];
  
      //the other values should be 0 as initialized.
  
      /*Step 7 -------------->  update */
      /*S = H*P*H'+R;*/
      arm_mat_trans_f32(&Matrix_H,&Matrix_H_T);
      arm_mat_mult_f32(&Matrix_H,&Matrix_P,&Matrix_temp_32);
      arm_mat_mult_f32(&Matrix_temp_32,&Matrix_H_T,&Matrix_temp_64);
      if (firstRun){
        temp_matrix_64[3] = 0;
        temp_matrix_64[10] = 0;
        temp_matrix_64[17] = 0;
        temp_matrix_64[24] = 0;
        temp_matrix_64[39] = 0;
        temp_matrix_64[46] = 0;
        temp_matrix_64[53] = 0;
        temp_matrix_64[60] = 0;
      }
      arm_mat_add_f32(&Matrix_temp_64,&Matrix_R,&Matrix_S);
  
      /*K = (P*H')/S;*/  
      arm_mat_inverse_f32(&Matrix_S,&Matrix_temp_64);
      arm_mat_mult_f32(&Matrix_P,&Matrix_H_T,&Matrix_temp_32_T);
      arm_mat_mult_f32(&Matrix_temp_32_T,&Matrix_temp_64,&Matrix_K);
  
      /*X_updated = (I-K*H)*X;*/
      arm_mat_mult_f32(&Matrix_K,&Matrix_H,&Matrix_temp_16);
      arm_mat_sub_f32(&Matrix_I,&Matrix_temp_16,&Matrix_temp_16_1);
      arm_mat_mult_f32(&Matrix_temp_16_1,&Matrix_X,&Matrix_X_prev);
  
      /*X_updated = X_updated/norm(X_updated);*/
      temp_value = data_matrix_X_prev[0]*data_matrix_X_prev[0] + data_matrix_X_prev[1]*data_matrix_X_prev[1] + data_matrix_X_prev[2]*data_matrix_X_prev[2] 
               + data_matrix_X_prev[3]*data_matrix_X_prev[3];
      arm_sqrt_f32(temp_value, &X_norm);
      for (int i=0;i<4;i++){
        data_matrix_X_prev[i] = data_matrix_X_prev[i]/X_norm;
      }
  
      /*P_updated = (I-K*H)*P;*/
  
      arm_mat_mult_f32(&Matrix_temp_16_1,&Matrix_P,&Matrix_P_prev);
  
      /*arm_mat_trans_f32(&Matrix_temp_16_1,&Matrix_temp_16_T);
      arm_mat_mult_f32(&Matrix_temp_16_1,&Matrix_P,&Matrix_temp_16);
      arm_mat_mult_f32(&Matrix_temp_16,&Matrix_temp_16_T,&Matrix_temp_16_1);
      arm_mat_trans_f32(&Matrix_K,&Matrix_temp_32);
      arm_mat_mult_f32(&Matrix_K,&Matrix_R,&Matrix_temp_32_T);
      arm_mat_mult_f32(&Matrix_temp_32_T,&Matrix_temp_32,&Matrix_temp_16);
      arm_mat_add_f32(&Matrix_temp_16_1,&Matrix_temp_16,&Matrix_P_prev);*/
      
      if(firstRun){
        firstRun = 0;
      }
}

void initMatrix(){
  arm_mat_init_f32(&Matrix_X,4,1,(float32_t*)data_matrix_X);
  arm_mat_init_f32(&Matrix_X_prev,4,1,(float32_t*)data_matrix_X_prev);
  arm_mat_init_f32(&Matrix_A,4,4,(float32_t*)data_matrix_A);
  arm_mat_init_f32(&Matrix_Q,4,4,(float32_t*)data_matrix_Q);
  arm_mat_init_f32(&Matrix_P,4,4,(float32_t*)data_matrix_P);
  arm_mat_init_f32(&Matrix_P_prev,4,4,(float32_t*)data_matrix_P_prev);
  arm_mat_init_f32(&Matrix_ra,3,3,(float32_t*)data_matrix_ra);
  arm_mat_init_f32(&Matrix_rm,3,3,(float32_t*)data_matrix_rm);
  arm_mat_init_f32(&Matrix_rg,3,3,(float32_t*)data_matrix_rg);
  arm_mat_init_f32(&Matrix_R,8,8,(float32_t*)data_matrix_R);
  arm_mat_init_f32(&Matrix_S,8,8,(float32_t*)data_matrix_S);
  arm_mat_init_f32(&Matrix_H,8,4,(float32_t*)data_matrix_H);
  arm_mat_init_f32(&Matrix_H_T,4,8,(float32_t*)data_H_T);
  arm_mat_init_f32(&Matrix_K,4,8,(float32_t*)data_matrix_K);
  arm_mat_init_f32(&Matrix_I,4,4,(float32_t*)data_matrix_I);
  arm_mat_init_f32(&Matrix_skewX,4,3,(float32_t*)data_matrix_skewX);
  arm_mat_init_f32(&Matrix_skewX_T,3,4,(float32_t*)data_skewX_T); 
  arm_mat_init_f32(&Matrix_temp_12,4,3,(float32_t*)temp_matrix_12);
  arm_mat_init_f32(&Matrix_temp_12_1,4,3,(float32_t*)temp_matrix_12_1);
  arm_mat_init_f32(&Matrix_temp_16,4,4,(float32_t*)temp_matrix_16);
  arm_mat_init_f32(&Matrix_temp_16_1,4,4,(float32_t*)temp_matrix_16_1);
  arm_mat_init_f32(&Matrix_temp_16_T,4,4,(float32_t*)temp_matrix_16_T);
  arm_mat_init_f32(&Matrix_temp_32,8,4,(float32_t*)temp_matrix_32); //8x4 matrix
  arm_mat_init_f32(&Matrix_temp_32_T,4,8,(float32_t*)temp_matrix_32_T); //4x8 matrix
  arm_mat_init_f32(&Matrix_temp_64,8,8,(float32_t*)temp_matrix_64);
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

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
