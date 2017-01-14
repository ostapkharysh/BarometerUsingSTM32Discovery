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
#include "stm32f3xx_hal.h"
#include "i2c.h"
#include "gpio.h"


/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */






  /* BMP180 Pressure calculation constants */
  #define BMP180_PARAM_MG                 3038
  #define BMP180_PARAM_MH                -7357
  #define BMP180_PARAM_MI                 3791
  #define BMP180_PARAM_BA                 407
  #define BMP180_PARAM_BP                 0.00750061683;


  uint16_t temperature;
  uint16_t pressure;


  uint8_t Buffer[8];

  int16_t AC1=0;
  int16_t AC2=0;
  int16_t AC3=0;
  uint16_t AC4=0;
  uint16_t AC5=0;
  uint16_t AC6=0;
  int16_t B1=0;
  int16_t B2=0;
  int16_t MB=0;
  int16_t MC=0;
  int16_t MD=0;

  int barRegisters[11] = {0xAA, 0xAC, 0xAE, 0xB0, 0xB2, 0xB4, 0xB6, 0xB8, 0xBa, 0xBC, 0xBE};
  int16_t dataINTHolders[11] = {AC1, AC2, AC3, 1, 1, 1, B1, B2, MB, MC, MD}; // 1 1 1  take the place for other data type elements
  uint16_t dataUINTHolders[3] = {AC4, AC5, AC6};


  int CalculateTempBAR(uint16_t data, uint16_t AC6, uint16_t AC5, int16_t MC, int16_t MD){
	  uint16_t B5;
	  B5 = ((data - AC6)*AC5/32768) + MC*2048/((data - AC6)*AC5/32768) + MD;
  	 return B5;
    	  }
  int CalculateUserTempBAR(uint16_t data, uint16_t AC6, uint16_t AC5, int16_t MC, int16_t MD){
	  return (CalculateTempBAR(data, AC6, AC5, MC, MD) + 8)/16 - 100;
  }


  int32_t CalculatePressBAR(uint32_t data, uint32_t AC6, uint32_t AC5, int32_t MC, int32_t MD, int32_t B2, uint8_t oss){
	  int32_t B3, B6, X3, p, X1, X2;
	  uint32_t B4, B7, B5;
	  B5 = CalculateTempBAR(data, AC6, AC5, MC, MD);
	  B6 = B5 - 4000;
	  X3 = (((int32_t) B2 *((B6 * B6) >> 12))>> 11) +((AC2 * B6) >> 11);
	  B3 = ((((int32_t)AC1 * 4 + X3) << oss) + 2) >> 2;
	  X3 = (((AC3 * B6) >> 13) + ((B1 * ((B6 * B6) >> 12)) >> 16) + 2) >> 2;
	  B4 = (AC4 * (uint32_t)(X3 + 32768)) >> 15;
	  B7 = ((uint32_t)data - B3) * (50000 >> oss);
	  if (B7 < 0x80000000) p = (B7 << 1) / B4; else p = (B7 / B4) << 1;
		p += ((((p >> 8) * (p >> 8) * BMP180_PARAM_MG) >> 16) + ((BMP180_PARAM_MH * p) >> 16) + BMP180_PARAM_MI) >> 4;

		return p * BMP180_PARAM_BA * BMP180_PARAM_BP;
	}


void FullfillDataArrays(){
  int count = 0;
  for (int i = 0; i < 11; i ++) {
	HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t) barRegisters[i], I2C_MEMADD_SIZE_8BIT, Buffer, 2, 500);
	if(i == 3 || i == 4 || i == 5){
		 dataUINTHolders[count] = (Buffer[0]<<8) | (Buffer[1]);
		 //printf("%d \n",dataUINTHolders[count]);
		 count++;
	}
	else{
		 dataINTHolders[i] = (Buffer[0]<<8) | (Buffer[1]);
		 //printf("%d \n",dataINTHolders[i]);
	}
 }

}

 FullfillDataArrays(); // fills the arrays data for temperature and pressure manipulations

  while (1)
  {
	  Buffer[0] = (uint16_t) 0x2E;
	  HAL_I2C_Mem_Write(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xF4), I2C_MEMADD_SIZE_8BIT, Buffer, 1, 500);
	  HAL_Delay(450);
	  HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xF6),  I2C_MEMADD_SIZE_8BIT, Buffer, 2, 500);
	  	  temperature = (Buffer[0]<<8) | (Buffer[1]);
	  	  //printf("%d \n", temperature);
	  	  printf("%d \n", CalculateUserTempBAR(temperature, dataUINTHolders[2], dataUINTHolders[1], dataINTHolders[9], dataINTHolders[10]));
	  	  printf("%d \n", CalculatePressBAR(temperature, dataUINTHolders[2], dataUINTHolders[1], dataINTHolders[9], dataINTHolders[10], dataINTHolders[7], 0));


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

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
