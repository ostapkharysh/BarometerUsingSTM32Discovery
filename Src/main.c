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





 int temperature;
 int B5 = 0;

  //uint32_t pressure;

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

  int CalculateTempBAR(int data, int AC6, int AC5, int MC, int MD){
	  B5 = (data-AC6)*(AC5/32768) + MC*2048/(data-AC6)*(AC5/32768)+MD;
	 return ((data-AC6)*(AC5/32768) + MC*2048/(data-AC6)*(AC5/32768)+MD + 8)/16;
  	  }

void FullfillDataArrays(){
  int count = 0;
  for (int i = 0; i < 11; i ++) {
	HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t) barRegisters[i], I2C_MEMADD_SIZE_8BIT, Buffer, 2, 500);
	if(i == 3 || i == 4 || i == 5){
		 dataUINTHolders[count] = (Buffer[0]<<8) | (Buffer[1]);
		 printf("%d \n",dataUINTHolders[count]);
		 count++;
	}
	else{
		 dataINTHolders[i] = (Buffer[0]<<8) | (Buffer[1]);
		 printf("%d \n",dataINTHolders[i]);
	}
}

}

/*
HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xAC), I2C_MEMADD_SIZE_8BIT, Buffer1, 2, 500);
	  AC2 = (Buffer1[0]<<8) | (Buffer1[1]);
	  printf("%d \n", AC2);

HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xAE), I2C_MEMADD_SIZE_8BIT, Buffer1, 2, 500);
	  AC3 = (Buffer1[0]<<8) | (Buffer1[1]);
	  printf("%d \n", AC3);

HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xB0), I2C_MEMADD_SIZE_8BIT, Buffer1, 2, 500);
	  AC4 = (Buffer1[0]<<8) | (Buffer1[1]);
	  printf("%d \n", AC4);

HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xB2), I2C_MEMADD_SIZE_8BIT, Buffer1, 2, 500);
	  AC5 = (Buffer1[0]<<8) | (Buffer1[1]);
	  printf("%d \n", AC5);

HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xB4), I2C_MEMADD_SIZE_8BIT, Buffer1, 2, 500);
	  AC6 = (Buffer1[0]<<8) | (Buffer1[1]);
	  printf("%d \n", AC6);

HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xB6), I2C_MEMADD_SIZE_8BIT, Buffer1, 2, 500);
	  B1 = (Buffer1[0]<<8) | (Buffer1[1]);
	  printf("%d \n", B1);

HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xB8), I2C_MEMADD_SIZE_8BIT, Buffer1, 2, 500);
	  B2 = (Buffer1[0]<<8) | (Buffer1[1]);
	  printf("%d \n", B2);

HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xBa), I2C_MEMADD_SIZE_8BIT, Buffer1, 2, 500);
	  MB = (Buffer1[0]<<8) | (Buffer1[1]);
	  printf("%d \n", MB);

HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xBC), I2C_MEMADD_SIZE_8BIT, Buffer1, 2, 500);
	  MC = (Buffer1[0]<<8) | (Buffer1[1]);
	  printf("%d \n", MC);

HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xBE), I2C_MEMADD_SIZE_8BIT, Buffer1, 2, 500);
	  MD = (Buffer1[0]<<8) | (Buffer1[1]);
	  printf("%d \n", MD);
*/

 FullfillDataArrays(); // fills the arrays data for temperature and pressure manipulations
 printf("ALLL STABLE DATA ENDED");





  while (1)
  {
	  Buffer[0] = (uint16_t) 0x2E;
	  HAL_I2C_Mem_Write(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xF4), I2C_MEMADD_SIZE_8BIT, Buffer, 1, 500);
	  HAL_Delay(450);
	  HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xF6),  I2C_MEMADD_SIZE_8BIT, Buffer, 2, 500);
	  	  temperature = (Buffer[0]<<8) | (Buffer[1]);
	  	  printf("%d \n", temperature);
	  	  printf("%d \n", CalculateTempBAR(temperature, dataUINTHolders[2], dataUINTHolders[1], dataINTHolders[9], dataINTHolders[10]));
	  	  /*
	  Buffer1[0] =  (uint16_t) 0x34;
	  HAL_I2C_Mem_Write(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xF4), I2C_MEMADD_SIZE_8BIT, Buffer1, 1, 500);
	  HAL_Delay(450);
      HAL_I2C_Mem_Read(&hi2c2, (uint16_t) (0xEE), (uint16_t)(0xF6), I2C_MEMADD_SIZE_8BIT, Buffer1, 2, 500);
		  pressure = (Buffer1[0]<<8) | (Buffer1[1]);
		  printf("%d \n", pressure);

		*/

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
