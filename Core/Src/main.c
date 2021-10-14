/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_STORAGE 0x08005000
#define page_size 0x400 //page size for STM32F103RB is 1KB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void flashSave(uint8_t* data)
{
	//normal stores the number of complete 4 bits in the total size
	// excess is used account for the excess bits that come other than the perfect bit.
	//if such an excess bit comes ,then an extra 4bit memory is allocated to the data in FLASH
	//sum of normal and excess makes up the total required size of data to be copied to FLASH (in 4 bits)

	uint8_t normal;
	normal= strlen((char*)data)/4;
	int excess=strlen((char *)data)%4?1:0;

	volatile uint32_t flashData[normal+excess];
	memset((uint8_t*)flashData,0,strlen((char*)flashData));
	strcpy((char*)flashData,(char*)data);

	int normal1=strlen((char*)flashData)/4;
	int excess1=strlen((char *)flashData)%4?1:0;
	volatile uint32_t data_length= normal1+excess1;

	int normal2=strlen((char*)data)/page_size;
	int excess2=strlen((char *)data)%page_size?1:0;
	volatile uint16_t pages= normal2+ excess2;
//	volatile uint32_t flashData[(strlen((char*)data)/4)	+ (int)((strlen((char*)data) % 4) != 0)];
//		memset((uint8_t*)flashData, 0, strlen((char*)flashData));
//		strcpy((char*)flashData, (char*)data);
//
//		volatile uint32_t data_length = (strlen((char*)flashData) / 4)
//										+ (int)((strlen((char*)flashData) % 4) != 0);
//		volatile uint16_t pages = (strlen((char*)data)/page_size)
//										+ (int)((strlen((char*)data)%page_size) != 0);

	/* ******* Unlock the FLash to enable the flash control register access ********/
	HAL_FLASH_Unlock();

	/* Allow Access to option bytes sector */
	HAL_FLASH_OB_Unlock();

	/*Fill EraseInit structure*/
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_STORAGE;
	EraseInitStruct.NbPages = pages;
	uint32_t PageError;

	volatile uint32_t counter_w=0, index=0;

	/* Erases the memory*/
	volatile HAL_StatusTypeDef status;
	status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

	/* Loop to write the data to the flash*/
	while(index <data_length)
	{
		if(status==HAL_OK)
		{
			status =HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_STORAGE+counter_w, flashData[index]);
			if(status==HAL_OK)
			{
				counter_w+=4;
				index++;
			}
		}
	}

	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();

}

void flashRead(uint8_t* data)
{
	volatile uint32_t read_data;
	volatile uint32_t counter_r=0;
	while(read_data!=0xFFFFFFFF)
	{
		// 32 bit data being split into 8 bit to read
		read_data = *(uint32_t*)(FLASH_STORAGE + counter_r);
		if(read_data != 0xFFFFFFFF)
		{
			data[counter_r] = (uint8_t)read_data;
			data[counter_r + 1] = (uint8_t)(read_data >> 8);
			data[counter_r + 2] = (uint8_t)(read_data >> 16);
			data[counter_r + 3] = (uint8_t)(read_data >> 24);
			counter_r += 4;
		}
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  char data1[50];     /* data1 is for writing the data into  flash memory*/
  memset(data1,0,sizeof(data1));
  strcpy(data1,"Hello Abel!");

  flashSave((uint8_t*)write_data); // 8-bit pointer to point to the address

  char data2[50];  /* data2 is for reading the data from flash memory*/
  memset(data2,0,sizeof(data2));

  flashRead((uint8_t*)data2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
