/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "bmp280.h"
#include "bmp280_defs.h"
#include "lcd_config.h"
#include "States.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BMP280_SPI (&hspi4)
#define BMP280_CS1 1
//#define BMP280_CS1 2
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int8_t rslt;
int32_t temp32;
double temp;

TIM_HandleTypeDef htim2;

double BMP280_temp;
float swv_temp;
char buffer[100];
uint32_t enc_count = 24;
int ENCODER; //Encoder Status

////////TEMPERATURE VARIABLES ////

uint8_t error;
double Temp_Ref = 24;
double TempRecived;
double TempRecived2;

double tmpTemp; //Temp Temperature

uint8_t Received[4];
//static unsigned short heater_state,fan_state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */



int8_t bmp280_spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t length);
int8_t bmp280_spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t length);


/**
 *
 * @brief HAL_UART_RxCpltCallback it's job is to return information
 * if data sent via UART is incorrect,
 * or to set received data as TempRef if data is correct.
 * Also at the same time it's shows information on second line of LCD.
 * To make it easier for user to notice we put LCD screen to sleep for 1s,
 * it doesn't change anything, because with current parts even if we take
 * each read from sensor in 1ms, any change will be noticeable after 1s.
 *
 * @param huart it's a UART3 handler
 */

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{

	if(Received[0] == 'E' && Received[2] == 'N'){
		HAL_GPIO_WritePin(ENCLED_GPIO_Port, ENCLED_Pin, 1);
		ENCODER = 1;
	}
	else if (Received[0] == 'E' && Received[2] == 'F'){
		HAL_GPIO_WritePin(ENCLED_GPIO_Port, ENCLED_Pin, 0);
		ENCODER = 0;
	}
	if(ENCODER == 0){

		TempRecived2=atoi(Received);
		TempRecived = TempRecived2/100;

		if (TempRecived < 21.0)
		{
			error = sprintf(buffer,"Incorrect value\n\r ");
			HAL_UART_Transmit(&huart3, (uint8_t*)buffer, error, 200);
			LCD_SetCursor(&hlcd1, 1, 0);
			LCD_printf(&hlcd1, "Incorrect value");
			lcd_delay_us(&hlcd1, 1000000);
			//		LCD_Clear(&hlcd1);

		}
		else if (TempRecived > 30.0)
		{
			error = sprintf(buffer,"Incorrect value\n\r ");
			HAL_UART_Transmit(&huart3, (uint8_t*)buffer, error, 200);
			LCD_SetCursor(&hlcd1, 1, 0);
			LCD_printf(&hlcd1, "Incorrect value");
			lcd_delay_us(&hlcd1, 1000000);
			//		LCD_Clear(&hlcd1);
		}
		else
		{
			Temp_Ref = TempRecived;
			LCD_SetCursor(&hlcd1, 1, 0);
			LCD_printf(&hlcd1, "Set %4.2f[degC]", Temp_Ref);
			lcd_delay_us(&hlcd1, 1000000);
		}
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{

		swv_temp = (float)temp;
		BMP280_temp = sprintf(buffer, "BMP280 Temp: %.2f [degC] \n\r", temp);  // Do zrobienia na floaty
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, BMP280_temp, 950);

		LCD_SetCursor(&hlcd1, 0, 0);
		LCD_printf(&hlcd1, "Temp %4.2f[degC]" , temp);
	}
	if (htim->Instance == TIM2)
		{
			LCD_SetCursor(&hlcd1, 1, 0);
			LCD_printf(&hlcd1, "Set %4.2f[degC]", Temp_Ref);
		}

}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct bmp280_dev bmp280_1 = {
		.dev_id = BMP280_CS1,
		.intf = BMP280_SPI_INTF,
		.read = bmp280_spi_reg_read,
		.write = bmp280_spi_reg_write,
		.delay_ms = HAL_Delay
};
struct bmp280_uncomp_data bmp280_1_data;


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
  MX_USART3_UART_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

	struct bmp280_config conf;
	rslt = bmp280_init(&bmp280_1);

	 /* Always read the current settings before writing, especially when
	  * all the configuration is not modified
	  */

	rslt = bmp280_get_config(&conf, &bmp280_1);
	conf.filter = BMP280_FILTER_OFF;
	/** Temperature oversampling set at 1x */
	conf.os_temp = BMP280_OS_1X;
	conf.os_pres = BMP280_OS_1X;
	/** Setting the output data rate as 1HZ(1000ms) */
	conf.odr = BMP280_ODR_1000_MS;
	rslt = bmp280_set_config(&conf, &bmp280_1);
	/**Always set the power mode after setting the configuration */
	rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp280_1);

	/** Timers and PWM declaration*/
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	LCD_Init(&hlcd1);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/** Reading the raw data from sensor */
		rslt = bmp280_get_uncomp_data(&bmp280_1_data, &bmp280_1);
		rslt = bmp280_get_comp_temp_32bit(&temp32, bmp280_1_data.uncomp_temp, &bmp280_1);
		 /**Getting the compensated temperature as floating point value */
		rslt = bmp280_get_comp_temp_double(&temp, bmp280_1_data.uncomp_temp, &bmp280_1);
		   /**Sleep time between measurements = BMP280_ODR_1000_MS */
		bmp280_1.delay_ms(1000);


			/////STATES RUN AND STATE_CHANGE///
		States_Run();
		State_Change(Temp_Ref,temp);

		/** Receiving Referenced temperature*/
		HAL_UART_Receive_IT(&huart3, Received, 4);

		if(ENCODER == 1){
			enc_count = __HAL_TIM_GET_COUNTER(&htim8) + 88;

			if(enc_count>21*4 && enc_count<30*4){
				tmpTemp = (double)enc_count/4;
			}
			else{
				enc_count = 88;
				tmpTemp = 22;
			}

			if(tmpTemp>21 && tmpTemp<30){
				Temp_Ref = tmpTemp;
			}
			HAL_Delay(5);
		}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

#define BMP280_SPI_BUFFER_LEN 28
#define BMP280_DATA_INDEX 1
/*!
 * @brief Function for writing the sensor 's registers through SPI bus.
 *
 * @param [in] cs : Chip select to enable the sensor .
 * @param [in] reg_addr : Register address .
 * @param [in] reg_data : Pointer to the data buffer whose data has to be
written
 * @param [in] length : No of bytes to write .
 *
 * @return Status of execution
 * @retval 0 -> Success
 * @retval >0 -> Failure Info
 *
 */
int8_t bmp280_spi_reg_write (uint8_t cs, uint8_t reg_addr, uint8_t
		*reg_data, uint16_t length)
{
	/* Implement the SPI write routine according to the target machine . */
	HAL_StatusTypeDef status = HAL_OK ;
	int32_t iError = BMP280_OK;
	uint8_t txarray[BMP280_SPI_BUFFER_LEN];
	uint8_t stringpos;
	/* Copy register address and data to tx buffer */
	txarray[0] = reg_addr;
	for (stringpos = 0; stringpos < length; stringpos++)
	{
		txarray[stringpos + BMP280_DATA_INDEX] = reg_data[stringpos];
	}
	// memcpy ( txarray + BMP280_DATA_INDEX , reg_data , length );
	/* Software slave selection procedure */
	if(cs == BMP280_CS1)
		HAL_GPIO_WritePin(BMP280_CS1_GPIO_Port, BMP280_CS1_Pin, GPIO_PIN_RESET);
	/* Data exchange */
	status = HAL_SPI_Transmit(BMP280_SPI, (uint8_t*)(&txarray), length+1, 100);
	while (BMP280_SPI->State == HAL_SPI_STATE_BUSY) {};
	/* Disable all slaves */
	HAL_GPIO_WritePin(BMP280_CS1_GPIO_Port, BMP280_CS1_Pin, GPIO_PIN_SET);
	if (status != HAL_OK)
	{
		// The BMP280 API calls for 0 return value as a success , and -1 returned as
		iError = (-1);
	}
	return (int8_t)iError;
}

/*!
 * @brief Function for reading the sensor 's registers through SPI bus.
 *
 * @param [in] cs : Chip select to enable the sensor .
 * @param [in] reg_addr : Register address .
 * @param [ out] reg_data : Pointer to the data buffer to store the read
data .
 * @param [in] length : No of bytes to read .
 *
 * @return Status of execution
 * @retval 0 -> Success
 * @retval >0 -> Failure Info
 *
 */
int8_t bmp280_spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t length)
{
	/* Implement the SPI read routine according to the target machine . */
	HAL_StatusTypeDef status = HAL_OK;
	int32_t iError = BMP280_OK;
	uint8_t txarray [BMP280_SPI_BUFFER_LEN] = {0 ,};
	uint8_t rxarray [BMP280_SPI_BUFFER_LEN] = {0 ,};
	uint8_t stringpos;
	txarray[0] = reg_addr;
	/* Software slave selection procedure */
	if(cs == BMP280_CS1)
		HAL_GPIO_WritePin(BMP280_CS1_GPIO_Port, BMP280_CS1_Pin, GPIO_PIN_RESET);
	/* Data exchange */
	status = HAL_SPI_TransmitReceive (BMP280_SPI, (uint8_t*) (&txarray),
			(uint8_t*)(&rxarray), length+1, 5);
	while (BMP280_SPI->State == HAL_SPI_STATE_BUSY) {};
	/* Disable all slaves */
	HAL_GPIO_WritePin(BMP280_CS1_GPIO_Port, BMP280_CS1_Pin, GPIO_PIN_SET);
	/* Copy data from rx buffer */
	for (stringpos = 0; stringpos < length ; stringpos++)
	{
		reg_data[stringpos] = rxarray[stringpos + BMP280_DATA_INDEX];
	}
	// memcpy ( reg_data , rxarray + BMP280_DATA_INDEX , length );
	if (status != HAL_OK)
	{
		// The BME280 API calls for 0 return value as a success , and -1 returned

		iError = (-1);
	}
	return (int8_t)iError;
}

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
