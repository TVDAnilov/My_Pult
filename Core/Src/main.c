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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define max_adc 4095

#define adress_this_device 0x20
#define adress_pult 0x20
#define adress_uaz 0x40
#define adress_ship 0x60
#define adress_WG_golf 0x80



#define ADC 100


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t transmitBuff [50] = {};
volatile uint8_t reciveBuff [50] = {};
volatile uint16_t SticPosADC[4] = {};
static uint8_t position[4] = { };  //Не спутать с 4 осями джойстка!
//position [0] - лев. стик, вертикаль,
//[1] - направление движенияпо вертикали, 0 стоим на месте, 1- вперед,  2 назад.
//[2] - правый стик, горизонталь.
//[3] - направление поворота руля, 0 среднее положение (прямо), 1 направо, 2 налево.

struct {
	volatile uint8_t timerEvent;
	volatile uint8_t adcDone;

}flagEvent;

void initPult(void) {
	flagEvent.timerEvent = 0;
	flagEvent.adcDone = 0;

	HAL_ADCEx_Calibration_Start(&hadc1);			// калибровка ацп

	__HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF); 		// очищаем флаг
	HAL_TIM_Base_Start_IT(&htim3); 					//Включаем прерывание

}

void updateTimerEvent(void) {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) SticPosADC, 4);
}

void normalizeSticValue(void) {

	/////////////////////////////////////////////////////////////////////// Вперед
	if (SticPosADC[1] < 700) {
		position[0] = 100;
		position[1] = 1;
	}
	if (SticPosADC[1] < 1400 && SticPosADC[1] > 700) {
		position[0] = 65;
		position[1] = 1;
	}

	if (SticPosADC[1] < 1800 && SticPosADC[1] > 1400) {
		position[0] = 30;
		position[1] = 1;
	}

	////////////////////////////////////////////////////////////////////////////////// назад

	if (SticPosADC[1] > 2200) {
		position[0] = 20;
		position[1] = 2;
	}

	/////////////////////////////////////////////////////////////////////////////////////// на месте

	if (SticPosADC[1] > 1800 && SticPosADC[1] < 2200) {
		position[0] = 0;
		position[1] = 0;
	}
	///////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////// Направо
	if (SticPosADC[2] < 700) {
		position[2] = 75;
		position[3] = 1;
	}
	if (SticPosADC[2] < 1400 && SticPosADC[2] > 700) {
		position[2] = 50;
		position[3] = 1;
	}

	if (SticPosADC[2] < 1800 && SticPosADC[2] > 1400) {
		position[2] = 20;
		position[3] = 1;
	}

	////////////////////////////////////////////////////////////////////////////////// НАлево

	if (SticPosADC[2] > 2200 && SticPosADC[2] < 3100) {
		position[2] = 20;
		position[3] = 2;
	}

	if (SticPosADC[2] > 3100 && SticPosADC[2] < 3800) {
		position[2] = 65;
		position[3] = 2;
	}

	if (SticPosADC[2] > 3800) {
		position[2] = 100;
		position[3] = 2;
	}

	/////////////////////////////////////////////////////////////////////////////////////// на месте

	if (SticPosADC[2] > 1800 && SticPosADC[2] < 2200) {
		position[2] = 0;
		position[3] = 0;
	}
	///////////////////////////////////////////////////////////////////////////////

}

void pushArrTX(uint8_t addres_module, uint16_t *pData, uint8_t size){
	//
}





void pushArrRX(uint8_t name_device, uint16_t *pData, uint8_t size){
	//
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	initPult();






	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (flagEvent.timerEvent == 1) {
			flagEvent.timerEvent = 0;
			updateTimerEvent();
		}

		if(flagEvent.adcDone == 1){
			flagEvent.adcDone = 0;
			pushArrTX(ADC, SticPosADC, sizeof(SticPosADC));
		}




		//normalizeSticValue();
		//HAL_UART_Transmit(&huart1, position, 4, HAL_MAX_DELAY);
		//HAL_Delay(100);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
//==============================================================================================================
void ADC1_IRQHandler(void) {
	/* USER CODE BEGIN ADC1_2_IRQn 0 */

	/* USER CODE END ADC1_2_IRQn 0 */
	HAL_ADC_IRQHandler(&hadc1);
	/* USER CODE BEGIN ADC1_2_IRQn 1 */

	/* USER CODE END ADC1_2_IRQn 1 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		flagEvent.timerEvent = 1;

	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	flagEvent.adcDone = 1;
}

//==============================================================================================================
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
