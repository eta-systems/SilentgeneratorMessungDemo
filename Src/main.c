/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include "max11615.h"
#include "max7313.h"
#include "silentgenerator.h"
#include "string.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t ADC_Buf[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define MAX7313_1  0x42        // 100 0010
#define MAX7313_2  0x44        // 100 0100
#define MAX11615_1 0x66

MAX11615 adcDriver_1;

MAX7313 ioDriver_1;
MAX7313 ioDriver_2;

MAX7313* ioDrivers[3];

volatile uint8_t interrupt_val = 0;
volatile uint8_t led_mode = 0;
volatile uint8_t STATE_INITIALIZED = 0;
uint8_t laufvariable = 0;

volatile SGButtons button_states_old, button_states_new;

const uint8_t MAX7313_LED_RED_Ports[4] = {
	PORT_LED_RED_1,
	PORT_LED_RED_2,
	PORT_LED_RED_3,
	PORT_LED_RED_4
};

const uint8_t MAX7313_LED_GRN_Ports[5] = {
	PORT_LED_GRN_1,
	PORT_LED_GRN_2,
	PORT_LED_GRN_3,
	PORT_LED_GRN_4,
	PORT_LED_GRN_5
};

const uint8_t MAX7313_LED_RED_Chips[4] = {
	CHIP_LED_RED_1,
	CHIP_LED_RED_2,
	CHIP_LED_RED_3,
	CHIP_LED_RED_4
};

const uint8_t MAX7313_LED_GRN_Chips[5] = {
	CHIP_LED_GRN_1,
	CHIP_LED_GRN_2,
	CHIP_LED_GRN_3,
	CHIP_LED_GRN_4,
	CHIP_LED_GRN_5
};

const uint8_t MAX7313_Ports[12] = {
	PORT_LED_METER_0,
	PORT_LED_METER_10,
	PORT_LED_METER_20,
	PORT_LED_METER_30,
	PORT_LED_METER_40,
	PORT_LED_METER_50,
	PORT_LED_METER_60,
	PORT_LED_METER_70,
	PORT_LED_METER_80,
	PORT_LED_METER_90,
	PORT_LED_METER_100,
	PORT_LED_METER_110
};

const uint8_t MAX7313_Chips[12] = {
	CHIP_LED_METER_0,
	CHIP_LED_METER_10,
	CHIP_LED_METER_20,
	CHIP_LED_METER_30,
	CHIP_LED_METER_40,
	CHIP_LED_METER_50,
	CHIP_LED_METER_60,
	CHIP_LED_METER_70,
	CHIP_LED_METER_80,
	CHIP_LED_METER_90,
	CHIP_LED_METER_100,
	CHIP_LED_METER_110
};

/* STATE MACHINE Begin */
typedef enum {St_SG_Off, St_SG_On, N_States} state_t;
typedef state_t state_func_t(void);

// OFF
state_t state_func_sg_off(void){
	if( button_states_new.SW5 > button_states_old.SW5 ){   // button pressed
		printf("turning ON\n");
		HAL_GPIO_WritePin( SG_USBPowerOn_GPIO_Port, SG_USBPowerOn_Pin, GPIO_PIN_SET );
		HAL_GPIO_WritePin( SG_5V_Power_On_GPIO_Port, SG_5V_Power_On_Pin, GPIO_PIN_SET );
		MAX7313_Pin_Write( ioDrivers[ CHIP_LED_GRN_5 ], PORT_LED_GRN_5, 14);
		return St_SG_On;
	}
	return St_SG_Off;
}

// ON
state_t state_func_sg_on(void){
	if( button_states_new.SW5 > button_states_old.SW5 ){   // button pressed
		printf("turning OFF\n");
		HAL_GPIO_WritePin( SG_USBPowerOn_GPIO_Port, SG_USBPowerOn_Pin, GPIO_PIN_RESET );
		HAL_GPIO_WritePin( SG_5V_Power_On_GPIO_Port, SG_5V_Power_On_Pin, GPIO_PIN_RESET );
		MAX7313_Pin_Write( ioDrivers[ CHIP_LED_GRN_5 ], PORT_LED_GRN_5, 15);
		return St_SG_Off;
	}
	return St_SG_On;
}

/* State Table */
state_func_t* const state_table[ N_States ] = {
	state_func_sg_off,
	state_func_sg_on
};

/* Execute State Machine */
state_t run_state(state_t state_now){
	return state_table[ state_now ]();    // Pointer conversation Error
};

/* STATE MACHINE End */

static void Silentgenerator_battery_bar(uint8_t percent, uint8_t bright, uint8_t off){
	if(percent > 110)
		percent = 110;
	for(uint8_t i = 1; (i*10) <= percent; i ++){
		MAX7313_Pin_Write(ioDrivers[ MAX7313_Chips[i] ], MAX7313_Ports[i], bright);
	}
	for(uint8_t i = (percent/10)+1; i <= 11; i++){
		MAX7313_Pin_Write(ioDrivers[ MAX7313_Chips[i] ], MAX7313_Ports[i], off);
	}
}


/* PRINTF REDIRECT to UART BEGIN */
// @see    http://www.keil.com/forum/60531/
// @see    https://stackoverflow.com/questions/45535126/stm32-printf-redirect
struct __FILE{
  int handle;
  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};

FILE __stdout;

int fputc(int ch, FILE *f){
  /* Your implementation of fputc(). */
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

int ferror(FILE *f){
  /* Your implementation of ferror(). */
  return 0;
}
/* PRINTF REDIRECT to UART END */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USB_PCD_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  // scanI2C(&hi2c1);
	
	//adcDriver_1 = new_MAX11615();
	ioDriver_1 = new_MAX7313();
	ioDriver_2 = new_MAX7313();
	
  ioDrivers[0] = &ioDriver_1;  // this array contains 3 items so that the chip number (U1, U2)
	ioDrivers[1] = &ioDriver_1;  // match the array indexes
  ioDrivers[2] = &ioDriver_2;
	
	MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_METER_0   ],    PORT_LED_METER_0, PORT_OUTPUT);
	MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_METER_10  ],   PORT_LED_METER_10, PORT_OUTPUT);
	MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_METER_20  ],   PORT_LED_METER_20, PORT_OUTPUT);
	MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_METER_30  ],   PORT_LED_METER_30, PORT_OUTPUT);
	MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_METER_40  ],   PORT_LED_METER_40, PORT_OUTPUT);
	MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_METER_50  ],   PORT_LED_METER_50, PORT_OUTPUT);
	MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_METER_60  ],   PORT_LED_METER_60, PORT_OUTPUT);
	MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_METER_70  ],   PORT_LED_METER_70, PORT_OUTPUT);
	MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_METER_80  ],   PORT_LED_METER_80, PORT_OUTPUT);
	MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_METER_90  ],   PORT_LED_METER_90, PORT_OUTPUT);
	MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_METER_100 ],  PORT_LED_METER_100, PORT_OUTPUT);
	MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_METER_110 ],  PORT_LED_METER_110, PORT_OUTPUT);
	
	MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_GRN_1 ], PORT_LED_GRN_1, PORT_OUTPUT);
  MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_GRN_2 ], PORT_LED_GRN_2, PORT_OUTPUT);
  MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_GRN_3 ], PORT_LED_GRN_3, PORT_OUTPUT);
  MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_GRN_4 ], PORT_LED_GRN_4, PORT_OUTPUT);
  MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_GRN_5 ], PORT_LED_GRN_5, PORT_OUTPUT);

  MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_RED_1 ], PORT_LED_RED_1, PORT_OUTPUT);
  MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_RED_2 ], PORT_LED_RED_2, PORT_OUTPUT);
  MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_RED_3 ], PORT_LED_RED_3, PORT_OUTPUT);
  MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_RED_4 ], PORT_LED_RED_4, PORT_OUTPUT);

  MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_ERROR ], PORT_LED_ERROR, PORT_OUTPUT);
  MAX7313_Pin_Mode( ioDrivers[ CHIP_LED_POWER ], PORT_LED_POWER, PORT_OUTPUT);

  MAX7313_Pin_Mode( ioDrivers[ CHIP_SWITCH_4 ], PORT_SWITCH_4, PORT_OUTPUT);
  MAX7313_Pin_Mode( ioDrivers[ CHIP_SWITCH_3 ], PORT_SWITCH_3, PORT_OUTPUT);
  MAX7313_Pin_Mode( ioDrivers[ CHIP_SWITCH_2 ], PORT_SWITCH_2, PORT_OUTPUT);
  MAX7313_Pin_Mode( ioDrivers[ CHIP_SWITCH_1 ], PORT_SWITCH_1, PORT_OUTPUT);

	MAX7313_Init(&ioDriver_1, &hi2c1, MAX7313_1);
	MAX7313_Init(&ioDriver_2, &hi2c1, MAX7313_2);
	MAX7313_Interrupt_Enable(&ioDriver_2);
	
	HAL_ADC_Start_DMA(&hadc1, ADC_Buf, 7);
	HAL_ADC_Start_IT(&hadc1);
	
	// analog ref: internal + reference not connected + internal reference always on
	MAX11615_Init(&adcDriver_1, &hi2c1, MAX11615_1, 4+2+1);
	
	button_states_new.SW5 = 0;
	button_states_old.SW5 = 0;
	
	state_t state = St_SG_Off;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
		
  /* USER CODE BEGIN 3 */
		readSGButton(&button_states_new); // old button state
		state = run_state(state);
		HAL_Delay(100);
		button_states_old = button_states_new;
		printf("Solar: %0.4f \tI_Out: %0.4f \tVint: %0.4f \tVUSB: %0.4f \tTemp: %0.4f \tVref: %0.4f \t(VBat: %0.4f)\n", \
 		ADC2Volt(ADC_Buf[0]), (ADC2Volt(ADC_Buf[1])/0.01f)/50.0f, ADC2Volt(ADC_Buf[2]), ADC2Volt(ADC_Buf[3]), \
		ADC2Volt(ADC_Buf[4]), ADC2Volt(ADC_Buf[5]), ADC2Volt(ADC_Buf[6]));
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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PCLK2_DIV4;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 57600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USB init function */
static void MX_USB_PCD_Init(void)
{

  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA0   ------> SharedAnalog_PA0
     PA1   ------> SharedAnalog_PA1
     PA4   ------> SharedAnalog_PA4
     PA5   ------> SharedAnalog_PA5
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, U23_RUN1_Pin|U23_RUN0_Pin|ACDC_On_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SHD_MPPT_Pin|EN2_VccS_Pin|_5V_Power_On_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|Fet1_Pin|Fet2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USBPowerOn_GPIO_Port, USBPowerOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Switch_WKUP_Pin */
  GPIO_InitStruct.Pin = Switch_WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Switch_WKUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : U23_ALERT_Pin Alert_CM_Pin Fan_Fail_Pin OT_Fan_Pin 
                           U61_PGOOD2_Pin */
  GPIO_InitStruct.Pin = U23_ALERT_Pin|Alert_CM_Pin|Fan_Fail_Pin|OT_Fan_Pin 
                          |U61_PGOOD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : U23_RUN1_Pin U23_RUN0_Pin ACDC_On_Pin */
  GPIO_InitStruct.Pin = U23_RUN1_Pin|U23_RUN0_Pin|ACDC_On_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Solar_Voltage_Pin I_Out_Pin V_int_Pin V_USB_Pin */
  GPIO_InitStruct.Pin = Solar_Voltage_Pin|I_Out_Pin|V_int_Pin|V_USB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SHD_MPPT_Pin EN2_VccS_Pin _5V_Power_On_Pin */
  GPIO_InitStruct.Pin = SHD_MPPT_Pin|EN2_VccS_Pin|_5V_Power_On_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Fault_MPPT_Pin */
  GPIO_InitStruct.Pin = Fault_MPPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Fault_MPPT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SS_Pin */
  GPIO_InitStruct.Pin = SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 Fet1_Pin Fet2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|Fet1_Pin|Fet2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USBPowerOn_Pin */
  GPIO_InitStruct.Pin = USBPowerOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USBPowerOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RaspberryGPIO2_Pin */
  GPIO_InitStruct.Pin = RaspberryGPIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RaspberryGPIO2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : U61_PGOOD1_Pin */
  GPIO_InitStruct.Pin = U61_PGOOD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(U61_PGOOD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Interrupt_HMI2_Pin */
  GPIO_InitStruct.Pin = Interrupt_HMI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Interrupt_HMI2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_Flag_Pin */
  GPIO_InitStruct.Pin = USB_Flag_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_Flag_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
