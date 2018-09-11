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
#include "silentgenerator.h"
#include "max11615.h"     // ADC
#include "max7313.h"      // I/O Port Expander
#include "max3440x.h"     // 
#include "ltc3886.h"      // DCDC Buck Converter
#include "max6615.h"      // PWM Fan Controller

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
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

MAX11615 adcDriver_1;
MAX3440X adcMonitor_1;
MAX7313 huiIODriver_1;
MAX7313 huiIODriver_2;
MAX7313 fetDriver;
MAX7313* ledDrivers[3];

MAX6615 fanDriver;
LTC3886 dcdc12VDriver;

volatile SGButtons 	button_states_old, \
										button_states_new, \
										button_action_required;
static uint8_t 	SG_Enabled_USB = 0, \
								SG_Enabled_AC = 0, \
								SG_Enabled_Solar = 0, \
								SG_Enabled_DC = 0;
volatile uint32_t display_delay = 0;

volatile uint8_t ERR_FLAG_USB = 0;


/* STATE MACHINE Begin */
typedef enum {St_SG_Off, St_SG_On, N_States} state_t;
typedef state_t state_func_t(void);

// OFF
state_t state_func_sg_off(void){
  if( button_states_new.SW5 > button_states_old.SW5 ){
    printf("turning ON\n");
    MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_GRN_5 ], SG_PORT_LED_GRN_5, SG_HUI_LED_BRIGHT_ON);
		display_delay = 20000;
    return St_SG_On;
  }
  return St_SG_Off;
}

// ON
state_t state_func_sg_on(void){
  if( button_states_new.SW5 > button_states_old.SW5 ){
    printf("turning OFF\n");
    HAL_GPIO_WritePin( SG_USBPowerOn_GPIO_Port, SG_USBPowerOn_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( SG_5V_Power_On_GPIO_Port, SG_5V_Power_On_Pin, GPIO_PIN_RESET );
    MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_GRN_5 ], SG_PORT_LED_GRN_5, 15);
		/** @todo restliche Funktionen ausschalten */
		SG_LED_SetAll(15);
		SG_Enabled_Solar = 0;
		SG_Enabled_AC = 0;
		SG_Enabled_DC = 0;
		SG_Enabled_USB = 0;
		printf("OFF\n");
		
    return St_SG_Off;   // back to OFF State
  }
	/* BEGIN Silentgenerator MAIN LOOP */
	
	/** @todo: dem Programmvorschlag entsprechend */
	
	display_delay += SG_TIME_DEBOUNCE;
		
	if(display_delay >= 5000){
		SG_BAT_LED_Update(ADC_Buf[2], 0);
		printf("Vint:\t%d\t%0.4f V\t%d\n", ADC_Buf[2], \
		SG_ADC_GetVint(ADC_Buf[2]), \
		SG_BAT_LED_GetLevel(SG_ADC_GetVint(ADC_Buf[2])));
		SG_BAT_LED_Update(ADC_Buf[2], 1);
		display_delay = 0;
	}
	
	/* BUTTON SOLAR */
	if(button_action_required.SW1){
		button_action_required.SW1 = 0;
		if(SG_Enabled_Solar){
			printf("Solar is OFF\n");
			MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_GRN_1 ], SG_PORT_LED_GRN_1, SG_BAT_LED_BRIGHT_OFF);
			SG_Enabled_Solar = 0;
		} else {
			printf("Solar is ON\n");
			MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_GRN_1 ], SG_PORT_LED_GRN_1, SG_BAT_LED_BRIGHT_ON);
			SG_Enabled_Solar = 1;
		}
	}
	/* BUTTON DC */
	if(button_action_required.SW2){
		button_action_required.SW2 = 0;
		if(SG_Enabled_DC){
			printf("DC is OFF\n");
			MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_GRN_2 ], SG_PORT_LED_GRN_2, SG_BAT_LED_BRIGHT_OFF);
			SG_Enabled_DC = 0;
		} else {
			printf("DC is ON\n");
			MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_GRN_2 ], SG_PORT_LED_GRN_2, SG_HUI_LED_BRIGHT_ON);
			SG_Enabled_DC = 1;
		}
	}
	/* BUTTON USB */
	if(button_action_required.SW3){
		button_action_required.SW3 = 0;
		if(SG_Enabled_USB){
			printf("USB is OFF\n");
			MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_GRN_3 ], SG_PORT_LED_GRN_3, SG_BAT_LED_BRIGHT_OFF);
			SG_Enabled_USB = 0;
			HAL_GPIO_WritePin(SG_USBPowerOn_GPIO_Port, SG_USBPowerOn_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin( SG_5V_Power_On_GPIO_Port, SG_5V_Power_On_Pin, GPIO_PIN_RESET );
		} else {
			printf("USB is ON\n");
			MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_GRN_3 ], SG_PORT_LED_GRN_3, SG_BAT_LED_BRIGHT_ON);
			MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_RED_3 ], SG_PORT_LED_RED_3, SG_BAT_LED_BRIGHT_OFF);
			SG_Enabled_USB = 1;
			HAL_GPIO_WritePin(SG_USBPowerOn_GPIO_Port, SG_USBPowerOn_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin( SG_5V_Power_On_GPIO_Port, SG_5V_Power_On_Pin, GPIO_PIN_SET );
		}
	}
	/* BUTTON AC */
	if(button_action_required.SW4){
		button_action_required.SW4 = 0;
		if(SG_Enabled_AC){
			printf("AC is OFF\n");
			MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_GRN_4 ], SG_PORT_LED_GRN_4, SG_BAT_LED_BRIGHT_OFF);
			SG_Enabled_AC = 0;
		} else {
			printf("AC is ON\n");
			MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_GRN_4 ], SG_PORT_LED_GRN_4, SG_BAT_LED_BRIGHT_ON);
			SG_Enabled_AC = 1;
		}
	}
	
	/* ERROR FLAGS from Interrupt Pins */
	if(ERR_FLAG_USB){
		ERR_FLAG_USB = 0;
		printf("USB ERROR - USB is OFF\n");
		MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_GRN_3 ], SG_PORT_LED_GRN_3, SG_BAT_LED_BRIGHT_OFF);
		MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_RED_3 ], SG_PORT_LED_RED_3, SG_BAT_LED_BRIGHT_ON);
		SG_Enabled_USB = 0;
	}
	
	/* END Silentgenerator MAIN LOOP */
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

/* STATE MACHINE END */

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
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_USB_PCD_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, ADC_Buf, 7);
  HAL_ADC_Start_IT(&hadc1);
  
  // analog ref: internal + reference not connected + internal reference always on
  MAX11615_Init(&adcDriver_1, &hi2c1, SG_ADDRESS_MAX11615_DCDC_1, 4+2+1);
  SG_MAX7313_Init();
	MAX3440X_Init(&adcMonitor_1,  &hi2c1, SG_ADDRESS_MAX3440X_MPPT_1);
	LTC3886_Init (&dcdc12VDriver, &hi2c1, SG_ADDRESS_MAX3886);
	MAX6615_Init (&fanDriver,     &hi2c1, SG_ADDRESS_MAX6615);
	
  button_states_new.SW5 = 0;
  button_states_old.SW5 = 0;
  
  state_t state = St_SG_Off;
	
	printf("\nETA Systems Silentgenerator\n");
	printf("Embedded Software Version (in development)\n\n");
	
  SG_I2C_ScanAddresses(&hi2c1);
	SG_LED_SetAll(15);
	MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_ERROR ], SG_PORT_LED_ERROR, 0);
	HAL_Delay(200);
	MAX7313_Pin_Write( ledDrivers[ SG_CHIP_LED_ERROR ], SG_PORT_LED_ERROR, 15);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
    /* MAIN PROGRAM LOOP */
    SG_BTN_ReadSW5(&button_states_new);      // poll SW5 state
    state = run_state(state);                // run state machine
		HAL_Delay(SG_TIME_DEBOUNCE);             // wait for 10~50ms
		button_states_old = button_states_new;   // refresh button states
		
		HAL_Delay(500);
		uint8_t asdf = 0;
		float temp1 = 0.0f, temp2 = 0.0;
		//LTC3886_Read8(&dcdc12VDriver, LTC3886_MFR_ID, &asdf);
		MAX6615_ReadTemperature(&fanDriver, 1, &temp1);
		MAX6615_ReadTemperature(&fanDriver, 2, &temp2);
		printf("CH1:\t%.3f\tCH2:\t%.3f\n", temp1, temp2);
		
    //printf("Solar: %0.4f \tI_Out: %0.4f \tVint: %0.4f \tVUSB: %0.4f \tTemp: %0.4f \tVref: %0.4f \t(VBat: %0.4f)\n", \
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

  /*Configure GPIO pin : Temp_Set_Pin */
  GPIO_InitStruct.Pin = Temp_Set_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Temp_Set_GPIO_Port, &GPIO_InitStruct);

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
