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

MAX7313 ioDriver_1;
MAX7313 ioDriver_2;

volatile uint8_t interrupt_val = 0;
volatile uint8_t led_mode = 0;
uint8_t laufvariable = 0;

const uint8_t battery_Leds[12] = {7, 8, 9, 10, 14, 15, 8, 9, 13, 14, 4, 1};

static void set_all_leds(uint8_t val){
	for(uint8_t i=0; i<12; i++){
		MAX7313_Pin_Write( ( (i/2 >= 3) ? (&ioDriver_1):(&ioDriver_2) ) , battery_Leds[i], val);
	}
}

static void set_battery_bar(uint8_t prozent){
	// Todo
	return;
}

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

#define MAX11615_1 0x66

MAX11615 adcDriver_1;

/** scan I2C ------------------------------------------------------------------*/
/** 
  * 
  */
static void scanI2C(I2C_HandleTypeDef *hi2c){
	uint8_t error, address;
  uint16_t nDevices;
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
		HAL_Delay(10);
		error = HAL_I2C_Master_Transmit(hi2c, address, 0x00, 1, 1);
    if (error == HAL_OK)
    {
      printf("I2C device found at address 0x");
      if (address<16)
        printf("0");
      printf("%X", address);
      printf("  !\n");
 
      nDevices++;
    }
  }
  if (nDevices == 0)
    printf("No I2C devices found\n");
  else
    printf("done\n");
}

#define VREF1V23 ADC_Buf[5]

static float ADC2Volt(uint32_t adc_val){
	/** ADC Values vs. Voltage
	1960      1.64024
	1790      1.49625
	1525      1.27725
	1180      1.99107
 (0         0.00000)
	*/
	return (float)adc_val * (1.64024f / 1960.0f);
}

static float MAX_ADC2Volt(uint32_t adc_val){
	// Vrefint = 2.048V
	// 1.968Vmin 2.048 2.128Vmax
	// float vrefint = 2.048f;
	return (float) adc_val * (2.048f) / 4096.0f;
}

static float TEMPIntCelsius(float Vtemp){
	// STM32F373 datasheet - chapter 6.3.20 (p.105)
	
	// float V25typ = 1.43;     // V
	// float Vslope = 0.0043;   // V/°C
	// float Tslope = 1.0/Vslope;
	// float Toffset = 25.0 - V25typ*Tslope;
	
	// #define TEMP110_CAL_VALUE (((uint16_t*)((uint32_t)0x1FFFF7C2 ))
  // #define TEMP30_CAL_VALUE  (((uint16_t*)((uint32_t)0x1FFFF7B8 ))
	// float temperature = (float)((110.0f - 30.0f) / ((float)(*TEMP110_CAL_VALUE) - (float)(*TEMP30_CAL_VALUE)) * ((float)ADC_Buf[4] - (float)(*TEMP30_CAL_VALUE)) + 30.0f);
	float temperature = (float)(((Vtemp*1000) - 1430) / (4.3f) + 25.0f);
	return temperature;
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
	
	MAX7313_Pin_Mode(&ioDriver_2, 7, PORT_OUTPUT);  //  0%
	MAX7313_Pin_Mode(&ioDriver_2, 8, PORT_OUTPUT);  // 10%
	MAX7313_Pin_Mode(&ioDriver_2, 9, PORT_OUTPUT);  // 20%
	MAX7313_Pin_Mode(&ioDriver_2,10, PORT_OUTPUT);  // 30%
	MAX7313_Pin_Mode(&ioDriver_2,14, PORT_OUTPUT);  // 40%
	MAX7313_Pin_Mode(&ioDriver_2,15, PORT_OUTPUT);  // 50%
	MAX7313_Pin_Mode(&ioDriver_1, 8, PORT_OUTPUT);  // 60%
	MAX7313_Pin_Mode(&ioDriver_1, 9, PORT_OUTPUT);  // 70%
	MAX7313_Pin_Mode(&ioDriver_1,13, PORT_OUTPUT);  // 80%
	MAX7313_Pin_Mode(&ioDriver_1,14, PORT_OUTPUT);  // 90%
	MAX7313_Pin_Mode(&ioDriver_1, 4, PORT_OUTPUT);  // 100%
	MAX7313_Pin_Mode(&ioDriver_1, 1, PORT_OUTPUT);  // 110%
	
	MAX7313_Pin_Mode(&ioDriver_2, 1, PORT_INPUT);
	MAX7313_Pin_Mode(&ioDriver_2, 2, PORT_INPUT);
	MAX7313_Pin_Mode(&ioDriver_2, 3, PORT_INPUT);
	MAX7313_Pin_Mode(&ioDriver_2, 4, PORT_INPUT);
	
	MAX7313_Init(&ioDriver_1, &hi2c1, MAX7313_1);
	MAX7313_Init(&ioDriver_2, &hi2c1, MAX7313_2);
	MAX7313_Interrupt_Enable(&ioDriver_2);
	
	HAL_ADC_Start_DMA(&hadc1, ADC_Buf, 7);
	HAL_ADC_Start_IT(&hadc1);
	
	// analog ref: internal + reference not connected + internal reference always on
	MAX11615_Init(&adcDriver_1, &hi2c1, MAX11615_1, 4+2+1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		/*
		switch(led_mode){
			case 0:							// 1 LED blinkt
				set_all_leds(15);
				if(laufvariable > 10){
					// MAX7313_Pin_Write(&ioDriver_1, 14, 0);
					MAX7313_Pin_Write(&ioDriver_1, 14, 0);
					HAL_GPIO_WritePin(USBPowerOn_GPIO_Port, USBPowerOn_Pin, GPIO_PIN_SET);
				} else {
					HAL_GPIO_WritePin(USBPowerOn_GPIO_Port, USBPowerOn_Pin, GPIO_PIN_RESET);
				}
				break;
			case 1:							// alle LEDs blinken
				if(laufvariable > 10)
					set_all_leds(0);
				else
					set_all_leds(15);
				break;
			case 2: 						// alle LEDs ein
				set_all_leds(0);
				break;
			case 3:							// alle LEDs aus
				set_all_leds(15);
				break;
			default:
				led_mode = 0;
		}
		HAL_Delay(100);
		
		uint8_t phase00, phase01, phase10, phase11;
		MAX7313_Read8(&ioDriver_2, MAX7313_BLINK_PHASE_0_00_07, &phase00);
		MAX7313_Read8(&ioDriver_2, MAX7313_BLINK_PHASE_0_08_15, &phase01);
		//printf("P0: "BYTE_TO_BINARY_PATTERN BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(phase01), BYTE_TO_BINARY(phase00));
		printf("P0 %#04x%02x\n", phase01, phase00);
		*/
		
		MAX7313_Pin_Write(&ioDriver_2, 7, 14);
		MAX7313_Pin_Write(&ioDriver_2, 8, 14);
		HAL_Delay(1000);
		MAX7313_Pin_Write(&ioDriver_2, 7, 15);
		MAX7313_Pin_Write(&ioDriver_2, 8, 15);
		HAL_Delay(1000);
		
		// Analog Werte an USART3 printen
		// PA0 - PA1 - PA4 - PA5 - TempInt - VrefInt - VBatInt
		
		//printf("Solar: %04i \tI_Out: %04i \tVint: %04i \tVUSB: %04i \tTemp: %04i \tVref: %04i \t(VBat: %04i)\n", \
		ADC_Buf[0], ADC_Buf[1], ADC_Buf[2], ADC_Buf[3], ADC_Buf[4], ADC_Buf[5], ADC_Buf[6]);
		//printf("Solar: %.03fV \tI_Out: %.03fV \tVint: %.03fV \tVUSB: %.03fV \tTemp: %2.03f°C \tVref: %.03fV \t(VBat: %.03fV)\n", \
		ADC2Volt(ADC_Buf[0]), ADC2Volt(ADC_Buf[1]), ADC2Volt(ADC_Buf[2]), ADC2Volt(ADC_Buf[3]), \
		TEMPIntCelsius(ADC2Volt(ADC_Buf[4])), ADC2Volt(ADC_Buf[5]), ADC2Volt(ADC_Buf[6]));
		
		// printf("V_int: %.03fV\n", ADC2Volt(ADC_Buf[2]));
		uint16_t adc_bits = 0;
		float adc_volt = 0.0;
		
		/*
		for(uint8_t i=0; i<8; i++){
			MAX11615_ADC_Read(&adcDriver_1, i, &adc_bits);
			// printf("%d:\t%05i\n", i, adc_bits);
			adc_volt = MAX_ADC2Volt(adc_bits);
			printf("%d:\t%04i   %.04fV\n", i, adc_bits, adc_volt);
		}*/
		printf("\n");

		laufvariable ++;
		if(laufvariable > 20)
			laufvariable = 0;
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
