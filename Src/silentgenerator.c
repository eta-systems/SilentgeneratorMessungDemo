
/**
  * @note       STILL IN DEVELOPMENT
  * @file       max3713.c
  * @author     Simon Burkhardt github.com/mnemocron
  * @copyright 	MIT license
  * @date       17.05.2018
  * @brief      C library for the MAX7313 port expander for STM32 HAL.
  * @details    
  * @see        https://datasheets.maximintegrated.com/en/ds/MAX7313.pdf
  * @see        https://forum.arduino.cc/index.php?topic=9682.0
  */

#include "silentgenerator.h"
#include "max7313.h"
#include "main.h"
#include "stm32f3xx_hal.h"

extern MAX7313* ledDrivers[3];

void SG_BTN_ReadSW5(volatile SGButtons *buttons){
	if ( HAL_GPIO_ReadPin(SG_Switch_WKUP_GPIO_Port, SG_Switch_WKUP_Pin) == GPIO_PIN_SET)
		buttons->SW5 = 1;
	else
		buttons->SW5 = 0;

}

/** scan I2C ------------------------------------------------------------------*/
void SG_I2C_ScanAddresses(I2C_HandleTypeDef *hi2c){
	uint8_t error, address;
	uint16_t nDevices;
	nDevices = 0;
	printf("Scanning for available I2C devices...");
	for(address = 1; address < 127; address++ )
	{
		HAL_Delay(10);
		error = HAL_I2C_Master_Transmit(hi2c, address, 0x00, 1, 1);
		//error = HAL_I2C_Master_Receive(hi2c, address, 0x00, 1, 1);
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

float SG_ADC2Volt(uint32_t adc_val){
	/**
	ADC Values vs. Voltage
	 (measurement values)
	1960      1.64024
	1790      1.49625
	1525      1.27725
	1180      1.99107
	*/
	return (float)adc_val * (1.64024f / 1960.0f);
}

/**
  * @brief converts MAX11615 AD value to a voltage
  * @note  the conversation depends on the analog reference of the chip
  */
float SG_MAX_ADC2Volt(uint32_t adc_val){
	// Vrefint = 2.048V
	// 1.968Vmin 2.048 2.128Vmax
	// float vrefint = 2.048f;
	return (float) adc_val * (2.048f) / 4096.0f;
}

/**
  * @ TODO : Untested - not working
  */
float SG_TEMPIntCelsius(float Vtemp){
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

// must be realized as a function because global array definitions are not possible
uint8_t SG_MAX7313_LED_RED_Ports(uint8_t n){
	return arr_MAX7313_LED_RED_Ports[n];
}
// must be realized as a function because global array definitions are not possible
uint8_t SG_MAX7313_LED_GRN_Ports(uint8_t n){
	return arr_MAX7313_LED_GRN_Ports[n];
}
// must be realized as a function because global array definitions are not possible
uint8_t SG_MAX7313_LED_RED_Chips(uint8_t n){
	return arr_MAX7313_LED_RED_Chips[n];
}
// must be realized as a function because global array definitions are not possible
uint8_t SG_MAX7313_LED_GRN_Chips(uint8_t n){
	return arr_MAX7313_LED_GRN_Chips[n];
}
// must be realized as a function because global array definitions are not possible
uint8_t SG_Battery_Meter_Ports(uint8_t n){
	return arr_MAX7313_Battery_Meter_Ports[n];
}
// must be realized as a function because global array definitions are not possible
uint8_t SG_Battery_Meter_Chips(uint8_t n){
  return arr_MAX7313_Battery_Meter_Chips[n];
}

void SG_LED_SetAll(uint8_t brightness){
	for(uint8_t i = 0; i<12; i++){
		MAX7313_Pin_Write(ledDrivers[ SG_Battery_Meter_Chips(i) ], SG_Battery_Meter_Ports(i), brightness);
	}
	for(uint8_t i = 0; i<4; i++){
		MAX7313_Pin_Write(ledDrivers[ SG_MAX7313_LED_RED_Chips(i) ], SG_MAX7313_LED_RED_Ports(i), brightness);
		MAX7313_Pin_Write(ledDrivers[ SG_MAX7313_LED_GRN_Chips(i) ], SG_MAX7313_LED_GRN_Ports(i), brightness);
	}
	MAX7313_Pin_Write(ledDrivers[ SG_MAX7313_LED_GRN_Chips(4) ], SG_MAX7313_LED_GRN_Ports(4), brightness);
}

/**
	*@brief 	Sets the LEDs as a BAR from 0 to percentage on the HUI according to the percentage
	*@param 	percent The percentage value to display (0 - 110)
	*@param 	bright_on Brightness value for ON LEDs (PWM setting 0-15)
	*@param 	bright_off Brightness value for OFF LEDs (PWM setting 0-15)
	*/
void SG_BAT_LED_SetBar(uint8_t percent, uint8_t bright_on, uint8_t bright_off){
	/** @bug on lowest voltages, the 0% LED stays on despite not being controlled by software
	possible bug with setting the registers in the MAX7313 library */
  if(percent > 110)
    percent = 110;
  for(uint8_t i = 1; (i*10) <= percent; i ++){
    MAX7313_Pin_Write(ledDrivers[ SG_Battery_Meter_Chips(i) ], SG_Battery_Meter_Ports(i), bright_on);
  }
  for(uint8_t i = (percent/10)+1; i < 12; i++){
    MAX7313_Pin_Write(ledDrivers[ SG_Battery_Meter_Chips(i) ], SG_Battery_Meter_Ports(i), bright_off);
  }
}

/**
	*@brief 	Sets only ONE LED on the HUI according to the percentage
	*@param 	percent The percentage value to display (0 - 110)
	*@param 	bright_on Brightness value for ON LEDs (PWM setting 0-15)
	*@param 	bright_off Brightness value for OFF LEDs (PWM setting 0-15)
	*/
void SG_BAT_LED_SetDot(uint8_t percent, uint8_t bright_on, uint8_t bright_off){
  if(percent > 110)
    percent = 110;
	for(uint8_t i = 0; i < 12; i++){
		if(percent >= (i*10) && percent < ((i+1)*10))
			MAX7313_Pin_Write(ledDrivers[ SG_Battery_Meter_Chips(i) ], SG_Battery_Meter_Ports(i), bright_on);
		else
			MAX7313_Pin_Write(ledDrivers[ SG_Battery_Meter_Chips(i) ], SG_Battery_Meter_Ports(i), bright_off);
	}
}

/**
	*@brief 	Sets the LEDs on the HUI according to the percentage
	*@param 	percent The percentage value to display (0 - 110)
	*@param 	bright_on Brightness value for ON LEDs (PWM setting 0-15)
	*@param 	bright_off Brightness value for OFF LEDs (PWM setting 0-15)
	*/
void SG_BAT_LED_Update(uint32_t adc, uint8_t fullbar){
	if(fullbar)
		SG_BAT_LED_SetBar( SG_BAT_LED_GetLevel( SG_ADC_GetVint(adc) ), SG_BAT_LED_BRIGHT_ON, SG_BAT_LED_BRIGHT_OFF);
	else
		SG_BAT_LED_SetDot( SG_BAT_LED_GetLevel( SG_ADC_GetVint(adc) ), SG_BAT_LED_BRIGHT_ON, SG_BAT_LED_BRIGHT_OFF);
}

uint16_t SG_BAT_LED_GetLevel(float volt){
	/** @todo Genaue Spannungs-Prozentkurve definieren */
	// in %/V
	float m = (100.0f - 10.0f)/\
		(SG_12V_BAT_LEV_VOLT_100 - SG_12V_BAT_LEV_VOLT_10);
	float q = 100.0f - (SG_12V_BAT_LEV_VOLT_100 * m);
	
	return (uint16_t)((volt * m) + q);
}

float SG_ADC_GetVint(uint32_t adc){
	/** Messung vom 2018.09.07 in Derendingen 
	ADC		 Vint
	 972    5.4409
	1763   11.2525
	2021   13.1705
	2320   15.4178
	*/
	float m = 0.007478;  // V / LSB
	float q = -1.9312f;  // V offset
	return ((float)adc) * m + q;
}


