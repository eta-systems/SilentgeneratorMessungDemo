
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

#include "max7313.h"
#include "main.h"
#include "stm32f3xx_hal.h"

/** scan I2C ------------------------------------------------------------------*/
void scanI2C(I2C_HandleTypeDef *hi2c){
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

float ADC2Volt(uint32_t adc_val){
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
float MAX_ADC2Volt(uint32_t adc_val){
	// Vrefint = 2.048V
	// 1.968Vmin 2.048 2.128Vmax
	// float vrefint = 2.048f;
	return (float) adc_val * (2.048f) / 4096.0f;
}

/**
  * @ TODO : Untested - not working
  */
float TEMPIntCelsius(float Vtemp){
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
