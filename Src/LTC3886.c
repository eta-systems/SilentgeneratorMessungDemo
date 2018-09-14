
/**
  * @file       ltc3886.c
  * @author     Simon Burkhardt github.com/mnemocron
  * @copyright  MIT license
  * @date       2018
  * @brief      C library for the LTC3886 Step-Down Controller for STM32 HAL.
  * @details
  * @see        github.com/eta-systems
  * @see        http://www.analog.com/media/en/technical-documentation/data-sheets/3886fe.pdf
  */

#include "main.h"
#include "ltc3886.h"



/**
  * @brief 		Initalizer function
  * @param 		*chip, pointer to the LTC3886 typedef struct
  * @param 		*wireIface a pointer to a HAL I2C_HandleTypeDef
  * @param 		address of the chip on the I2C bus
  */
uint8_t LTC3886_Init(LTC3886 *chip, I2C_HandleTypeDef *wireIface, uint16_t address){
	chip->wireIface = wireIface;
	chip->devAddress = address;
	return 0;
}
