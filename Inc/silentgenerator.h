
#include "max7313.h"
#include "main.h"
#include "stm32f3xx_hal.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SILENTGENERATOR_H
#define __SILENTGENERATOR_H

/* Private define ------------------------------------------------------------*/
#define PORT_LED_METER_0    7
#define PORT_LED_METER_10   8
#define PORT_LED_METER_20   9
#define PORT_LED_METER_30   10
#define PORT_LED_METER_40   14
#define PORT_LED_METER_50   15
#define PORT_LED_METER_60   8
#define PORT_LED_METER_70   9
#define PORT_LED_METER_80   13
#define PORT_LED_METER_90   14
#define PORT_LED_METER_100  4
#define PORT_LED_METER_110  1
#define PORT_LED_POWER      0
#define PORT_LED_RED_4      2
#define PORT_LED_GRN_4      3
#define PORT_LED_GRN_3      11
#define PORT_LED_ERROR      7
#define PORT_LED_RED_3      12
#define PORT_LED_RED_1      5
#define PORT_LED_GRN_1      6
#define PORT_LED_GRN_2      12      
#define PORT_LED_RED_2      13
#define PORT_LED_GRN_5      0

#define PORT_SWITCH_4       0
#define PORT_SWITCH_3       1
#define PORT_SWITCH_2       2
#define PORT_SWITCH_1       3

#define CHIP_LED_METER_0    2
#define CHIP_LED_METER_10   2
#define CHIP_LED_METER_20   2
#define CHIP_LED_METER_30   2
#define CHIP_LED_METER_40   2
#define CHIP_LED_METER_50   2
#define CHIP_LED_METER_60   1
#define CHIP_LED_METER_70   1
#define CHIP_LED_METER_80   1
#define CHIP_LED_METER_90   1
#define CHIP_LED_METER_100  1
#define CHIP_LED_METER_110  1
#define CHIP_LED_POWER      1
#define CHIP_LED_RED_4      1
#define CHIP_LED_GRN_4      1
#define CHIP_LED_ERROR      1
#define CHIP_LED_GRN_3      1
#define CHIP_LED_RED_3      1
#define CHIP_LED_RED_1      2
#define CHIP_LED_GRN_1      2
#define CHIP_LED_GRN_2      2   
#define CHIP_LED_RED_2      2
#define CHIP_LED_GRN_5      1

#define CHIP_SWITCH_4       2
#define CHIP_SWITCH_3       2
#define CHIP_SWITCH_2       2
#define CHIP_SWITCH_1       2

#define SG_U23_ALERT_Pin 				GPIO_PIN_0
#define SG_U23_ALERT_GPIO_Port 			GPIOC
#define SG_U23_RUN1_Pin 				GPIO_PIN_1
#define SG_U23_RUN1_GPIO_Port 			GPIOC
#define SG_U23_RUN0_Pin 				GPIO_PIN_2
#define SG_U23_RUN0_GPIO_Port 			GPIOC
#define SG_Alert_CM_Pin 				GPIO_PIN_3
#define SG_Alert_CM_GPIO_Port 			GPIOC
#define SG_Solar_Voltage_Pin 			GPIO_PIN_0
#define SG_Solar_Voltage_GPIO_Port 		GPIOA
#define SG_I_Out_Pin 					GPIO_PIN_1
#define SG_I_Out_GPIO_Port 				GPIOA
#define SG_V_int_Pin 					GPIO_PIN_4
#define SG_V_int_GPIO_Port 				GPIOA
#define SG_V_USB_Pin 					GPIO_PIN_5
#define SG_V_USB_GPIO_Port 				GPIOA
#define SG_SHD_MPPT_Pin 				GPIO_PIN_0
#define SG_SHD_MPPT_GPIO_Port 			GPIOB
#define SG_Fault_MPPT_Pin 				GPIO_PIN_1
#define SG_Fault_MPPT_GPIO_Port 		GPIOB
#define SG_EN2_VccS_Pin 				GPIO_PIN_2
#define SG_EN2_VccS_GPIO_Port 			GPIOB
#define SG_SS_Pin 						GPIO_PIN_8
#define SG_SS_GPIO_Port 				GPIOE
#define SG_ACDC_On_Pin 					GPIO_PIN_6
#define SG_ACDC_On_GPIO_Port 			GPIOC
#define SG_Fan_Fail_Pin 				GPIO_PIN_8
#define SG_Fan_Fail_GPIO_Port 			GPIOC
#define SG_Fan_Fail_EXTI_IRQn 			EXTI9_5_IRQn
#define SG_OT_Fan_Pin 					GPIO_PIN_9
#define SG_OT_Fan_GPIO_Port 			GPIOC
#define SG_OT_Fan_EXTI_IRQn 			EXTI9_5_IRQn
#define SG_Fet1_Pin 					GPIO_PIN_9
#define SG_Fet1_GPIO_Port 				GPIOA
#define SG_Fet2_Pin 					GPIO_PIN_10
#define SG_Fet2_GPIO_Port 				GPIOA
#define SG_USBPowerOn_Pin 				GPIO_PIN_6
#define SG_USBPowerOn_GPIO_Port 		GPIOF
#define SG_RaspberryGPIO2_Pin 			GPIO_PIN_7
#define SG_RaspberryGPIO2_GPIO_Port 	GPIOF
#define SG_U61_PGOOD2_Pin 				GPIO_PIN_12
#define SG_U61_PGOOD2_GPIO_Port 		GPIOC
#define SG_U61_PGOOD1_Pin 				GPIO_PIN_2
#define SG_U61_PGOOD1_GPIO_Port 		GPIOD
#define SG_Interrupt_HMI2_Pin 			GPIO_PIN_5
#define SG_Interrupt_HMI2_GPIO_Port 	GPIOB
#define SG_Interrupt_HMI2_EXTI_IRQn 	EXTI9_5_IRQn
#define SG_5V_Power_On_Pin 				GPIO_PIN_6
#define SG_5V_Power_On_GPIO_Port 		GPIOB
#define SG_USB_Flag_Pin 				GPIO_PIN_7
#define SG_USB_Flag_GPIO_Port 			GPIOB
#define SG_USB_Flag_EXTI_IRQn 			EXTI9_5_IRQn

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

void scanI2C(I2C_HandleTypeDef *);
float ADC2Volt(uint32_t);
float MAX_ADC2Volt(uint32_t);
float TEMPIntCelsius(float);

/**
  * @}
  */ 

#endif /* __SILENTGENERATOR_H */
/************************ (C) COPYRIGHT ETA Systems *****END OF FILE****/
