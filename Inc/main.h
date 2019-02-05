/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Switch_WKUP_Pin GPIO_PIN_13
#define Switch_WKUP_GPIO_Port GPIOC
#define U23_ALERT_Pin GPIO_PIN_0
#define U23_ALERT_GPIO_Port GPIOC
#define U23_RUN1_Pin GPIO_PIN_1
#define U23_RUN1_GPIO_Port GPIOC
#define U23_RUN0_Pin GPIO_PIN_2
#define U23_RUN0_GPIO_Port GPIOC
#define Alert_CM_Pin GPIO_PIN_3
#define Alert_CM_GPIO_Port GPIOC
#define Solar_Voltage_Pin GPIO_PIN_0
#define Solar_Voltage_GPIO_Port GPIOA
#define I_Out_Pin GPIO_PIN_1
#define I_Out_GPIO_Port GPIOA
#define Cell_On_Pin GPIO_PIN_2
#define Cell_On_GPIO_Port GPIOA
#define UART_MPPT_Pin GPIO_PIN_3
#define UART_MPPT_GPIO_Port GPIOA
#define V_int_Pin GPIO_PIN_4
#define V_int_GPIO_Port GPIOA
#define V_USB_Pin GPIO_PIN_5
#define V_USB_GPIO_Port GPIOA
#define Temp_Set_Pin GPIO_PIN_6
#define Temp_Set_GPIO_Port GPIOA
#define Temp_Set_EXTI_IRQn EXTI9_5_IRQn
#define SHD_MPPT_Pin GPIO_PIN_0
#define SHD_MPPT_GPIO_Port GPIOB
#define Fault_MPPT_Pin GPIO_PIN_1
#define Fault_MPPT_GPIO_Port GPIOB
#define EN2_VccS_Pin GPIO_PIN_2
#define EN2_VccS_GPIO_Port GPIOB
#define SS_Pin GPIO_PIN_8
#define SS_GPIO_Port GPIOE
#define ACDC_On_Pin GPIO_PIN_6
#define ACDC_On_GPIO_Port GPIOC
#define Fan_Fail_Pin GPIO_PIN_8
#define Fan_Fail_GPIO_Port GPIOC
#define Fan_Fail_EXTI_IRQn EXTI9_5_IRQn
#define OT_Fan_Pin GPIO_PIN_9
#define OT_Fan_GPIO_Port GPIOC
#define OT_Fan_EXTI_IRQn EXTI9_5_IRQn
#define Fet1_Pin GPIO_PIN_9
#define Fet1_GPIO_Port GPIOA
#define Fet2_Pin GPIO_PIN_10
#define Fet2_GPIO_Port GPIOA
#define USBPowerOn_Pin GPIO_PIN_6
#define USBPowerOn_GPIO_Port GPIOF
#define RaspberryGPIO2_Pin GPIO_PIN_7
#define RaspberryGPIO2_GPIO_Port GPIOF
#define U61_PGOOD2_Pin GPIO_PIN_12
#define U61_PGOOD2_GPIO_Port GPIOC
#define U61_PGOOD1_Pin GPIO_PIN_2
#define U61_PGOOD1_GPIO_Port GPIOD
#define Interrupt_HMI2_Pin GPIO_PIN_5
#define Interrupt_HMI2_GPIO_Port GPIOB
#define Interrupt_HMI2_EXTI_IRQn EXTI9_5_IRQn
#define _5V_Power_On_Pin GPIO_PIN_6
#define _5V_Power_On_GPIO_Port GPIOB
#define USB_Flag_Pin GPIO_PIN_7
#define USB_Flag_GPIO_Port GPIOB
#define USB_Flag_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
