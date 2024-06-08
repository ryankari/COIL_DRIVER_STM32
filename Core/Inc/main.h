/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "encoder.h"
#include "DAC80501.h"
#include "ADS131M_Func.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	union {
		uint8_t rx_buffer[64];
		uint16_t rx_buffer_int[32];
		uint32_t rx_buffer_long[16];
	} UN;
} rx_buffer_typedef;



typedef union {
	uint8_t TxData[8];
	struct {
		uint16_t int1;
		uint16_t int2;
		uint16_t int3;
		uint16_t int4;
	} Integers;
} TxData_typedef;



typedef struct {
	union {
	uint16_t STATE;
	struct {
        uint32_t USBreceived : 1;
        uint32_t stateTIM1 : 1;
        uint32_t stateTIM2 : 1;
        uint32_t stateTIM3 : 1;
        uint32_t buttonPress : 1;
        uint32_t sendData : 1;
        uint32_t stateTIM4 : 1;
        uint32_t motorRamp : 1;
        uint32_t stateTIM5 : 1;
        uint32_t SW1 : 1;
        uint32_t SW2 : 1;
        uint32_t motorOn : 1;
        uint32_t motorDirForward : 1;
        uint32_t motorChangeState : 1;
        uint32_t executeRandomPosition : 1;
        uint32_t randomPositionUnitialized : 1;
        uint32_t rampMotorOn :1;
        uint32_t unused1:1;
        uint32_t unused2:1;
        uint32_t unused3:1;
        uint32_t unused4:1;
        uint32_t unused5:1;
        uint32_t unused6:1;
        uint32_t unused7:1;
        uint32_t unusedCHar:8;

		} BITS;
	} VALUES;
} STATE_typedef;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SS_Pin_Pin GPIO_PIN_4
#define SS_Pin_GPIO_Port GPIOA
#define DRDY_Pin GPIO_PIN_4
#define DRDY_GPIO_Port GPIOC
#define DRDY_EXTI_IRQn EXTI4_IRQn
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SPI_TRANSITIONS 15

typedef struct STATES {
	union {
		uint16_t outputChannels;
		struct {
			uint16_t tmr1:1;
			uint16_t drdy:1;
			uint16_t tmr2:1;
			uint16_t spi_int:1;
			uint16_t adc_active:1;
			uint16_t bit5:1;
			uint16_t bit6:1;
			uint16_t bit7:1;
			uint8_t  char0;
			uint8_t char1;

		} bits;
	} cmd;
} states_var_typedef;


typedef struct TXBuffer {
	union {
		int16_t signedint[SPI_TRANSITIONS];
		uint16_t ints[SPI_TRANSITIONS];
		uint8_t bytes[SPI_TRANSITIONS*2];
	} MAP;
} spi_buffer_typedef;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
