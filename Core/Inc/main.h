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
#include "EEPROM.h"
#include "Queue.h"
#include "motorFunctions.h"

extern StateQueue_t stateQueue;

#define BUFFER_LENGTH 1000
#define BUFFER_HEADER_LENGTH 24
#define NUMBER_BUFFERS 4
#define USBTXbufferSize (BUFFER_LENGTH+BUFFER_HEADER_LENGTH)
//#define USBTXbufferLength 510
#define USBReceiveLength 64

#define PI 3.14159265358979323846

void USB_CDC_RxHandler(uint8_t*, uint32_t);
void USB_CDC_RxHandler(uint8_t*, uint32_t);

void handleStateTIM2(void);
void handleStateSendPeriodicData(void);
void handleUSBReceived(void);
void handleBufferOutput(void);

#define START_USB_UPDATE_PACKET_TIMER  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)  {   Error_Handler();  }
#define STOP_USB_UPDATE_PACKET_TIMER  if (HAL_TIM_Base_Stop_IT(&htim3) != HAL_OK)  {   Error_Handler();  }

#define START_BURST_LOG_TIMER if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)  {   Error_Handler();  }
#define STOP_BURST_LOG_TIMER if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)  {   Error_Handler();  }


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


typedef union {
	uint8_t TxData[8];
	struct {
		uint16_t int1;
		uint16_t int2;
		uint16_t int3;
		uint16_t int4;
	} Integers;
} TxData_typedef;


typedef union {
	int16_t TxData[BUFFER_LENGTH*NUMBER_BUFFERS+BUFFER_HEADER_LENGTH*NUMBER_BUFFERS];
	struct {
		uint16_t INITA[BUFFER_HEADER_LENGTH];
		uint16_t A[BUFFER_LENGTH];
		uint16_t INITB[BUFFER_HEADER_LENGTH];
		uint16_t B[BUFFER_LENGTH];
		uint16_t INITC[BUFFER_HEADER_LENGTH];
		uint16_t C[BUFFER_LENGTH];
		uint16_t INITD[BUFFER_HEADER_LENGTH];
		uint16_t D[BUFFER_LENGTH];
	} Buffers;
} DATABUFFER_typedef;


#define EEPROM_byte_length 8
typedef union {
	uint8_t RxData[EEPROM_byte_length];
	struct {
		uint16_t int1;
		uint16_t int2;
		uint16_t int3;
		uint16_t int4;
	} Integers;
} EEPROM_Data_typedef;

typedef struct {
    union {
        struct {
            uint32_t USBreceived : 1;
            uint32_t stateTIM1 : 1;
            uint32_t stateTIM6 : 1;
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
            uint32_t rampMotorOn : 1;
            uint32_t I2Cinterrupt : 1;
            uint32_t USBReceived : 1;
            uint32_t sendSineWave : 1;
            uint32_t stateTIM7 : 1;
            uint32_t sendPeriodicUSB : 1;
            uint32_t BufferFull : 1;
            uint32_t stateTIM2 : 1;
            uint32_t unusedCHar : 8;
        } BITS;
        uint32_t fullState; // 32-bit representation of the state
        struct {
            uint16_t STATElow;
            uint16_t STATEhigh;
        } PARTS;
    } VALUES;
} STATE_typedef;

extern STATE_typedef STATE;


typedef struct {
    unsigned int Amplitude;
    int Offset;
    unsigned int Freq;
    unsigned int Cycles;
    int numCycles;
    uint16_t PtsPerCycle;
    union {
        unsigned int REGISTER;
        struct {
            uint8_t CTRL_CYCLES:1;
            uint8_t DACON:1;
            uint16_t UPDATE_WAVEFORM:1;
            uint16_t bit3:1;
            uint16_t DAC_Status_Update_Display:1;
            uint16_t TFActive:1;
            uint16_t bit6:1;
            uint16_t bit7:1;
            uint16_t bit8:1;
            uint16_t bit9:1;
            uint16_t bit10:1;
            uint16_t bit11:1;
            uint16_t bit12:1;
            uint16_t bit13:1;
            uint16_t bit14:1;
            uint16_t bit15:1;
        } STATES;
    };
} coilParams_typedef;
extern coilParams_typedef coilParams;

void buildWaveform(coilParams_typedef *params);
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
#define MOS1_Pin GPIO_PIN_0
#define MOS1_GPIO_Port GPIOA
#define MOS2_Pin GPIO_PIN_1
#define MOS2_GPIO_Port GPIOA
#define MOS4_Pin GPIO_PIN_3
#define MOS4_GPIO_Port GPIOA
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
