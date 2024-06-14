

#ifndef ADS131M_FUNCTIONS_H
#define ADS131M_FUNCTIONS_H

#include "main.h"

/*Requirements
 * - Setup ADC_SYNC to GPIO output to allow resetting device
 * - Setup TIM (tim1) to provide oscillator to ADC CLK_IN
 * - Setup SS as GPIO set to high speed. (PA4)
 * - Setup SPI1 (PA5, 6, 7) CPOL = Low, CPHA = Edge 2 (falling edge), Speed > 10MHz used
 * 		- DMA used for SPI1 (DMA2)
 * 		Following added into _it.c file. DMA interrupts seen tripping before SPI completed (ADC_STATUS not filled with correct values).
 * 		void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
		{
			  UNUSED(hspi);
			  extern uint32_t STATE_SPI;
			  STATE_SPI = 1;
		}
		Following added into DRDY interrupt handler
 * 		void EXTI15_10_IRQHandler(void)
		{
			triggerADC();


*/

int ADS131M08_cmd_interrupt(uint16_t command_word,uint16_t addr,uint16_t reg_value);
int ADS131M08_read_regs_int(uint16_t addr);
uint32_t ADS131M08_init(uint32_t*);
void triggerADC(void);
void readADC(void);
uint32_t setGain(uint16_t gain);



#define wreg 		0b0110000000000000
#define rreg 		0b1010000000000000
#define reset_reg 	0b0000000000010001
#define null_oper	0b0000000000000000
#define mode_addr	0x02

#define mode_16_bit	0x0004
#define mode_16_bit_lowDRDY	0x0005 //Results in freezing
#define mode_24_bit	0x0104
#define mode_32_bit	0x0304
#define gain1_addr	4
#define adsClock_addr 3
#define pgagainx1	0x2000
#define pgagainx2	0x1111
#define pgagainx4	0x2222

#define adsOSR16256 0x0F1F
#define adsOSR8192 0x0F1B
#define adsOSR128 0x0F03
#define adsOSR256  0x0F07
#define adsOSR512  0x0F0B
#define adsOSR1024 0x0F0F //Default
#define adsOSR2048 0x0F13


#define SS_Pin GPIO_PIN_4
#define SS_GPIO_Port GPIOA
#define ADC_SYNC_Pin GPIO_PIN_1
#define ADC_SYNC_GPIO_Port GPIOB
#define DRDY_Pin GPIO_PIN_4
#define DRDY_GPIO_Port GPIOC

#define ADC_SS_LOW HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, 0)
#define ADC_SS_HIGH HAL_GPIO_WritePin(SS_GPIO_Port, SS_Pin, 1)
#define DRDY_INT_VECTOR EXTI15_10_IRQn

typedef struct REGISTER {
    struct {
        uint16_t RESPONSE;
        uint16_t ID;
        uint16_t STATUS;
        uint16_t MODE;
        uint16_t CLOCK;
        uint16_t GAIN;
        uint16_t CFG;
    } REGISTERS;
} ADC_STATUS_TYPEDEF;

typedef struct OUTPUT {
    union {
        union {
            uint16_t upper;
            uint8_t lower;
            struct {
                uint8_t reg0;
                uint8_t reg1;
                uint8_t reg2;
            } regs;
        } parts;
        struct {
            uint16_t nib0:4;
            uint16_t nib1:4;
            uint16_t nib2:4;
            uint16_t nib3:4;
            uint8_t low;
           } nibbles;
    } cmd_nibs;
} ads_cmd_typedef;

typedef struct ads_16bitout {
    union {
        union {
            uint16_t upper;
            struct {
                uint8_t reg0;
                uint8_t reg1;
            } regs;
        } parts;
        struct {
            uint16_t nib0:4;
            uint16_t nib1:4;
            uint16_t nib2:4;
            uint16_t nib3:4;
           } nibbles;
    } cmd_nibs;
} ads_16bitcmd_typedef;


typedef struct ads_16bit_data {
	int16_t data0;
	int16_t data1;
	int16_t data2;
	int16_t data3;
} ads_data_typedef;


#endif
