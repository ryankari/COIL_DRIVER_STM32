#ifndef DAC80501

	#define DAC80501
	HAL_StatusTypeDef DAC_Send(I2C_HandleTypeDef *,uint16_t Data);
	HAL_StatusTypeDef DAC_Send_DMA(I2C_HandleTypeDef *hi2c_handle,uint16_t Data);
	HAL_StatusTypeDef DAC_Initialize(I2C_HandleTypeDef *hi2c_handle) ;

	#define DAC_Address     0b1001000
	#define NOOPcmd 0x00;
	#define DEVIDcmd 0x1
	#define SYNCcmd 0x2
	#define CONFIGcmd 0x3
	#define GAINcmd 0x4
	#define TRIGGERcmd 0x5
	#define STATUScmd 0x7
	#define DACDATAcmd 0x8
#endif
