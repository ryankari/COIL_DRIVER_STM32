#include "main.h"
#include <math.h>
// Buffer to store received data
 uint8_t sendData[8];


 HAL_StatusTypeDef i2cStatus;


 // Function to initialize the DAC80501
 HAL_StatusTypeDef DAC_Initialize(I2C_HandleTypeDef *hi2c_handle) {
		sendData[0] = GAINcmd;
		sendData[1] = 0;
		sendData[2] =1;
		i2cStatus = HAL_I2C_Master_Transmit(hi2c_handle, DAC_Address<<1, sendData, 3, HAL_MAX_DELAY);


		return i2cStatus;
 }

 uint16_t convert_to_unsigned(int16_t signed_data) {
     // Add the midpoint to shift the signed range to unsigned range
     uint16_t unsigned_data = (uint16_t)(signed_data + 32768);
     return unsigned_data;
 }


// Function to sendData to the DAC80501
HAL_StatusTypeDef DAC_Send(I2C_HandleTypeDef *hi2c_handle,uint16_t Data) {
	//[Address Byte][Command byte][MSDB][LSDB]
	uint16_t unsigned_data = convert_to_unsigned(Data);
	sendData[0] = DACDATAcmd;
	sendData[2] = Data & 0xFF;
	sendData[1] = (Data&0xFF00)>>8;
    //i2cStatus = HAL_I2C_Master_Send(hi2c_handle, FX29ReadAddr<<1, sendData, 2, HAL_MAX_DELAY);
    i2cStatus = HAL_I2C_Master_Transmit(hi2c_handle, DAC_Address<<1, sendData, 3, HAL_MAX_DELAY);

    return i2cStatus;
}



// Function to sendData to the DAC80501
HAL_StatusTypeDef DAC_Send_DMA(I2C_HandleTypeDef *hi2c_handle,int16_t Data) {
	//[Address Byte][Command byte][MSDB][LSDB]
	uint16_t unsigned_data = convert_to_unsigned(Data);
	sendData[0] = DACDATAcmd;
	sendData[1] = (unsigned_data&0xFF00)>>8;
	sendData[2] = unsigned_data & 0xFF;

    //i2cStatus = HAL_I2C_Master_Send(hi2c_handle, FX29ReadAddr<<1, sendData, 2, HAL_MAX_DELAY);
    i2cStatus = HAL_I2C_Master_Transmit_DMA(hi2c_handle, DAC_Address<<1, sendData, 3);
    return i2cStatus;
}


void buildWaveform(coilParams_typedef *params)  {
	extern uint16_t DAC_BUFFER[BUFFER_LENGTH];
	//htim7.Init.Prescaler = 50;
	// if (HAL_TIM_Base_Init(&htim7) != HAL_OK)  {Error_Handler();  }
	//Freq = Xtal/Prescalar/Counter/BUFFER_LENGTH/Cycles
	float temp;
	for (uint16_t i=0;i<BUFFER_LENGTH;i++) {
		temp = 0.5*sinf(2*PI*i/BUFFER_LENGTH*coilParams.numCycles);
		DAC_BUFFER[i] = (int16_t)(coilParams.Amplitude * temp)+coilParams.Offset;
	}
}

