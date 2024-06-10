#include "main.h"

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
// Function to sendData to the DAC80501
HAL_StatusTypeDef DAC_Send(I2C_HandleTypeDef *hi2c_handle,uint16_t Data) {
	//[Address Byte][Command byte][MSDB][LSDB]
	sendData[0] = DACDATAcmd;
	sendData[1] = Data & 0xFF;
	sendData[2] = (Data&0xFF00)>>8;


    //i2cStatus = HAL_I2C_Master_Send(hi2c_handle, FX29ReadAddr<<1, sendData, 2, HAL_MAX_DELAY);
    i2cStatus = HAL_I2C_Master_Transmit(hi2c_handle, DAC_Address<<1, sendData, 3, HAL_MAX_DELAY);

    return i2cStatus;
}
