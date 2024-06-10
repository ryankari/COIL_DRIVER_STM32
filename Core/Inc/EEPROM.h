
#ifndef EEPROM_H

	#define EEPROM_H



	#define EEPROM_Address     0b1010000

	HAL_StatusTypeDef EERPROM_Initialize(I2C_HandleTypeDef *hi2c_handle) ;
	HAL_StatusTypeDef EEPROM_WriteBytes(I2C_HandleTypeDef *hi2c_handle,uint8_t ,uint16_t *sendBuffer,  size_t);

	HAL_StatusTypeDef EEPROM_ReadByte(I2C_HandleTypeDef *hi2c_handle,uint8_t memAddress);




#endif
