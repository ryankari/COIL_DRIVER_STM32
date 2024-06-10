#include "main.h"
#include "string.h"
 extern HAL_StatusTypeDef i2cStatus;

 uint8_t sendDataEEPROM[8];
uint8_t receiveDataEERPOM[8];

 HAL_StatusTypeDef EERPROM_Initialize(I2C_HandleTypeDef *hi2c_handle) {
	 sendDataEEPROM[0] = 0;
	 sendDataEEPROM[1] = 0;
	 sendDataEEPROM[2] =1;
		i2cStatus = HAL_I2C_Master_Transmit(hi2c_handle, EEPROM_Address<<1, sendDataEEPROM, 3, HAL_MAX_DELAY);


		return i2cStatus;
 }


 // Function to sendData to the DAC80501
 HAL_StatusTypeDef EEPROM_WriteBytes(I2C_HandleTypeDef *hi2c_handle,uint8_t memAddress,uint16_t *sendData, size_t length) {

	 	 size_t byte_length = length;
	    uint8_t sendDataEEPROM[byte_length];
	    sendDataEEPROM[0] = memAddress; // The first byte is the memory address

	    // Copy the data to be sent into the buffer
	    memcpy(&sendDataEEPROM[1], sendData, byte_length );

	    // Send the data to the EEPROM
	    //i2cStatus = HAL_I2C_Master_Transmit(hi2c_handle, EEPROM_Address << 1, sendDataEEPROM, byte_length, HAL_MAX_DELAY);
	    i2cStatus = HAL_I2C_Master_Transmit(hi2c_handle, EEPROM_Address << 1, sendDataEEPROM, byte_length+1, HAL_MAX_DELAY);


 	//[Address Byte][Command byte][MSDB][LSDB]
	 //sendDataEEPROM[0] = memAddress; //Address in bytes
	 //sendDataEEPROM[1] = sendData & 0xFF;
	 //sendDataEEPROM[2] = (sendData&0xFF00)>>8;
	 //sendDataEEPROM[3] = 0x78;
	 //sendDataEEPROM[4] = 0x9A;
	 //sendDataEEPROM[5] = 0xBC;
	//i2cStatus = HAL_I2C_Master_Transmit(hi2c_handle, EEPROM_Address<<1, sendDataEEPROM, 6, HAL_MAX_DELAY);
	return(i2cStatus);
}


 // Function to sendData to the DAC80501
 HAL_StatusTypeDef EEPROM_ReadByte(I2C_HandleTypeDef *hi2c_handle,uint8_t memAddress) {
	 extern STATE_typedef STATE;
	 extern EEPROM_Data_typedef EEPROM_Data;

	    sendDataEEPROM[0] = memAddress; //Address in bytes
		 i2cStatus = HAL_I2C_Master_Transmit(hi2c_handle, EEPROM_Address << 1, sendDataEEPROM, 1, HAL_MAX_DELAY);

		// i2cStatus = HAL_I2C_Master_Receive(hi2c_handle, EEPROM_Address<<1, receiveDataEERPOM, 7, HAL_MAX_DELAY);
		// i2cStatus = HAL_I2C_Master_Receive(hi2c_handle, EEPROM_Address<<1, EEPROM_Data.RxData, 7, HAL_MAX_DELAY);
		 i2cStatus = HAL_I2C_Master_Receive_IT(hi2c_handle, EEPROM_Address<<1, EEPROM_Data.RxData, EEPROM_byte_length);
		 while (  STATE.VALUES.BITS.I2Cinterrupt == 1) {}
		 STATE.VALUES.BITS.I2Cinterrupt =0;
		//  memcpy(&EEPROM_Data.RxData[0], receiveDataEERPOM, 6 );


}
