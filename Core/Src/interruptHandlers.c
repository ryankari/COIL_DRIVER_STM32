#include "interruptHandlers.h"
#include "main.h"
#include "usbd_cdc_if.h"


void handleStateTIM2(void) {
		static uint16_t dacIndex;
		dacIndex = 0;
		  STATE.VALUES.BITS.stateTIM2 = 0;
		 // DAC_Send_DMA(&hi2c2,5000);
		  //DAC_Send_DMA(&hi2c2,DAC_BUFFER[dacIndex]);
		  dacIndex++;
		  if (dacIndex>DAC_BUFFER_LENGTH) {
			  dacIndex = 0;
		  }
		  dequeue(&stateQueue);
}


// ------------------------------------------------------------------------------//
void send_uint16_array(uint16_t *data) {
    uint8_t *byte_data = (uint8_t *)data;
    CDC_Transmit_FS(byte_data, bufferLength*2);
}
// ------------------------------------------------------------------------------//


void handleStateTIM3(void) {
	extern ads_data_typedef ADC_DATA;
	extern uint16_t TxBuffer[bufferLength];
	static uint16_t increment; // Used for testing DAC
    STATE.VALUES.BITS.stateTIM3 = 0;
	TxBuffer[0] = increment;
	increment++;
	TxBuffer[1]	= ADC_DATA.data0;
	TxBuffer[2]	= ADC_DATA.data1;
	TxBuffer[3]	= ADC_DATA.data2;
	TxBuffer[4]	= ADC_DATA.data3;
	TxBuffer[5] = 0;
	TxBuffer[6] = 0;
    send_uint16_array(TxBuffer);
}


void handleUSBReceived(void) {
	extern I2C_HandleTypeDef hi2c2;
	extern uint16_t USBReceivedBuf[USBReceiveLength];
	  STATE.VALUES.BITS.USBreceived = 0;
	  DAC_Send_DMA(&hi2c2,USBReceivedBuf[0]);
}
