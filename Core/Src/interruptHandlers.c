#include "interruptHandlers.h"
#include "main.h"
#include "usbd_cdc_if.h"
	extern coilParams_typedef coilParams;

void handleStateTIM2(void) {

}


// ------------------------------------------------------------------------------//
void send_uint16_array(uint16_t *data,uint16_t Length) {
    uint8_t *byte_data = (uint8_t *)data;
    //while (CDC_Transmit_FS(byte_data, Length * 2) == USBD_BUSY) {
    	CDC_Transmit_FS(byte_data, Length*2);
    //}
}
// ------------------------------------------------------------------------------//


void handleStateTIM3(void) {
	extern ads_data_typedef ADC_DATA;
	extern uint16_t BufferA[USBTXbufferSize];
	extern uint16_t TxBuffer[USBTXbufferSize];
	static uint16_t increment; // Used for testing DAC
	memcpy(TxBuffer,BufferA, USBTXbufferSize * sizeof(uint16_t));
	TxBuffer[0] = increment;
	increment++;
	TxBuffer[1]	= ADC_DATA.data0;
	TxBuffer[2]	= ADC_DATA.data1;
	TxBuffer[3]	= ADC_DATA.data2;
	TxBuffer[4]	= ADC_DATA.data3;
	TxBuffer[5] = 0;
	TxBuffer[6] = 0;
    send_uint16_array(TxBuffer,USBTXbufferLength);
}


void handleUSBReceived(void) {
	extern I2C_HandleTypeDef hi2c2;
	extern uint16_t USBReceivedBuf[USBReceiveLength];
	extern TIM_HandleTypeDef htim3;
	extern TIM_HandleTypeDef htim7;
	extern uint16_t dacIndex;
	  STATE.VALUES.BITS.USBreceived = 0;
	  //DAC_Send_DMA(&hi2c2,USBReceivedBuf[0]);
	  uint16_t dataType;
	  dataType  = USBReceivedBuf[0];

	  if (dataType == 1) {
		  //Parameters
		  coilParams.Amplitude = USBReceivedBuf[1];
		  coilParams.Offset = USBReceivedBuf[2];
		  coilParams.Freq = USBReceivedBuf[3];
		  coilParams.numCycles = USBReceivedBuf[4];

	  } else if (dataType == 2) {
		  //Start burst logging and execute test}
		  //START_BURST_LOG_TIMER
			  STATE.VALUES.BITS.sendSineWave = 1;
			  dacIndex = 0;
			  STATE.VALUES.BITS.BufferFull = 0;
//htim6.Init.Prescaler = USBReceivedBuf[2];



		  } else if (dataType == 3) {
			  //Send USB packet data
			  STOP_USB_UPDATE_PACKET_TIMER
			  STATE.VALUES.BITS.sendData = 1;
		  }
}

void handleBufferOutput(void) {
	extern uint16_t BufferA[USBTXbufferSize];
	extern ads_data_typedef ADC_DATA;
	extern uint16_t TxBuffer[USBTXbufferSize];
	memcpy(TxBuffer,BufferA, USBTXbufferSize);
	static uint16_t increment;
	TxBuffer[0] = increment;
	increment++;
	TxBuffer[1]	= ADC_DATA.data0;
	TxBuffer[2]	= ADC_DATA.data1;
	TxBuffer[3]	= ADC_DATA.data2;
	TxBuffer[4]	= ADC_DATA.data3;
	TxBuffer[6] = 0;
    send_uint16_array(TxBuffer,USBTXbufferLength);
   // send_uint16_array(BufferA,1024);

}
