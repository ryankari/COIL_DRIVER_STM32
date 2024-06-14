#include "interruptHandlers.h"
#include "main.h"
#include "usbd_cdc_if.h"
	extern coilParams_typedef coilParams;

void handleStateTIM2(void) {

}


// ------------------------------------------------------------------------------//
void send_uint16_array(int16_t *data,uint16_t Length) {
    uint8_t *byte_data = (uint8_t *)data;
    //while (CDC_Transmit_FS(byte_data, Length * 2) == USBD_BUSY) {
    	CDC_Transmit_FS(byte_data, Length*2);
    //}
}
// ------------------------------------------------------------------------------//



void handleStateSendPeriodicData(void) {
	extern ads_data_typedef ADC_DATA;
	extern int16_t BufferA[BUFFER_LENGTH];
	extern int16_t BufferB[BUFFER_LENGTH];
	extern int16_t BufferC[BUFFER_LENGTH];
	extern int16_t BufferD[BUFFER_LENGTH];
	extern int16_t BufferE[BUFFER_LENGTH];
	extern int16_t TxBuffer[USBTXbufferSize];
	static uint16_t increment; // Used for testing DAC

	TxBuffer[0] = increment;
	increment++;
	if (increment>4) {
		increment = 0;
	}
	if (increment == 0 ) {
		TxBuffer[5] = 0x1230;
		memcpy(&TxBuffer[BUFFER_HEADER_LENGTH],BufferA, BUFFER_LENGTH * sizeof(uint16_t));
	} else if (increment == 1) {
		TxBuffer[5] = 0x1231;
		memcpy(&TxBuffer[BUFFER_HEADER_LENGTH],BufferB, BUFFER_LENGTH * sizeof(uint16_t));
	} else if (increment == 2) {
		TxBuffer[5] = 0x1232;
		memcpy(&TxBuffer[BUFFER_HEADER_LENGTH],BufferC, BUFFER_LENGTH * sizeof(uint16_t));
	} else if (increment == 3) {
		TxBuffer[5] = 0x1233;
		memcpy(&TxBuffer[BUFFER_HEADER_LENGTH],BufferD, BUFFER_LENGTH * sizeof(uint16_t));
	} else if (increment == 4) {
		TxBuffer[5] = 0x1234;
		memcpy(&TxBuffer[BUFFER_HEADER_LENGTH],BufferE, BUFFER_LENGTH * sizeof(uint16_t));
	}
	TxBuffer[1]	= ADC_DATA.data0;
	TxBuffer[2]	= ADC_DATA.data1;
	TxBuffer[3]	= ADC_DATA.data2;
	TxBuffer[4]	= ADC_DATA.data3;
	TxBuffer[6] = 6;
	TxBuffer[7] = 7;
	TxBuffer[8] = 8;
	TxBuffer[9] = 9;
	send_uint16_array(TxBuffer,USBTXbufferSize);

}

int16_t convert_to_signed(uint16_t data) {
    // Midpoint of the 16-bit range
    const uint16_t MIDPOINT = 32768;
    // Subtract the midpoint and cast to signed int
    int16_t signed_data = (int16_t)(data - MIDPOINT);
    return signed_data;
}

void handleUSBReceived(void) {
	extern I2C_HandleTypeDef hi2c2;
	extern uint16_t USBReceivedBuf[USBReceiveLength];
	extern TIM_HandleTypeDef htim3;
	extern TIM_HandleTypeDef htim7;
	extern uint16_t dacIndex;
	  STATE.VALUES.BITS.USBreceived = 0;

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
		  	  if (USBReceivedBuf[1] == 1) {
				  STATE.VALUES.BITS.sendSineWave = 1;
				  dacIndex = 0;
				  STATE.VALUES.BITS.BufferFull = 0;
		  	  } else {
				  DAC_Send_DMA(&hi2c2,convert_to_signed(USBReceivedBuf[2]));
		  	  }
//htim6.Init.Prescaler = USBReceivedBuf[2];

		  } else if (dataType == 3) {
			  //Send USB packet data
			  STOP_USB_UPDATE_PACKET_TIMER
			  STATE.VALUES.BITS.sendData = 1;
		  } else if (dataType == 4) {

		  }
}

void handleBufferOutput(void) {
	extern ads_data_typedef ADC_DATA;
	extern int16_t BufferA[BUFFER_LENGTH];
	extern int16_t TxBuffer[USBTXbufferSize];
	static uint16_t increment; // Used for testing DAC
	memcpy(TxBuffer,BufferA, BUFFER_LENGTH * sizeof(uint16_t));
	TxBuffer[0] = increment;
	increment++;
	TxBuffer[1]	= ADC_DATA.data0;
	TxBuffer[2]	= ADC_DATA.data1;
	TxBuffer[3]	= ADC_DATA.data2;
	TxBuffer[4]	= ADC_DATA.data3;
	TxBuffer[5] = 0x1234;
	TxBuffer[6] = 1;
    send_uint16_array(TxBuffer,USBTXbufferSize);
   // send_uint16_array(BufferA,1024);

}
