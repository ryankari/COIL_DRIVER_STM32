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

/*
void send_continuous_data(int16_t *data) {
     uint8_t *byte_data = (uint8_t *)data;
    send_uint16_array(byte_data, 8);
}

void send_buffered_data(int16_t *data) {
    uint8_t *byte_data = (uint8_t *)data;
    send_uint16_array(buffered_data, 1024);
}
*/
void handleStateSendPeriodicData(void) {
	//[0 Transmit Length][1 mux][2 Status][3 ID][4 data0][5 data1][6 data2][7 data3][8 empty][9 buffer]
	extern ads_data_typedef ADC_DATA;
	extern DATABUFFER_typedef DATABUFFER;
	extern int16_t TxBuffer[USBTXbufferSize];
	static uint16_t mux; // Used for testing DAC
	int16_t ID;
	/*
	TxBuffer[0] = USBTXbufferSize; //[0 Length]
	TxBuffer[1] = 0; // [1 mux]
	TxBuffer[2] = STATE.VALUES.STATE; // [2 status]
	TxBuffer[3] = ID_Buffer0; // [3 ID]
	TxBuffer[4]	= ADC_DATA.data0;
	TxBuffer[5]	= ADC_DATA.data1;
	TxBuffer[6]	= ADC_DATA.data2;
	TxBuffer[7]	= ADC_DATA.data3;
	TxBuffer[8] = 0x0000;
	TxBuffer[9] = 0x0000;
	send_uint16_array(TxBuffer,USBTXbufferSize);
	*/
	switch (mux)  {
	case 0:
		ID = ID_Buffer0;
		DATABUFFER.Buffers.INITA[0] = USBTXbufferSize; //[0 Length]
		DATABUFFER.Buffers.INITA[1] = mux; // [1 mux]
		DATABUFFER.Buffers.INITA[2] = 0; // [2 status]
		DATABUFFER.Buffers.INITA[3] = ID; // [3 ID]
		DATABUFFER.Buffers.INITA[4]	= ADC_DATA.data0;
		DATABUFFER.Buffers.INITA[5]	= ADC_DATA.data1;
		DATABUFFER.Buffers.INITA[6]	= ADC_DATA.data2;
		DATABUFFER.Buffers.INITA[7]	= ADC_DATA.data3;
		DATABUFFER.Buffers.INITA[8] = STATE.VALUES.PARTS.STATElow;
		DATABUFFER.Buffers.INITA[9] = STATE.VALUES.PARTS.STATEhigh;
		send_uint16_array(&DATABUFFER.TxData[0],USBTXbufferSize);
		break;
	case 1:
		ID = ID_Buffer1;
		DATABUFFER.Buffers.INITB[0] = USBTXbufferSize; //[0 Length]
		DATABUFFER.Buffers.INITB[1] = mux; // [1 mux]
		DATABUFFER.Buffers.INITB[2] =0; // [2 status]
		DATABUFFER.Buffers.INITB[3] = ID; // [3 ID]
		DATABUFFER.Buffers.INITB[4]	= ADC_DATA.data0;
		DATABUFFER.Buffers.INITB[5]	= ADC_DATA.data1;
		DATABUFFER.Buffers.INITB[6]	= ADC_DATA.data2;
		DATABUFFER.Buffers.INITB[7]	= ADC_DATA.data3;
		DATABUFFER.Buffers.INITB[8] = STATE.VALUES.PARTS.STATElow;
		DATABUFFER.Buffers.INITB[9] = STATE.VALUES.PARTS.STATEhigh;
		send_uint16_array(&DATABUFFER.TxData[1010],USBTXbufferSize);
		break;
	case 2:
		ID = ID_Buffer2;
		DATABUFFER.Buffers.INITC[0] = USBTXbufferSize; //[0 Length]
		DATABUFFER.Buffers.INITC[1] = mux; // [1 mux]
		DATABUFFER.Buffers.INITC[2] = 0;
		DATABUFFER.Buffers.INITC[3] = ID; // [3 ID]
		DATABUFFER.Buffers.INITC[4]	= ADC_DATA.data0;
		DATABUFFER.Buffers.INITC[5]	= ADC_DATA.data1;
		DATABUFFER.Buffers.INITC[6]	= ADC_DATA.data2;
		DATABUFFER.Buffers.INITC[7]	= ADC_DATA.data3;
		DATABUFFER.Buffers.INITC[8] = STATE.VALUES.PARTS.STATElow;
		DATABUFFER.Buffers.INITC[9] = STATE.VALUES.PARTS.STATEhigh;
		send_uint16_array(&DATABUFFER.TxData[2020],USBTXbufferSize);
		break;
	case 3:
		ID = ID_Buffer3;
		DATABUFFER.Buffers.INITD[0] = USBTXbufferSize; //[0 Length]
		DATABUFFER.Buffers.INITD[1] = mux; // [1 mux]
		DATABUFFER.Buffers.INITD[2] = 0;
		DATABUFFER.Buffers.INITD[3] = ID; // [3 ID]
		DATABUFFER.Buffers.INITD[4]	= ADC_DATA.data0;
		DATABUFFER.Buffers.INITD[5]	= ADC_DATA.data1;
		DATABUFFER.Buffers.INITD[6]	= ADC_DATA.data2;
		DATABUFFER.Buffers.INITD[7]	= ADC_DATA.data3;
		DATABUFFER.Buffers.INITD[8] = STATE.VALUES.PARTS.STATElow;
		DATABUFFER.Buffers.INITD[9] = STATE.VALUES.PARTS.STATEhigh;
		send_uint16_array(&DATABUFFER.TxData[3030],USBTXbufferSize);
		break;
	default:
		ID = 0;
	}

	mux++;
	if (mux>4) {
		mux = 0;
	}

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
		  buildWaveform(&coilParams);

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
			//  STOP_USB_UPDATE_PACKET_TIMER
			  STATE.VALUES.BITS.sendData = 1;
			  STATE.VALUES.BITS.sendPeriodicUSB = 0;
		  } else if (dataType == 4) {

		  }
}

void handleBufferOutput(void) {
	extern ads_data_typedef ADC_DATA;
	/*
	extern int16_t BufferA[BUFFER_LENGTH];
	extern int16_t BufferB[BUFFER_LENGTH];
	extern int16_t BufferC[BUFFER_LENGTH];
	extern int16_t BufferD[BUFFER_LENGTH];
	extern int16_t BufferE[BUFFER_LENGTH];
	*/
	DATABUFFER_typedef DATABUFFER;

	extern int16_t TxBuffer[USBTXbufferSize];
	static uint16_t mux; // Used for testing DAC

	TxBuffer[0] = USBTXbufferSize; //[0 Length]
	TxBuffer[1] = 0; // [1 mux]
	TxBuffer[2] = 0; // [2 status]
	TxBuffer[3] = 0; // [3 ID]
	TxBuffer[4]	= ADC_DATA.data0;
	TxBuffer[5]	= ADC_DATA.data1;
	TxBuffer[6]	= ADC_DATA.data2;
	TxBuffer[7]	= ADC_DATA.data3;
	TxBuffer[8] = 0x0000;
	TxBuffer[9] = 0x0000;
	memcpy(&TxBuffer[BUFFER_HEADER_LENGTH],DATABUFFER.TxData, BUFFER_LENGTH * sizeof(uint16_t));
	send_uint16_array(TxBuffer,1010);

	/*DATABUFFER.INIT[0] = 4010; //[0 Length]
	DATABUFFER.INIT[1] = mux; // [1 mux]

	DATABUFFER.INIT[4]	= ADC_DATA.data0;
	DATABUFFER.INIT[5]	= ADC_DATA.data1;
	DATABUFFER.INIT[6]	= ADC_DATA.data2;
	DATABUFFER.INIT[7]	= ADC_DATA.data3;
	*/
	/*
	switch (mux)  {
	case 0:
		TxBuffer[3] = ID_Buffer0;
		memcpy(&TxBuffer[BUFFER_HEADER_LENGTH],DATABUFFER.Buffers.A, BUFFER_LENGTH * sizeof(uint16_t));
		break;
	case 1:
		TxBuffer[3] = ID_Buffer1;
		memcpy(&TxBuffer[BUFFER_HEADER_LENGTH],DATABUFFER.Buffers.C, BUFFER_LENGTH * sizeof(uint16_t));
		break;
	case 2:
		TxBuffer[3] = ID_Buffer2;
		memcpy(&TxBuffer[BUFFER_HEADER_LENGTH],DATABUFFER.Buffers.C, BUFFER_LENGTH * sizeof(uint16_t));
		break;
	case 3:
		TxBuffer[3] = ID_Buffer3;
		memcpy(&TxBuffer[BUFFER_HEADER_LENGTH],DATABUFFER.Buffers.D, BUFFER_LENGTH * sizeof(uint16_t));
		break;
	case 4:
		TxBuffer[3] = ID_Buffer4;
		memcpy(&TxBuffer[BUFFER_HEADER_LENGTH],DATABUFFER.Buffers.D, BUFFER_LENGTH * sizeof(uint16_t));
		break;
	default:
		break;
	}
*/
	//TxBuffer[8] = 0x0000;
	//send_uint16_array(DATABUFFER.TxData,1010);
	//send_uint16_array(TxBuffer,1010);
	/*
	mux++;
	if (mux>4) {
		mux = 0;
		STATE.VALUES.BITS.sendPeriodicUSB = 1;
		STATE.VALUES.BITS.sendData = 0;
	}
	*/
	STATE.VALUES.BITS.sendPeriodicUSB = 1;
	STATE.VALUES.BITS.sendData = 0;
}
