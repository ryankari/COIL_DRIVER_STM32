#include "interruptHandlers.h"
#include "main.h"
#include "usbd_cdc_if.h"
extern coilParams_typedef coilParams;


// ------------------------------------------------------------------------------//
void send_uint16_array(int16_t *data,uint16_t Length) {
    uint8_t *byte_data = (uint8_t *)data;
    //while (CDC_Transmit_FS(byte_data, Length * 2) == USBD_BUSY) {
    	CDC_Transmit_FS(byte_data, Length*2);
    //}
}
// ------------------------------------------------------------------------------//

void handleStateSendPeriodicData(void) {
	//[0 Transmit Length][1 mux][2 Status][3 ID][4 data0][5 data1][6 data2][7 data3][8 empty][9 buffer]
	extern ads_data_typedef ADC_DATA;
	extern DATABUFFER_typedef DATABUFFER;
	static uint16_t mux; // Used for testing DAC
	extern uint32_t pulseCount;
	int16_t ID;

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
		DATABUFFER.Buffers.INITA[10] = pulseCount;
		send_uint16_array(&DATABUFFER.TxData[USBTXbufferSize*0],USBTXbufferSize);
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
		DATABUFFER.Buffers.INITB[10] = pulseCount;
		send_uint16_array(&DATABUFFER.TxData[USBTXbufferSize*1],USBTXbufferSize);
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
		DATABUFFER.Buffers.INITC[10] = pulseCount;
		send_uint16_array(&DATABUFFER.TxData[USBTXbufferSize*2],USBTXbufferSize);
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
		DATABUFFER.Buffers.INITD[10] = pulseCount;
		send_uint16_array(&DATABUFFER.TxData[USBTXbufferSize*3],USBTXbufferSize);
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
	//dataType 1 - Parameters, 2 - Start Test, 3 - Not Used, 4 - Motor Functions, 5 Engage Pins
	extern I2C_HandleTypeDef hi2c2;
	extern uint16_t USBReceivedBuf[USBReceiveLength];
	extern uint16_t dacIndex;
	  STATE.VALUES.BITS.USBreceived = 0;

	  uint16_t dataType;
	  dataType  = USBReceivedBuf[0];

	  switch (dataType) {
		  case 1:
			  coilParams.Amplitude = USBReceivedBuf[1];
			  coilParams.Offset = USBReceivedBuf[2];
			  coilParams.Freq = USBReceivedBuf[3];
			  coilParams.numCycles = USBReceivedBuf[4];
			  buildWaveform(&coilParams);
			  break;
		  case 2:
			  //Start burst logging and execute test}
			  //START_BURST_LOG_TIMER
			  	  if (USBReceivedBuf[1] == 1) {
					  STATE.VALUES.BITS.sendSineWave = 1;
					  dacIndex = 0;
					  STATE.VALUES.BITS.BufferFull = 0;
			  	  } else {
					  DAC_Send_DMA(&hi2c2,convert_to_signed(USBReceivedBuf[2]));
			  	  }
			  	  break;
		  case 3:
			  //Send USB packet data
			  STATE.VALUES.BITS.sendData = 1;
			  STATE.VALUES.BITS.sendPeriodicUSB = 0;
			  break;
		  case 4: // Motor functions
			  if (USBReceivedBuf[2] == 1) {
				  STATE.VALUES.BITS.motorDirForward = 1;
			  } else {
				  STATE.VALUES.BITS.motorDirForward = 0;
			  }

		  	  if (USBReceivedBuf[1] == 1) {
		  		 updateSpeedandDuty(USBReceivedBuf[3],USBReceivedBuf[4]);
		  		 turnMotorOn();
		  	  } else {
		  		 turnMotorOff();
		  	  }
		  	  break;
		  case 5: //Engage functions
			  if (USBReceivedBuf[1] == 1) {
			  				  STATE.VALUES.BITS.engageState = 1;
			  				HAL_GPIO_WritePin(GPIOB, LED1_Pin,1);
			  			  } else {
			  				  STATE.VALUES.BITS.engageState = 0;
			  				HAL_GPIO_WritePin(GPIOB, LED1_Pin,0);
			  			  }

		  default:
			  break;
	  }

}

