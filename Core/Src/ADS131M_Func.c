

#include <ADS131M_Func.h>

uint32_t ADS131M08_init(uint32_t* STATE_SPI) {
	//  extern uint32_t STATE_SPI;
	  uint32_t error;
	  STATE_SPI = 0;

	  //Reset ADS
	  HAL_GPIO_WritePin(ADC_SYNC_GPIO_Port, ADC_SYNC_Pin, 0);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(ADC_SYNC_GPIO_Port, ADC_SYNC_Pin, 1);

	  error = ADS131M08_cmd_interrupt(wreg,mode_addr,mode_16_bit);
	  if (error > 0) {
		  return(error);
	  }

	  error = ADS131M08_cmd_interrupt(null_oper,0,0);
	  if (error > 0) {
		  return(error);
	  }
	  error = ADS131M08_cmd_interrupt(wreg,gain1_addr,pgagainx1);
	  if (error > 0) {
		  return(error);
	  }
	  error = ADS131M08_read_regs_int(0);
	  if (error > 0) {
		  return(error);
	  }

	  HAL_NVIC_EnableIRQ(DRDY_INT_VECTOR);
	  return (error);

}


int ADS131M08_read_regs_int(uint16_t addr) {
    extern ADC_STATUS_TYPEDEF ADC_STATUS;
    uint16_t increment,error,cmd;
	#define registers_to_read 4

    extern uint32_t STATE_SPI;
    extern spi_buffer_typedef tx_spi_buffer, rx_spi_buffer;
    extern SPI_HandleTypeDef hspi1;

    //Raise SS
    ADC_SS_HIGH;

    //Prepare command
    cmd = rreg + (addr << 7) + registers_to_read;

    tx_spi_buffer.MAP.ints[0] = cmd;
    tx_spi_buffer.MAP.ints[1] = 0;

    // Clear RxBuffer to make it clear what data is read
    for (increment = 0;increment<SPI_TRANSITIONS;increment++) {
    	rx_spi_buffer.MAP.ints[0] = 0;
    }

    ADC_SS_LOW;

    STATE_SPI = 0;
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_spi_buffer.MAP.bytes, rx_spi_buffer.MAP.bytes,6);

    error = 0;
    increment = 0;
    while ((STATE_SPI == 0) && (error == 0)) {
    	increment++;
        if (increment > 5000) {
            error = 2;
        }
    }



    //Wait for DRDY to go low, to wait for prior transmission to complete
    error = 0;
    increment = 0;
    while ((HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin) == 0) && (error == 0)) {
        increment++;
        if (increment > 5000) {
            error = 1;
        }
    }


    ADC_SS_HIGH;
    // Clear RxBuffer to make it clear what data is read
    for (increment = 0;increment<SPI_TRANSITIONS;increment++) {
    	rx_spi_buffer.MAP.ints[increment] = 0;
    	tx_spi_buffer.MAP.ints[increment] = 0;
    }

    ADC_SS_LOW;

    // Run more SPI transmission to read register data

    STATE_SPI = 0;
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_spi_buffer.MAP.bytes, rx_spi_buffer.MAP.bytes,12);

    error = 0;
    increment = 0;
    while ((STATE_SPI == 0) && (error == 0)) {
    	increment++;
        if (increment > 5000) {
            error = 2;
        }
    }


    ADC_SS_HIGH;
    ADC_STATUS.REGISTERS.RESPONSE = rx_spi_buffer.MAP.ints[0];
    ADC_STATUS.REGISTERS.ID = rx_spi_buffer.MAP.ints[1];
    ADC_STATUS.REGISTERS.STATUS = rx_spi_buffer.MAP.ints[2];
    ADC_STATUS.REGISTERS.MODE = rx_spi_buffer.MAP.ints[3];
    ADC_STATUS.REGISTERS.CLOCK = rx_spi_buffer.MAP.ints[4];
    ADC_STATUS.REGISTERS.GAIN = rx_spi_buffer.MAP.ints[5];
    ADC_STATUS.REGISTERS.CFG = rx_spi_buffer.MAP.ints[6];

    return(error);
}




int ADS131M08_cmd_interrupt(uint16_t command_word,uint16_t addr,uint16_t reg_value) {
    uint16_t error,increment;


    extern spi_buffer_typedef tx_spi_buffer, rx_spi_buffer;

    extern SPI_HandleTypeDef hspi1;
    uint16_t cmd;
    extern uint32_t STATE_SPI;

    ADC_SS_HIGH;
    cmd = command_word + (addr << 7);
    //Only sends 1 command. Append number of registers to send after cmd
    for (increment = 0;increment<SPI_TRANSITIONS;increment++) {
    	tx_spi_buffer.MAP.ints[increment] = 0;
    }
    tx_spi_buffer.MAP.ints[0] = cmd;
    tx_spi_buffer.MAP.ints[1] = reg_value;
    ADC_SS_LOW;

    STATE_SPI = 0;
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_spi_buffer.MAP.bytes, rx_spi_buffer.MAP.bytes,6);

    error = 0;
    increment = 0;
    while ((STATE_SPI == 0) && (error == 0)) {
    	increment++;
        if (increment > 5000) {
            error = 3;
        }
    }

    //Wait for DRDY to go low, to wait for prior transmission to complete
    error = 0;
    increment = 0;
    while ((HAL_GPIO_ReadPin(DRDY_GPIO_Port, DRDY_Pin) == 0) && (error == 0)) {
        increment++;
        if (increment > 5000) {
            error = 4;
        }
    }

    ADC_SS_HIGH;
    for (increment = 0;increment<SPI_TRANSITIONS;increment++) {
    	tx_spi_buffer.MAP.ints[increment] = 0;
    	rx_spi_buffer.MAP.ints[increment] = 0;
    }

    ADC_SS_LOW;

    STATE_SPI = 0;
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_spi_buffer.MAP.bytes, rx_spi_buffer.MAP.bytes,12);

    error = 0;
    increment = 0;
    while ((STATE_SPI == 0) && (error == 0)) {
    	increment++;
        if (increment > 5000) {
            error = 5;
        }
    }
    ADC_SS_HIGH;

    return(error);
}


void triggerADC(void) {



    extern SPI_HandleTypeDef hspi1;
	extern uint32_t STATE_SPI;
	static uint32_t index = 0;
	extern spi_buffer_typedef rx_spi_buffer, tx_spi_buffer;
	tx_spi_buffer.MAP.ints[0] = 0;
	tx_spi_buffer.MAP.ints[1] = 0;

	ADC_SS_LOW;
	STATE_SPI = 1;
	HAL_SPI_TransmitReceive_DMA(&hspi1, tx_spi_buffer.MAP.bytes, rx_spi_buffer.MAP.bytes,6);
	/*
	if 	(STATE_SPI == 0) {
		tx_spi_buffer.MAP.ints[0] = 0;
		tx_spi_buffer.MAP.ints[1] = 0;

		ADC_SS_LOW;
		STATE_SPI = 1;
		HAL_SPI_TransmitReceive_DMA(&hspi1, tx_spi_buffer.MAP.bytes, rx_spi_buffer.MAP.bytes,6);
	} else {
		index++;
		if (index > 50) {
			index = 0;
			ADC_SS_LOW;
			STATE_SPI = 1;
			HAL_SPI_TransmitReceive_DMA(&hspi1, tx_spi_buffer.MAP.bytes, rx_spi_buffer.MAP.bytes,6);
		}
	}
	*/

}


void readADC(void) {
	extern ads_data_typedef ADC_DATA;
	extern spi_buffer_typedef rx_spi_buffer;
	ADC_DATA.data0 = rx_spi_buffer.MAP.ints[1];
	ADC_DATA.data1 = rx_spi_buffer.MAP.ints[2];
	ADC_DATA.data2 = rx_spi_buffer.MAP.ints[3];
	ADC_DATA.data3 = rx_spi_buffer.MAP.ints[4];

}

uint32_t setGain(uint16_t gain) {
	uint32_t error;
	if (gain == 1) {
		error = ADS131M08_cmd_interrupt(wreg,gain1_addr,pgagainx1);
	} else if (gain == 2) {
		error = ADS131M08_cmd_interrupt(wreg,gain1_addr,pgagainx2);

	} else if (gain == 4) {
		error = ADS131M08_cmd_interrupt(wreg,gain1_addr,pgagainx4);
	} else {
		error = ADS131M08_cmd_interrupt(wreg,gain1_addr,pgagainx1);
	}
	  if (error > 0) {
		  return(error);
	  }
	  error = ADS131M08_read_regs_int(0);
	  if (error > 0) {
		  return(error);
	  }
}
