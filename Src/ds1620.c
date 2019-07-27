#include "main.h"
#include "gpio.h"
#include "ds1620.h"
#include "cmsis_os.h"

void initDS1620(void)
{
	// All pins -> output
	writeCommandTo1620( DS1620_CMD_WRITECONF, 0x02 );			// CPU mode; continous conversion
//	writeByteTo1620( DS1620_CMD_STARTCONV );					// Start conversion
}

void shiftOutByte( uint8_t val )
{
	int i;
	// Send uint8_t, LSB first
	for( i = 0; i < 8; i++ )
	{
		HAL_GPIO_WritePin(GPIOB, DS1620_PIN_CLK, GPIO_PIN_RESET);
		
		// Set bit
		if( val & (1 << i))
		{
			HAL_GPIO_WritePin(GPIOB, DS1620_PIN_DQ, GPIO_PIN_SET);
			osDelay(2);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, DS1620_PIN_DQ, GPIO_PIN_RESET);
			osDelay(2);
		}
		HAL_GPIO_WritePin(GPIOB, DS1620_PIN_CLK, GPIO_PIN_SET);
	}
}

void writeByteTo1620( uint8_t cmd )
{
	HAL_GPIO_WritePin(GPIOB, DS1620_PIN_RST, GPIO_PIN_SET);

	shiftOutByte( cmd );
	
	HAL_GPIO_WritePin(GPIOB, DS1620_PIN_RST, GPIO_PIN_RESET);
}

void writeCommandTo1620( uint8_t cmd, uint8_t data )
{
	HAL_GPIO_WritePin(GPIOB, DS1620_PIN_RST, GPIO_PIN_SET);
	
	shiftOutByte( cmd );	// send command
	shiftOutByte( data );	// send 8 bit data
	
	HAL_GPIO_WritePin(GPIOB, DS1620_PIN_RST, GPIO_PIN_RESET);
}

void writeTempTo1620( uint8_t reg, int temp )
{
	uint8_t lsb = temp;											// truncate to high uint8_t
	uint8_t msb = temp >> 8;									// shift high -> low uint8_t
	
	HAL_GPIO_WritePin(GPIOB, DS1620_PIN_RST, GPIO_PIN_SET);
	
	shiftOutByte( reg );	// send register select
	shiftOutByte( lsb );	// send LSB 8 bit data
	shiftOutByte( msb );	// send MSB 8 bit data (only bit 0 is used)
	
	HAL_GPIO_WritePin(GPIOB, DS1620_PIN_RST, GPIO_PIN_RESET);

}

int readTempFrom1620()
{
	int i;
	GPIO_PinState state;
	
	HAL_GPIO_WritePin(GPIOB, DS1620_PIN_RST, GPIO_PIN_SET);
	
	shiftOutByte( DS1620_CMD_READTEMP );						// send register select
	
	set_input();
	int raw = 0;
	
	for( i=0; i<9; i++ )										// read 9 bits
	{
		HAL_GPIO_WritePin(GPIOB, DS1620_PIN_CLK, GPIO_PIN_RESET);

		osDelay(10);
		state = HAL_GPIO_ReadPin(GPIOB, DS1620_PIN_DQ);
		if(state == GPIO_PIN_SET)
			raw |= (1 << i);									// add value
		HAL_GPIO_WritePin(GPIOB, DS1620_PIN_CLK, GPIO_PIN_SET);
	}
	
	HAL_GPIO_WritePin(GPIOB, DS1620_PIN_RST, GPIO_PIN_RESET);
	
	set_output();
//	return (double)(raw/(double)2);								// divide by 2 and return
	return raw;
}
