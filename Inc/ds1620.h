#ifndef _DS1620_H
#define _DS1620_H
#define DS1620_CMD_READTEMP		0xAA
#define DS1620_CMD_WRITETH		0x01
#define DS1620_CMD_WRITETL		0x02
#define DS1620_CMD_READTH		0xA1
#define DS1620_CMD_READTL		0xA2
#define DS1620_CMD_READCNTR		0xA0
#define DS1620_CMD_READSLOPE	0xA9
#define DS1620_CMD_STARTCONV	0xEE
#define DS1620_CMD_STOPCONV		0x22
#define DS1620_CMD_WRITECONF	0x0C
#define DS1620_CMD_READCONF		0xAC

// DQ is set from input to output and back in gpio.c
// the others need to be init'd somehow if not configured using STMCube
#define DS1620_PIN_DQ	GPIO_PIN_0
#define DS1620_PIN_CLK	GPIO_PIN_1
#define DS1620_PIN_RST	GPIO_PIN_2
/*
red 5v
org gnd
org/wh rst
blu clk
blu/wh data

+
8 7 6 5
1 2 3 4
	  -
1 - DQ	
2 - CLK
3 - RST
4 - GND
8 - VDD

*/

void initDS1620(void);
void writeByteTo1620( uint8_t cmd );
void writeCommandTo1620( uint8_t cmd, uint8_t data );
void writeTempTo1620( uint8_t reg, int temp );
//double readTempFrom1620();
int readTempFrom1620();
#endif