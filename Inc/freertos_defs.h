#ifndef __FREERTOS_H
#define __FREERTOS_H

enum rt_values_offsets
{
	OIL_PRES,
	AIR_TEMP,
	MAP,
	OIL_TEMP,
	O2,
	ODOM,
	TRIP,
	RPM,
	MPH,
	RUN_TIME,
	ENG_TEMP,
	OUTDOOR_TEMP,
	INDOOR_TEMP
} RT_VALUES_OFFSETS_TYPES;

enum key_types
{
 	KP_1 = 0xE0, // '1'		- E0
	KP_2, // '2'		- E1
	KP_3, // '3'		- E2
	KP_4, // '4'		- E3
	KP_5, // '5'		- E4
	KP_6, // '6'		- E5
	KP_7, // '7'		- E6
	KP_8, // '8'		- E7
	KP_9, // '9'		- E8
	KP_A, // 'A'		- E9
	KP_B, // 'B'		- EA
	KP_C, // 'C'		- EB
	KP_D, // 'D'		- EC
	KP_POUND,	// '#'	- ED
	KP_AST, // '*'		- EE
	KP_0 	// '0'		- EF
} KEY_TYPES;

typedef struct
{
	UCHAR row;
	UCHAR col;
	UCHAR data_col;
	UCHAR str;
} FORMAT_STR;

#define col0 col0_Pin
#define col1 col1_Pin
#define col2 col2_Pin
#define col3 col3_Pin

#define row0 row0_Pin
#define row1 row1_Pin
#define row2 row2_Pin
#define row3 row3_Pin

#define NUM_ROWS 4
#define NUM_COLS 4

#define ROWS 16
#define COLUMN 40
#define DISPLAY_OFF         0x90    //0b10010000, display off
#define CURSOR_ON_BLINK_OFF 0x92    //0b1001xx10, cursor on without blinking
#define CURSOR_BLINK_ON     0x93    //0b1001xx11, cursor on with blinking
#define TEXT_ON             0x94    //0b100101xx, text on, graphics off
#define GRAPHIC_ON          0x98    //0b100110xx, text off, graphics on
#define TEXT_GRH_ON         0x9C    //0b100111xx, text on, graphics on
#define ATTR_NORMAL         0x00    //Normal Display
#define ATTR_REVERSE        0x05    
//Reverse Di../t6963.h:189:33: error: expected ';', ',' or ')' before 'mode'splay
#define ATTR_INHIBIT        0x03    //Inhibit Display
#define ATTR_BLINK          0x08    //Blinking of Normal Display
#define ATTR_BLINK_REVERSE  0x0D    //Blinking of Reverse Display
#define ATTR_BLINK_INHIBIT  0x0B    //Blinking of Inhibit Display
#define LINE_8_CURSOR       0xA7    //8-line cursor mode
#define LINE_7_CURSOR       0xA6    //7-line cursor mode
#define LINE_6_CURSOR       0xA5    //6-line cursor mode
#define LINE_5_CURSOR       0xA4    //5-line cursor mode
#define LINE_4_CURSOR       0xA3    //4-line cursor mode
#define LINE_3_CURSOR       0xA2    //3-Line cursor mode
#define LINE_2_CURSOR       0xA1    //2-Line cursor mode
#define LINE_1_CURSOR       0xA0    //1-Line cursor mode

typedef enum
{
	NORMAL,
	PASSWORD,
	NUMENTRY,
}KEY_MODE;

#endif