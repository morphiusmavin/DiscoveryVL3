#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "screen.h"
#include "../mytypes.h"
#include "cmsis_os.h"
#include "freertos_defs.h"

extern osMessageQId SendAVRQueueHandle;

extern FORMAT_STR rtlabel_str[NUM_RT_LABELS];
extern FORMAT_STR status_label_str[NUM_STATUS_LABELS];

void init_rtlabels(void)
{
	static UCHAR col, data_col, row, str;
	int i;
	
	col = START_RT_VALUE_COL;
	data_col = col + 20;
	
	// rt labels at bottom of screen
 	for(str = 0,row = START_RT_VALUE_ROW;str < NUM_RT_LABELS;str++,row++)
	{
		rtlabel_str[str].str = str + RT_VALUES_OFFSET;
		rtlabel_str[str].row = row;
		rtlabel_str[str].col = col;
		rtlabel_str[str].data_col = data_col;

		if(row == ENDING_RT_VALUE_ROW)
		{
			row = START_RT_VALUE_ROW;
			col += RT_VALUE_COL_WIDTH;
			data_col = col + 15;
		}
	}
	row = col = 0;
	data_col = 10;

	// init labels for status above rt labels
 	for(str = 0;str < NUM_STATUS_LABELS+1;str++,row++)
	{
		status_label_str[str].str = str + STATUS_VALUES_OFFSET;
		status_label_str[str].row = row;
		status_label_str[str].col = col;
		status_label_str[str].data_col = data_col;
	}
}

// display the real time labels when:
// 1) startup
// 2) after clearing the screen
// 3) after the password process is finished

void display_rtlabels(void)
{
	static UCHAR data2, i, onoff, code;
	uint64_t avr_buffer[5];
	UCHAR ucbuff[8];

/*
	static int off_status[7] = {
			SHUTDOWN,
			BLOWER_OFF,
			OFF_FAN,
			OFF_LIGHTS,
			OFF_RUNNING_LIGHTS,
			OFF_BRIGHTS,
			OFF_BRAKES };
*/

/*
	onoff = 1;
	for(i = 0;i < 7;i++)		// start off with all the status labels at 'OFF'
	{
		code = off_status[i];
		avr_buffer[0] = EEPROM_STR2;
		avr_buffer[1] = code;
		avr_buffer[2] = onoff;
		AVR_CALL();
	}
*/
	init_rtlabels();

	ucbuff[0] = DISPLAY_RTLABELS;
	ucbuff[1] = START_RT_VALUE_ROW;
	ucbuff[2] = START_RT_VALUE_COL;
	ucbuff[3] = ENDING_RT_VALUE_ROW;
	ucbuff[4] = RT_VALUE_COL_WIDTH;
	ucbuff[5] = NUM_RT_LABELS;

	avr_buffer[0] = pack64(ucbuff);
	xQueueSend(SendAVRQueueHandle,avr_buffer,0);
	
	ucbuff[0] = DISPLAY_STATUSLABELS;

	avr_buffer[0] = pack64(ucbuff);
	xQueueSend(SendAVRQueueHandle,avr_buffer,0);

/*
		avr_buffer[0] = DISPLAY_RTLABELS;
		// starting row
		avr_buffer[1] = START_RT_VALUE_ROW;
		// starting col
		avr_buffer[2] = START_RT_VALUE_COL;
		// ending row
		avr_buffer[3] = ENDING_RT_VALUE_ROW;
		// col width
		avr_buffer[4] = RT_VALUE_COL_WIDTH;
		// num lables
		avr_buffer[5] = NUM_RT_LABELS;
		AVR_CALL();

		avr_buffer[0] = DISPLAY_STATUSLABELS;
		AVR_CALL();
*/
}

#if 0
void display_rtvalues(void)
{
    static int i;
    static UCHAR data1;

	data1 = 0x21;

	while(1)
	{
		if(key_mode == NORMAL)
		{
			// engine temp doesn't come from ADC anymore
			// 
			for(i = OIL_PRES ;i <= O2+1;i++)
			{
				avr_buffer[0] = SEND_BYTE_RT_VALUES;
				avr_buffer[1] = rtlabel_str[i].row;
				avr_buffer[2] = rtlabel_str[i].data_col;
				avr_buffer[3] = u8_pot[i];
				AVR_CALL();
			}

//	 		ESOS_TASK_WAIT_ON_AVAILABLE_OUT_COMM();
// 2 knobs on front of mbox
/*	send to serial port for testing
			for(i = 0;i < 2;i++)
				ESOS_TASK_WAIT_ON_SEND_UINT8(u8_pot[i]);

			for(i = 8;i < 11;i++)
				ESOS_TASK_WAIT_ON_SEND_UINT8(u8_pot[i]);
			ESOS_TASK_SIGNAL_AVAILABLE_OUT_COMM();
		}
*/
		}

}
#endif