/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#if 1
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "dwt_stm32_delay.h"
#include "../mytypes.h"
#include "ds1620.h"	
#include "freertos_defs.h"
#include "usart.h"
#include "screen.h"
#include "string.h"
#include "gpio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SEND_FPGA_DELAY() vTaskDelay(20)

typedef enum
{
	STATE_WAIT_FOR_PRESS = 1,
	STATE_WAIT_FOR_RELEASE
} ISRSTATE;

static ISRSTATE e_isrState;

const uint8_t au8_keyTable[NUM_ROWS][NUM_COLS] =
{
/*
	{KP_1, KP_2, KP_3, KP_A },
	{KP_4, KP_5, KP_6, KP_B },
	{KP_7, KP_8, KP_9, KP_C },
	{KP_AST, KP_0, KP_POUND, KP_D}
*/
	{KP_AST, KP_0, KP_POUND, KP_D},
	{KP_7, KP_8, KP_9, KP_C },
	{KP_4, KP_5, KP_6, KP_B },
	{KP_1, KP_2, KP_3, KP_A }
};

FORMAT_STR rtlabel_str[NUM_RT_LABELS];
FORMAT_STR status_label_str[NUM_STATUS_LABELS];
uint64_t pack64(UCHAR *buff);
uint32_t pack32(UCHAR *buff);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint64_t pack64(UCHAR *buff);
uint32_t pack32(UCHAR *buff);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static UCHAR buff[SERIAL_BUFF_SIZE+1];
static int menu_ptr;
static int lights_on;
static int brights_on;
static int blower_on;
static int fan_on;
static int engine_on;
static int running_lights_on;
static int wipers_on;
static int system_up;
static int run_time;

static KEY_MODE key_mode;
//static int key_enter_time;
static char password[PASSWORD_SIZE];
static char correct_password[PASSWORD_SIZE];
static UCHAR password_valid;
static int password_ptr;
static int password_retries;
static int dim_screen;
static int silent_key;
static char num_entry_num[SIZE_NUM];
//static int task7on;

static void clr_scr(void)
{
	UCHAR key[2];
	key[0] = LCD_CLRSCR;
	key[1] = 0xFE;
	HAL_UART_Transmit(&huart2, key, 2, 100);
}

static void goto_scr(UCHAR row, UCHAR col)
{
	UCHAR key[4];
	key[0] = GOTO_CMD;
	key[1] = row;
	key[2] = col;
	key[3] = 0xFE;
	HAL_UART_Transmit(&huart2, key, 4, 100);
}

static void setOneRowLow(uint8_t u8_x)
{
	switch (u8_x)
	{
		case 0:
			
			HAL_GPIO_WritePin(GPIOB, row0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, row1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, row2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, row3, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOB, row0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, row1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, row2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, row3, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOB, row0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, row1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, row2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, row3, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOB, row0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, row1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, row2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, row3, GPIO_PIN_SET);
			break;
		default:
			HAL_GPIO_WritePin(GPIOB, row0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, row1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, row2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, row3, GPIO_PIN_SET);
			break;
	}
}

static void drive_row_high(void)
{
	HAL_GPIO_WritePin(GPIOB, row0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, row1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, row2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, row3, GPIO_PIN_SET);
}

/*
int key_is_pressed(void)
{
	GPIO_PinState state0, state1, state2, state3;

	state0 = HAL_GPIO_ReadPin(GPIOC,col0);
	state1 = HAL_GPIO_ReadPin(GPIOC,col1);
	state2 = HAL_GPIO_ReadPin(GPIOC,col2);
	state3 = HAL_GPIO_ReadPin(GPIOC,col3);
	if (state0 == GPIO_PIN_RESET || state1 == GPIO_PIN_RESET || 
					state2 == GPIO_PIN_RESET || state3 == GPIO_PIN_RESET) 
		return 1;
	
	return 0;
}
*/

int key_is_released(void)
{
	GPIO_PinState state0, state1, state2, state3;

	state0 = HAL_GPIO_ReadPin(GPIOC,col0);
	state1 = HAL_GPIO_ReadPin(GPIOC,col1);
	state2 = HAL_GPIO_ReadPin(GPIOC,col2);
	state3 = HAL_GPIO_ReadPin(GPIOC,col3);
	if (state0 == GPIO_PIN_SET && state1 == GPIO_PIN_SET && 
					state2 == GPIO_PIN_SET && state3 == GPIO_PIN_SET) 
		return 1;
	
	return 0;
}

UCHAR do_keyscan(void)
{
	uint8_t u8_row;
	UCHAR key_pressed;
	GPIO_PinState state;

	key_pressed = 0;
//	drive_row_high();

	for (u8_row = 0; u8_row < NUM_ROWS; u8_row++)
	{
		setOneRowLow(u8_row);
//			vTaskDelay(1);
		state = HAL_GPIO_ReadPin(GPIOC,col0);
		if(state == GPIO_PIN_RESET)
		{
			key_pressed = (au8_keyTable[u8_row][0]);
			return key_pressed;
		}
		state = HAL_GPIO_ReadPin(GPIOC,col1);
		if(state == GPIO_PIN_RESET)
		{
			key_pressed = (au8_keyTable[u8_row][1]);
			return key_pressed;
		}
		state = HAL_GPIO_ReadPin(GPIOC,col2);
		if(state == GPIO_PIN_RESET)
		{
			key_pressed = (au8_keyTable[u8_row][2]);
			return key_pressed;
		}
		state = HAL_GPIO_ReadPin(GPIOC,col3);
		if(state == GPIO_PIN_RESET)
		{
			key_pressed = (au8_keyTable[u8_row][3]);
			return key_pressed;
		}
	}
	return 0;
}

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;
osThreadId myTask08Handle;
osThreadId myTask09Handle;
osThreadId myTask10Handle;
osMessageQId keypressedHandle;
osMessageQId Send7200Handle;
osMessageQId SendAVRHandle;
osMessageQId SendFPGAHandle;
osTimerId myTimer01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartAVRTask(void const * argument);
void StartTask7200(void const * argument);
void StartRecv7200(void const * argument);
void StartSendFPGA(void const * argument);
void StartRecvFPGA(void const * argument);
void StartKeyStateTask(void const * argument);
void Callback01(void const * argument);
#endif
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
#if 0
  /* definition and creation of myTask04 */
  osThreadDef(myTask04, StartAVRTask, osPriorityIdle, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, StartTask7200, osPriorityIdle, 0, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* definition and creation of myTask06 */
  osThreadDef(myTask06, StartRecv7200, osPriorityIdle, 0, 128);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

  /* definition and creation of myTask08 */
  osThreadDef(myTask08, StartSendFPGA, osPriorityIdle, 0, 128);
  myTask08Handle = osThreadCreate(osThread(myTask08), NULL);

  /* definition and creation of myTask09 */
  osThreadDef(myTask09, StartRecvFPGA, osPriorityIdle, 0, 128);
  myTask09Handle = osThreadCreate(osThread(myTask09), NULL);

  /* definition and creation of myTask10 */
  osThreadDef(myTask10, StartKeyStateTask, osPriorityIdle, 0, 128);
  myTask10Handle = osThreadCreate(osThread(myTask10), NULL);
#endif
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of keypressed */
  osMessageQDef(keypressed, 16, uint16_t);
  keypressedHandle = osMessageCreate(osMessageQ(keypressed), NULL);

  /* definition and creation of Send7200 */
  osMessageQDef(Send7200, 16, uint16_t);
  Send7200Handle = osMessageCreate(osMessageQ(Send7200), NULL);

  /* definition and creation of SendAVR */
  osMessageQDef(SendAVR, 16, uint16_t);
  SendAVRHandle = osMessageCreate(osMessageQ(SendAVR), NULL);

  /* definition and creation of SendFPGA */
  osMessageQDef(SendFPGA, 16, uint16_t);
  SendFPGAHandle = osMessageCreate(osMessageQ(SendFPGA), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  	int i;
	uint64_t avr_buffer[5];
	UCHAR ucbuff[8];
	UCHAR key;
	UCHAR cmd;
	uint16_t recval;

	lights_on = 0;
	brights_on = 0;
	fan_on = 0;
	blower_on = 0;
	engine_on = 0;
	wipers_on = 0;
  	menu_ptr = 0;
    password_valid = 0;
    password_ptr = 0;
	password_retries = 0;
	dim_screen = 0;
	silent_key = 0;
	system_up = 1;
	run_time = 0;
//	key_mode = PASSWORD;
	key_mode = NORMAL;
	DWT_Delay_Init();
	init_DS1620();
//	set_output();		// set the DS1620 lines to outputs (for testing)

	memset(correct_password,0,sizeof(correct_password));
	strcpy(correct_password,"2354795\0");
	memset(password,0,PASSWORD_SIZE);
	osTimerStart(myTimer01Handle,10000);

	for(;;)
	{
		vTaskDelay(1000);
	}

	menu_ptr = 0;
/*  
  	for(i = 0;i < 20;i++)
	{
  		if(menu_ptr == 0)
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
			vTaskDelay(50);
			menu_ptr = 1;
		}else
		{
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			vTaskDelay(50);
			menu_ptr = 0;
		}
	}
*/
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

	vTaskDelay(2000);

	clr_scr();
	vTaskDelay(1000);

	init_rtlabels();
//	display_rtlabels();	

	vTaskDelay(10);

	ucbuff[0] = DISPLAY_RTLABELS;
	ucbuff[1] = START_RT_VALUE_ROW;
	ucbuff[2] = START_RT_VALUE_COL;
	ucbuff[3] = ENDING_RT_VALUE_ROW;
	ucbuff[4] = RT_VALUE_COL_WIDTH;
	ucbuff[5] = NUM_RT_LABELS;

	avr_buffer[0] = pack64(ucbuff);
//	xQueueSend(SendAVRHandle,avr_buffer,0);

	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
  /* Infinite loop */

  
  for(;;)
  {
		xQueueReceive(keypressedHandle, &recval, portMAX_DELAY);
		key = (UCHAR)recval;
		cmd = 0;

		switch(key)
		{
			case KP_1:
				cmd = START_SEQ;
				if(engine_on == 0)
				{
					ucbuff[0] = cmd;
					buff[0] = pack64(ucbuff);
					xQueueSend(Send7200Handle, buff, 0);
					engine_on = 1;
				}
			break;
			case KP_2:
				cmd = SHUTDOWN;
				ucbuff[0] = cmd;
				buff[0] = pack64(ucbuff);
				xQueueSend(Send7200Handle, buff, 0);
				cmd = OFF_FAN;
				ucbuff[0] = cmd;
				buff[0] = pack64(ucbuff);
				xQueueSend(Send7200Handle, buff, 0);
				engine_on = 0;
				fan_on = 0;

			break;
			case KP_3:
				if(lights_on == 1)
				{
					if(brights_on == 0)
					{
						cmd = ON_BRIGHTS;
						ucbuff[0] = cmd;
						buff[0] = pack64(ucbuff);
						xQueueSend(Send7200Handle, buff, 0);
						brights_on = 1;
					}
					else
					{
						cmd = OFF_BRIGHTS;
						ucbuff[0] = cmd;
						buff[0] = pack64(ucbuff);
						xQueueSend(Send7200Handle, buff, 0);
						brights_on = 0;
					}
				}
			break;
			case KP_4:
				if(fan_on == 0)
				{
					cmd = ON_FAN;
					ucbuff[0] = cmd;
					buff[0] = pack64(ucbuff);
					xQueueSend(Send7200Handle, buff, 0);
					fan_on = 1;
				}
				else
				{
					cmd = OFF_FAN;
					ucbuff[0] = cmd;
					buff[0] = pack64(ucbuff);
					xQueueSend(Send7200Handle, buff, 0);
					fan_on = 0;
				}
			break;
			case KP_5:
				switch(blower_on)
				{
					case 0:	
						cmd = BLOWER1;
						ucbuff[0] = cmd;
						buff[0] = pack64(ucbuff);
						xQueueSend(Send7200Handle, buff, 0);
						blower_on = 1;
						break;
					case 1:
						cmd = BLOWER2;
						ucbuff[0] = cmd;
						buff[0] = pack64(ucbuff);
						xQueueSend(Send7200Handle, buff, 0);
						blower_on = 2;
						break;
					case 2:
						cmd = BLOWER3;
						ucbuff[0] = cmd;
						buff[0] = pack64(ucbuff);
						xQueueSend(Send7200Handle, buff, 0);
						blower_on = 3;
						break;
					case 3:
						cmd = BLOWER_OFF;
						ucbuff[0] = cmd;
						buff[0] = pack64(ucbuff);
						xQueueSend(Send7200Handle, buff, 0);
						blower_on = 0;
						break;
					default:
						blower_on = 0;
						break;
				}
			break;
			case KP_6:
				if(running_lights_on == 0)
				{
					running_lights_on = 1;
					cmd = ON_RUNNING_LIGHTS;
					ucbuff[0] = cmd;
					buff[0] = pack64(ucbuff);
					xQueueSend(Send7200Handle, buff, 0);
				}else
				{
					running_lights_on = 0;
					cmd = OFF_RUNNING_LIGHTS;
					ucbuff[0] = cmd;
					buff[0] = pack64(ucbuff);
					xQueueSend(Send7200Handle, buff, 0);
				}
			break;
			case KP_7:
				if(lights_on == 0)
				{
					cmd = ON_LIGHTS;
					ucbuff[0] = cmd;
					buff[0] = pack64(ucbuff);
					xQueueSend(Send7200Handle, buff, 0);
					lights_on = 1;
				}
				else
				{
					cmd = OFF_LIGHTS;
					lights_on = 0;
					ucbuff[0] = cmd;
					buff[0] = pack64(ucbuff);
					xQueueSend(Send7200Handle, buff, 0);

					if(brights_on == 1)
					{
						cmd = OFF_BRIGHTS;
						ucbuff[0] = cmd;
						buff[0] = pack64(ucbuff);
						xQueueSend(Send7200Handle, buff, 0);
						brights_on = 0;
					}
				}
			break;
			case KP_8:
				switch (wipers_on)
				{
					case 0:
						cmd = WIPER1;
						wipers_on = 1;
						break;
					case 1:
						cmd = WIPER2;
						wipers_on = 2;
						break;
					case 2:
						cmd = WIPER_OFF;
						wipers_on = 0;
						break;
					default:
						wipers_on = 0;
						break;
				}
				ucbuff[0] = cmd;
				buff[0] = pack64(ucbuff);
				xQueueSend(Send7200Handle, buff, 0);
			break;
			case KP_9:
			break;
		}
		vTaskDelay(10);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartAVRTask */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAVRTask */
void StartAVRTask(void const * argument)
{
  /* USER CODE BEGIN StartAVRTask */
	uint64_t avr_buffer[5];
	UCHAR ucbuff[8];
	UCHAR end_byte;
//	uint64_t temp;

  /* Infinite loop */
	for(;;)
	{
		xQueueReceive(SendAVRHandle, avr_buffer, portMAX_DELAY);

		ucbuff[0] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		ucbuff[1] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		ucbuff[2] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		ucbuff[3] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		ucbuff[4] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		ucbuff[5] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		ucbuff[6] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		ucbuff[7] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		switch (ucbuff[0])
		{
			case EEPROM_STR:
			case EEPROM_STR2:
			case DISPLAY_TEMP:
			case SEND_INT_RT_VALUES:
			case DISPLAY_ELAPSED_TIME:
			case SET_MODE_CMD:
				HAL_UART_Transmit(&huart2, ucbuff, 5, 100);
				break;
			case CHAR_CMD:
			case SET_NUM_ENTRY_MODE:
			case PASSWORD_MODE:
				HAL_UART_Transmit(&huart2, ucbuff, 2, 100);
				break;
			case GOTO_CMD:
				HAL_UART_Transmit(&huart2, ucbuff, 3, 100);
				break;
			case DISPLAY_RTLABELS:
				HAL_UART_Transmit(&huart2, ucbuff, 6, 100);
				break;
			case SEND_BYTE_HEX_VALUES:
			case SEND_BYTE_RT_VALUES:
				HAL_UART_Transmit(&huart2, ucbuff, 4, 100);
				break;
			case LCD_CLRSCR:
			case DISPLAY_STATUSLABELS:
				HAL_UART_Transmit(&huart2, ucbuff, 1, 100);
				break;
			default:
				break;
		}
		end_byte = 0xFE;
		HAL_UART_Transmit(&huart2, &end_byte, 1, 100);
		end_byte = 0;

		do {
			HAL_UART_Receive(&huart2, &end_byte, 1, portMAX_DELAY);
		} while(end_byte != 0xFD);
	}
  /* USER CODE END StartAVRTask */
}

/* USER CODE BEGIN Header_StartTask7200 */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask7200 */
void StartTask7200(void const * argument)
{
  /* USER CODE BEGIN StartTask7200 */
  	uint64_t avr_buffer[5];
	UCHAR ucbuff[SERIAL_BUFF_SIZE];
	UCHAR marker = 0xFF;

	memset(ucbuff,0,sizeof(ucbuff));

	/* Infinite loop */
	for(;;)
	{
		xQueueReceive(Send7200Handle, avr_buffer, portMAX_DELAY);
/*
		if(menu_ptr == 0)
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
			menu_ptr = 1;
		}else
		{
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			menu_ptr = 0;
		}
*/
		ucbuff[0] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		ucbuff[1] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		ucbuff[2] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		ucbuff[3] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		ucbuff[4] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		ucbuff[5] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		ucbuff[6] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		ucbuff[7] = (UCHAR)avr_buffer[0];
		avr_buffer[0] >>= 8;

		if(system_up == 1)
		//if(1)
		{
			HAL_UART_Transmit(&huart1, &marker, 1, 100);
			HAL_UART_Transmit(&huart1, &ucbuff[0], SERIAL_BUFF_SIZE, 100);
		}
		//		HAL_UART_Transmit_IT(&huart1, &marker, 1);
		//		HAL_UART_Transmit_IT(&huart1, &ucbuff[0], SERIAL_BUFF_SIZE);
		/*
		HAL_UART_Receive(&huart1, &Recv7200buff[0], SERIAL_BUFF_SIZE, 1000);

		ucbuff[0] = SEND_BYTE_RT_VALUES;
		ucbuff[1] = rtlabel_str[RUN_TIME].row;
		ucbuff[2] = rtlabel_str[RUN_TIME].data_col;
		ucbuff[3] = Recv7200buff[0];		// hours
		avr_buffer[0] = pack64(ucbuff);
		xQueueSend(SendAVRHandle,avr_buffer,0);
		vTaskDelay(5);

		ucbuff[1] = rtlabel_str[RUN_TIME].row;
		ucbuff[2] = rtlabel_str[RUN_TIME].data_col + 3;
		ucbuff[3] = Recv7200buff[1];		// minutes
		avr_buffer[0] = pack64(ucbuff);
		xQueueSend(SendAVRHandle,avr_buffer,0);
		vTaskDelay(5);

		ucbuff[1] = rtlabel_str[RUN_TIME].row;
		ucbuff[2] = rtlabel_str[RUN_TIME].data_col + 6;
		ucbuff[3] = Recv7200buff[2];		// seconds
		avr_buffer[0] = pack64(ucbuff);
		xQueueSend(SendAVRHandle,avr_buffer,0);
		vTaskDelay(5);

		vTaskDelay(100);

		if(menu_ptr == 0)
		{
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
		menu_ptr = 1;
		}else
		{
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		menu_ptr = 0;
		}
		*/
	}
  /* USER CODE END StartTask7200 */
}

/* USER CODE BEGIN Header_StartRecv7200 */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRecv7200 */
void StartRecv7200(void const * argument)
{
  /* USER CODE BEGIN StartRecv7200 */
	UCHAR cmd;
	int j = 99;
	int k = 0;
	UCHAR buff[SERIAL_BUFF_SIZE];
  	uint64_t avr_buffer[5];
	UCHAR ucbuff[SERIAL_BUFF_SIZE];
	UCHAR marker = 0xFF;

	/* Infinite loop */
	for(;;)
	{
		HAL_UART_Receive_IT(&huart1, &cmd, 1);
		if(cmd >= ENABLE_START && cmd <= SEND_TIME_DATA)
		{	
			switch (cmd)
			{
				case 	ON_ACC:				// 3
					break;
				case 	OFF_ACC:			// 4
					break;
				case 	ON_FUEL_PUMP:		// 5
					break;
				case 	OFF_FUEL_PUMP:		// 6
					break;
				case 	ON_FAN:				// 7
					fan_on = 1;
					break;
				case 	OFF_FAN:			// 8
					fan_on = 0;
					break;
				case 	ON_LIGHTS:			// 9
					lights_on = 1;
					break;
				case 	OFF_LIGHTS:			// 10
					lights_on = 0;
					break;
				case 	ON_BRIGHTS:			// 11
					brights_on = 1;
					break;
				case 	OFF_BRIGHTS:		// 12
					brights_on = 0;
					break;
				case 	ON_BRAKES:
					break;
				case 	OFF_BRAKES:
					break;
				case 	ON_RUNNING_LIGHTS:
					running_lights_on = 1;
					break;
				case 	OFF_RUNNING_LIGHTS:
					running_lights_on = 0;
					break;
				case 	SPECIAL_CMD:
					break;
				case 	START_SEQ:
					engine_on = 1;
					break;
				case 	SHUTDOWN:
					engine_on = 0;
					break;
				case 	ON_LLIGHTS:
					break;
				case 	OFF_LLIGHTS:
					break;
				case 	ON_LBRIGHTS:
					break;
				case 	OFF_LBRIGHTS:
					break;
				case 	ON_RLIGHTS:
					break;
				case 	OFF_RLIGHTS:
					break;
				case 	ON_RBRIGHTS:
					break;
				case 	OFF_RBRIGHTS:
					break;
				case 	BLOWER_OFF:
					blower_on = 0;
					break;
				case 	BLOWER1:
					blower_on = 1;
					break;
				case 	BLOWER2:
					blower_on = 2;
					break;
				case 	BLOWER3:
					blower_on = 3;
					break;
				case 	WIPER1:
					wipers_on = 1;
					break;
				case 	WIPER2:
					wipers_on = 2;
					break;
				case 	WIPER_OFF:
					wipers_on = 0;
					break;
				case START_MBOX_XMIT:	
					HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
					system_up = 1;
					break;
				case STOP_MBOX_XMIT:	
					HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
					system_up = 0;
					break;
				case SEND_TIME_DATA:
#if 0
					HAL_UART_Receive_IT(&huart1, &buff[0], SERIAL_BUFF_SIZE);
					if(buff[0] == 0xAA && buff[1] == 0x55)
					{
						if(menu_ptr == 0)
						{
						HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
						menu_ptr = 1;
						}else
						{
						HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
						menu_ptr = 0;
						}
					}


/*
running_hours,minutes,seconds,
total running_hours,minutes,seconds,
odometer_high,low, trip_high,low
*/

					ucbuff[0] = SEND_BYTE_RT_VALUES;
					ucbuff[1] = rtlabel_str[RUN_TIME].row;
					ucbuff[2] = rtlabel_str[RUN_TIME].data_col;
					ucbuff[3] = buff[1];		// hours
					avr_buffer[0] = pack64(ucbuff);
					xQueueSend(SendAVRHandle,avr_buffer,0);
					vTaskDelay(5);

					ucbuff[1] = rtlabel_str[RUN_TIME].row;
					ucbuff[2] = rtlabel_str[RUN_TIME].data_col + 3;
					ucbuff[3] = buff[2];		// minutes
					avr_buffer[0] = pack64(ucbuff);
					xQueueSend(SendAVRHandle,avr_buffer,0);
					vTaskDelay(5);

					ucbuff[1] = rtlabel_str[RUN_TIME].row;
					ucbuff[2] = rtlabel_str[RUN_TIME].data_col + 6;
					ucbuff[3] = buff[3];		// seconds
					avr_buffer[0] = pack64(ucbuff);
					xQueueSend(SendAVRHandle,avr_buffer,0);
					vTaskDelay(5);
					
					ucbuff[0] = SEND_INT_RT_VALUES;
					ucbuff[1] = rtlabel_str[ODOM].row;
					ucbuff[2] = rtlabel_str[ODOM].data_col;
					ucbuff[3] = buff[7];
					ucbuff[4] = buff[8];
					avr_buffer[0] = pack64(ucbuff);
					xQueueSend(SendAVRHandle,avr_buffer,0);
					vTaskDelay(10);

					ucbuff[0] = SEND_INT_RT_VALUES;
					ucbuff[1] = rtlabel_str[TRIP].row;
					ucbuff[2] = rtlabel_str[TRIP].data_col;
					ucbuff[3] = buff[9];
					ucbuff[4] = buff[10];
					avr_buffer[0] = pack64(ucbuff);
					xQueueSend(SendAVRHandle,avr_buffer,0);
#endif
					break;
					
				default:
					break;
			}
		}
		cmd = 0;
		vTaskDelay(50);
	}
  /* USER CODE END StartRecv7200 */
}

/* USER CODE BEGIN Header_StartSendFPGA */
/**
* @brief Function implementing the myTask08 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendFPGA */
void StartSendFPGA(void const * argument)
{
  /* USER CODE BEGIN StartSendFPGA */
	UCHAR cmd, param1, param2, param3, param4, param5, param6, param7;
	uint64_t rec_val[5];

	/* Infinite loop */
	for(;;)
	{
		xQueueReceive(SendFPGAHandle, rec_val, portMAX_DELAY);

		cmd = (UCHAR)rec_val[0];
		rec_val[0] >>= 8;

		param1 = (UCHAR)rec_val[0];
		rec_val[0] >>= 8;

		param2 = (UCHAR)rec_val[0];
		rec_val[0] >>= 8;

		param3 = (UCHAR)rec_val[0];
		rec_val[0] >>= 8;

		param4 = (UCHAR)rec_val[0];
		rec_val[0] >>= 8;

		param5 = (UCHAR)rec_val[0];
		rec_val[0] >>= 8;

		param6 = (UCHAR)rec_val[0];
		rec_val[0] >>= 8;

		param7 = (UCHAR)rec_val[0];

		HAL_GPIO_WritePin(GPIOC, CmdParam_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_SET);
		SEND_FPGA_DELAY();
		HAL_UART_Transmit(&huart3, &cmd, 1, 100);

		SEND_FPGA_DELAY();
		HAL_GPIO_WritePin(GPIOC, CmdParam_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_RESET);
		SEND_FPGA_DELAY();

		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_SET);
		SEND_FPGA_DELAY();
		HAL_UART_Transmit(&huart3, &param1, 1, 100);
		SEND_FPGA_DELAY();
		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_RESET);
		SEND_FPGA_DELAY();

		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_SET);
		SEND_FPGA_DELAY();
		HAL_UART_Transmit(&huart3, &param2, 1, 100);
		SEND_FPGA_DELAY();
		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_RESET);
		SEND_FPGA_DELAY();

		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_SET);
		SEND_FPGA_DELAY();
		HAL_UART_Transmit(&huart3, &param3, 1, 100);
		SEND_FPGA_DELAY();
		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_SET);
		SEND_FPGA_DELAY();
		HAL_UART_Transmit(&huart3, &param4, 1, 100);
		SEND_FPGA_DELAY();
		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_SET);
		SEND_FPGA_DELAY();
		HAL_UART_Transmit(&huart3, &param5, 1, 100);
		SEND_FPGA_DELAY();
		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_SET);
		SEND_FPGA_DELAY();
		HAL_UART_Transmit(&huart3, &param6, 1, 100);
		SEND_FPGA_DELAY();
		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_SET);
		SEND_FPGA_DELAY();
		HAL_UART_Transmit(&huart3, &param7, 1, 100);
		SEND_FPGA_DELAY();
		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_RESET);

		param7 = 0xFF;
		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_SET);
		SEND_FPGA_DELAY();
		HAL_UART_Transmit(&huart3, &param7, 1, 100);
		SEND_FPGA_DELAY();
		HAL_GPIO_WritePin(GPIOC, DataReady_Pin, GPIO_PIN_RESET);
	}

  /* USER CODE END StartSendFPGA */
}

/* USER CODE BEGIN Header_StartRecvFPGA */
/**
* @brief Function implementing the myTask09 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRecvFPGA */
void StartRecvFPGA(void const * argument)
{
  /* USER CODE BEGIN StartRecvFPGA */
	int i = 0;
	/* Infinite loop */
	UCHAR xbyte = 0;
	GPIO_PinState state;
	uint64_t avr_buffer[5];
	UCHAR ucbuff[8];
	UCHAR buff[5];
	int frame_ptr;
	UINT rpm, mph;
	rpm = 500;
	mph = 1;
	
	xbyte = 0x21;


	vTaskDelay(2000);
	HAL_GPIO_WritePin(GPIOA, PP_ACK_Pin, GPIO_PIN_SET);
	vTaskDelay(10);
	HAL_GPIO_WritePin(GPIOA, PP_ACK_Pin, GPIO_PIN_RESET);
	vTaskDelay(1);
	
	frame_ptr = 0;

  /* Infinite loop */
	
	for(;;)
	{
		do {
			state = HAL_GPIO_ReadPin(GPIOA,PP_CS_Pin);
		}while(state != GPIO_PIN_SET);

		
		state = HAL_GPIO_ReadPin(GPIOB,PP0_Pin);
		if(state == GPIO_PIN_SET)
			xbyte |= 1;
		state = HAL_GPIO_ReadPin(GPIOB,PP1_Pin);
		if(state == GPIO_PIN_SET)
			xbyte |= 0x2;
		state = HAL_GPIO_ReadPin(GPIOB,PP2_Pin);
		if(state == GPIO_PIN_SET)
			xbyte |= 0x4;
		state = HAL_GPIO_ReadPin(GPIOB,PP3_Pin);
		if(state == GPIO_PIN_SET)
			xbyte |= 0x8;
		state = HAL_GPIO_ReadPin(GPIOB,PP4_Pin);
		if(state == GPIO_PIN_SET)
			xbyte |= 0x10;
		state = HAL_GPIO_ReadPin(GPIOB,PP5_Pin);
		if(state == GPIO_PIN_SET)
			xbyte |= 0x20;
		state = HAL_GPIO_ReadPin(GPIOB,PP6_Pin);
		if(state == GPIO_PIN_SET)
			xbyte |= 0x40;
		state = HAL_GPIO_ReadPin(GPIOA,PP7_Pin);
		if(state == GPIO_PIN_SET)
			xbyte |= 0x80;

		do {
			state = HAL_GPIO_ReadPin(GPIOA,PP_CS_Pin);
		}while(state != GPIO_PIN_RESET);

		if(xbyte == 0xFE)
		{
			frame_ptr = 0;

			ucbuff[0] = SEND_INT_RT_VALUES;
			ucbuff[1] = rtlabel_str[RPM].row;
			ucbuff[2] = rtlabel_str[RPM].data_col;
//			ucbuff[3] = (UCHAR)(rpm >> 8);
//			ucbuff[4] = (UCHAR)rpm;
			ucbuff[4] = buff[0];
			ucbuff[3] = buff[1];
			avr_buffer[0] = pack64(ucbuff);
			xQueueSend(SendAVRHandle,avr_buffer,0);
			vTaskDelay(10);

			ucbuff[1] = rtlabel_str[MPH].row;
			ucbuff[2] = rtlabel_str[MPH].data_col;
//			ucbuff[3] = (UCHAR)(mph >> 8);
//			ucbuff[4] = (UCHAR)mph;
			ucbuff[4] = buff[2];
			ucbuff[3] = buff[3];
			avr_buffer[0] = pack64(ucbuff);
			xQueueSend(SendAVRHandle,avr_buffer,0);

			if((rpm+= 12) > 5000)
				rpm = 500;
			if(++mph > 100)
				mph = 1;

			vTaskDelay(10);

			if(engine_on != 0)
			{
				ucbuff[0] = SEND_RT_VALUES;
/*
				ucbuff[1] = (UCHAR)(rpm >> 8);
				ucbuff[2] = (UCHAR)rpm;
				ucbuff[3] = (UCHAR)(mph >> 8);
				ucbuff[4] = (UCHAR)mph;
*/
				ucbuff[2] = buff[0];
				ucbuff[1] = buff[1];
				ucbuff[4] = buff[2];
				ucbuff[3] = buff[3];

				avr_buffer[0] = pack64(ucbuff);
				xQueueSend(Send7200Handle, avr_buffer, 0);
			}
		}else
		{
			buff[frame_ptr] = xbyte;
			frame_ptr++;
		}
		HAL_GPIO_WritePin(GPIOA, PP_ACK_Pin, GPIO_PIN_SET);
		vTaskDelay(5);
		HAL_GPIO_WritePin(GPIOA, PP_ACK_Pin, GPIO_PIN_RESET);
		xbyte = 0;
	}
  /* USER CODE END StartRecvFPGA */
}

/* USER CODE BEGIN Header_StartKeyStateTask */
/**
* @brief Function implementing the myTask10 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeyStateTask */
void StartKeyStateTask(void const * argument)
{
  /* USER CODE BEGIN StartKeyStateTask */
 	static UCHAR key;
	unsigned long sendval;
  
	e_isrState = STATE_WAIT_FOR_PRESS;  

	vTaskDelay(1000);
  /* Infinite loop */
	for(;;)
	{
		switch (e_isrState)
		{
			case STATE_WAIT_FOR_PRESS:
				key = do_keyscan();
				if(key != 0)
				{
					e_isrState = STATE_WAIT_FOR_RELEASE;
				}
				break;

			case STATE_WAIT_FOR_RELEASE:
				//keypad released
				if(key_is_released() == 1)
				{
					e_isrState = STATE_WAIT_FOR_PRESS;
					drive_row_high();
					sendval = (unsigned long)key;
					xQueueSend(keypressedHandle, &sendval, 0);
					key = 0;
				}
				break;
			default:
				e_isrState = STATE_WAIT_FOR_PRESS;
				break;
		}
		vTaskDelay(1);
	}
  /* USER CODE END StartKeyStateTask */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */
#if 0

	UINT raw_data, temp_raw;
	uint64_t avr_buffer[5];
	UCHAR ucbuff[8];

//	initDS1620();

//	row = col = 0;
	raw_data = 43;		// test using 70F
//	rpm = 500;
//	mph = 1;
	memset(ucbuff, 0, 8);

	raw_data = 1;
	temp_raw = 42;

		ucbuff[0] = DISPLAY_TEMP;
		ucbuff[1] = rtlabel_str[ENG_TEMP].row;
		ucbuff[2] = rtlabel_str[ENG_TEMP].data_col;
		ucbuff[4] = (UCHAR)temp_raw;
		temp_raw >>= 8;
		ucbuff[3] = (UCHAR)temp_raw;	// send high byte 1st
		avr_buffer[0] = pack64(ucbuff);
		xQueueSend(SendAVRHandle,avr_buffer,0);
		vTaskDelay(10);
		
		ucbuff[0] = ENGINE_TEMP;
		ucbuff[1] = ucbuff[3];			// send high byte 1st
		ucbuff[2] = ucbuff[4];
		avr_buffer[0] = pack64(ucbuff);
		xQueueSend(Send7200Handle, avr_buffer, 0);
		vTaskDelay(1000);

		writeByteTo1620(DS1620_CMD_STARTCONV);
		vTaskDelay(30);
		raw_data = readTempFrom1620();
		temp_raw = raw_data;
		vTaskDelay(30);
		writeByteTo1620(DS1620_CMD_STOPCONV);
		vTaskDelay(100);

		raw_data++;				// used for testing
		if(raw_data > 230)
			raw_data = 10;
		temp_raw = raw_data;
#endif
		
	if(menu_ptr == 0)
	{
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
		menu_ptr = 1;

		HAL_GPIO_WritePin(GPIOB, DS1620_PIN_CLK, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, DS1620_PIN_DQ, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, DS1620_PIN_RST, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOC, Test_Pin, GPIO_PIN_RESET);
	}else
	{
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOB, DS1620_PIN_CLK, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, DS1620_PIN_DQ, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, DS1620_PIN_RST, GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOC, Test_Pin, GPIO_PIN_SET);
		menu_ptr = 0;
	}

  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
/*
		if(menu_ptr == 0)
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
			menu_ptr = 1;
		}else
		{
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			menu_ptr = 0;
		}
*/
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UCHAR cmd;

	if(huart->Instance == USART1)
	{
/*
		if(menu_ptr == 0)
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
			menu_ptr = 1;
		}else
		{
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			menu_ptr = 0;
		}
*/
	}
}
uint64_t pack64(UCHAR *buff)
{
	uint64_t var64;

	var64 = (uint64_t)buff[7];
	var64 <<= 8;
	var64 |= (uint64_t)buff[6];
	var64 <<= 8;
	var64 |= (uint64_t)buff[5];
	var64 <<= 8;
	var64 |= (uint64_t)buff[4];
	var64 <<= 8;
	var64 |= (uint64_t)buff[3];
	var64 <<= 8;
	var64 |= (uint64_t)buff[2];
	var64 <<= 8;
	var64 |= (uint64_t)buff[1];
	var64 <<= 8;
	var64 |= (uint64_t)buff[0];
	return var64;
}
uint32_t pack32(UCHAR *buff)
{
	uint32_t var32;
	UCHAR temp;

	var32 = 0L;
	var32 |= (uint32_t)buff[3];
	var32 <<= 8;
	var32 |= (uint32_t)buff[2];
	var32 <<= 8;
	var32 |= (uint32_t)buff[1];
	var32 <<= 8;
	var32 |= (uint32_t)buff[0];

	return var32;
}
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
