/*
 * loadcell.c
 *
 *  Created on: Apr 1, 2021
 *      Author: ParkChoungHan */


/* USER CODE BEGIN Includes */
#include "EMG10K.h"
#include "dwt_stm32_delay.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
/* USER CODE BEGIN Includes */

/* STM32 system struct */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;


/* USER CODE BEGIN PV */
uint8_t RearLine [REAR_COUNT], RearLine_Prt [REAR_COUNT], Rear_Count = 1, Rear_Count_Prt = 1, send_cnt;
uint8_t data_strem[1004]={0}, init_run_data[64]={0}, RUN_ACTIVE_NUM = 0x00;

uint32_t sample, arrNumbers[500] = {0}, pos = 0, filtereadValue=0, sensorValue;
long sum = 0;
uint32_t len = sizeof(arrNumbers) / sizeof(uint32_t);

uint32_t SOL_SELECT = 0;

uint8_t G_PACKER_CNT[MAX_GRADE], P_SOL_CNT[MAX_PACKER], GRADE_PACKER[MAX_PACKER][MAX_PACKER];
uint32_t timerCnt,  Grade_speed;

bool send_count = false, sendvalue = false;

uint8_t COM_MOD;
uint8_t rxtemp;

uint32_t RxLength, revcnt;
uint8_t rx_data[256], rx_data_[256], run_data[256],rx_data_tmp[256];

uint8_t txbuff[100];
uint8_t txbuff_SEND[1024];

uint32_t msTimer;
uint32_t interuptCnt=0, startTime=0 , endTime=0;

uint8_t GRADE_PACKER_SELECT_DOUBLE;
uint8_t PACKER_SOL_SELECT_DOUBLE[MAX_PACKER_SOL];

/* ------------------Loadcell params--------------*/
unsigned int avg = 0;
unsigned int avg_real = 0;
unsigned int avg_volt = 0;
float avg_float = 0;
unsigned int time_count = 0;
/* ---------------------END-----------------------*/

/* ------------------BUZZER_ON_OFF--------------*/
void BUZZER_ON_OFF()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); HAL_Delay(100);
}

/* ------------------FLASH_UPDATAE--------------*/
void FLASH_UPDATE(uint8_t select)
{
	uint32_t *ptr ;
	HAL_FLASH_Unlock();
	uint32_t PAGEError = 0;
	uint32_t *Address;
	uint32_t Data;
	uint32_t i;

	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;

	switch (select)
	{
		case 1:
			EraseInitStruct.PageAddress = GRADE_INFO_START_ADDR;
			EraseInitStruct.NbPages     = (GRADE_INFO_END_ADDR - GRADE_INFO_START_ADDR) / FLASH_PAGE_SIZE;
			HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
			ptr = (uint32_t*)&GRADE_INFO;
			for( i=0 ; i < sizeof(GRADE_INFO); i++)
			{
				Address = (uint32_t*)GRADE_INFO_START+i;
				Data = *((uint32_t*)ptr+i);
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)Address,Data);
			}
			break;

		case 2:
			EraseInitStruct.PageAddress = PACKER_INFO_START_ADDR;
			EraseInitStruct.NbPages     = (PACKER_INFO_END_ADDR - PACKER_INFO_START_ADDR) / FLASH_PAGE_SIZE;
			HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
			ptr = (uint32_t*)&PACKER_INFO;
			for( i=0 ; i < sizeof(PACKER_INFO); i++)
			{
				Address = (uint32_t*)PACKER_INFO_START+i;
				Data = *((uint32_t*)ptr+i);
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)Address,Data);
			}
			break;

		case 3:
			EraseInitStruct.PageAddress = MARKING_INFO_START_ADDR;
			EraseInitStruct.NbPages     = (MARKING_INFO_END_ADDR - MARKING_INFO_START_ADDR) / FLASH_PAGE_SIZE;
			HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
			ptr = (uint32_t*)&MARKING_INFO ;
			for( i=0 ; i < sizeof(MARKING_INFO); i++)
			{
				Address = (uint32_t*)MARKING_INFO_START+i;
				Data = *((uint32_t*)ptr+i);
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)Address,Data);
			}
			break;

		case 4:
			EraseInitStruct.PageAddress = SOL_RUN_TIME_INFO_START_ADDR;
			EraseInitStruct.NbPages     = (SOL_RUN_TIME_INFO_END_ADDR - SOL_RUN_TIME_INFO_START_ADDR) / FLASH_PAGE_SIZE;
			HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
			ptr = (uint32_t*)&SOL_RUN_TIME_INFO;
			for( i=0 ; i < sizeof(SOL_RUN_TIME_INFO); i++)
			{
				Address = (uint32_t*)SOL_RUN_TIME_INFO_START+i;
				Data = *((uint32_t*)ptr+i);
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)Address,Data);
			}
			break;

		case 5:
			EraseInitStruct.PageAddress = MEASUREMENT_INFO_START_ADDR;
			EraseInitStruct.NbPages     = (MEASUREMENT_INFO_END_ADDR - MEASUREMENT_INFO_START_ADDR) / FLASH_PAGE_SIZE;
			HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
			ptr = (uint32_t*)&MEASUREMENT_INFO;
			for( i=0 ; i < sizeof(MEASUREMENT_INFO); i++)
			{
				Address = (uint32_t*)MEASUREMENT_INFO_START +i;
				Data = *((uint32_t*)ptr+i);
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)Address,Data);
			}
			break;

		case 6:
			EraseInitStruct.PageAddress = LOADCELL_INFO_START_ADDR;
			EraseInitStruct.NbPages     = (LOADCELL_INFO_END_ADDR - LOADCELL_INFO_START_ADDR) / FLASH_PAGE_SIZE;
			HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
			ptr = (uint32_t*)&LOADCELL_INFO;
			for( i=0 ; i < sizeof(LOADCELL_INFO); i++)
			{
				Address = (uint32_t*)LOADCELL_INFO_START+i;
				Data = *((uint32_t*)ptr+i);
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)Address,Data);
			}
			break;
		}
  		HAL_FLASH_Lock(); // lock
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); HAL_Delay(100);
}

/* ------------------SOL RUN--------------*/
void SOL_RUN_SELECTE(uint8_t selcte1)
{
//  int selcte = selcte1;

	switch(selcte1)
	{
		case 1:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			break;
		case 8:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			break;
		case 9:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			break;
		case 10:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			break;
		case 11:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
			break;
		case 12:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
			break;
		case 13:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case 14:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			break;
		case 15:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
			break;
		case 16:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
			break;
		case 17:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case 18:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			break;
		case 19:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			break;
		case 20:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			break;
		case 21:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			break;
		case 22:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			break;
		case 23:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case 24:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
			break;
	}
}
/* ------------------------------ RUN SOL TEST------------------------*/
void SOL_RUN_(uint8_t selcte1)
{
  int selcte =(int)selcte1;

	switch(selcte)
	{
			case 1:
				/*-------------SOL_1-1-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
				break;
			case 2:
				/*-------------SOL_1-2-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
				break;
			case 3:
				/*-------------SOL_1-3-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
				break;
			case 4:
				/*-------------SOL_1-4-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
			  break;
			case 5:
				/*-------------SOL_1-5-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
			 break;
			case 6:
				/*-------------SOL_1-6-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			  break;
			case 7:
				/*-------------SOL_1-7-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			  break;
			case 8:
				/*-------------SOL_1-8-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
			  break;
			case 9:
				/*-------------SOL_2-1-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
				break;
			case 10:
				/*-------------SOL_2-2-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
				break;
			case 11:
				/*-------------SOL_2-3-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
			case 12:
				/*-------------SOL_2-4-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
				break;
			case 13:
				/*-------------SOL_2-5-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
				break;
			case 14:
				/*-------------SOL_2-6-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				break;
			case 15:
				/*-------------SOL_2-7-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
				break;
			case 16:
				/*-------------SOL_2-8-------------------*/
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
				break;
			case 17:
				/*-------------SOL_3-1-------------------*/
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
				break;
			case 18:
				/*-------------SOL_3-2-------------------*/
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
				break;
			case 19:
				/*-------------SOL_3-3-------------------*/
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
				break;
			case 20:
				/*-------------SOL_3-4-------------------*/
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
				break;
			case 21:
				/*-------------SOL_3-5-------------------*/
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				break;
			case 22:
				/*-------------SOL_3-6-------------------*/
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
				break;
			case 23:
				/*-------------SOL_3-7-------------------*/
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
				break;
			case 24:
				/*-------------SOL_3-8-------------------*/
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
				DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
				break;
	}
}
/* ------------------ ALL PACKER SOL OFF--------------*/
void SOL_RUN_OFF()
{
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	}

/* ------------------------------ RUN Packer setting and Sol select------------------------*/
void SOL_RUN(void)
{
	for(int i = 0; i < MAX_PACKER; i++)
	{
		for(int j = 0; j < MAX_PACKER_SOL; j++)
		{
			if(RearLine[PACKER_INFO[i].SOL_NUMBER[j]] == PACKER_INFO[i].SOL_NUMBER[j] && PACKER_INFO[i].SOL_NUMBER[j] !=0)
			{
					SOL_RUN_SELECTE(PACKER_INFO[i].SOL_CONNECT[j]);
					RearLine[PACKER_INFO[i].SOL_NUMBER[j]] = NORUN;
			}
		}
	}
	if(RearLine_Prt[MARKING_INFO.MARKING_1_BURKET_NUM] == MARKING_INFO.MARKING_1_BURKET_NUM)
	{
		SOL_RUN_SELECTE(MARKING_INFO.MARKING_1_CONNECT);
		RearLine_Prt[MARKING_INFO.MARKING_1_BURKET_NUM] = NORUN;
	}

	if(RearLine_Prt[MARKING_INFO.MARKING_2_BURKET_NUM] == MARKING_INFO.MARKING_2_BURKET_NUM)
	{
		SOL_RUN_SELECTE(MARKING_INFO.MARKING_2_CONNECT);
		RearLine_Prt[MARKING_INFO.MARKING_2_BURKET_NUM] = NORUN;
	}
}
//END PACKER SOL
/* ------------------------------ SOL TEST --------------------------------------------  */
void SOL_TEST_RUN()
{
	/*-------------SOL_1-1-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	/*-------------SOL_1-2-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	/*-------------SOL_1-3-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	/*-------------SOL_1-4-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	/*-------------SOL_1-5-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	/*-------------SOL_1-6-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	/*-------------SOL_1-7-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	/*-------------SOL_1-8-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	/*-------------SOL_2-1-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	/*-------------SOL_2-2-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	/*-------------SOL_2-3-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	/*-------------SOL_2-4-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	/*-------------SOL_2-5-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
	/*-------------SOL_2-6-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	/*-------------SOL_2-7-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	/*-------------SOL_2-8-------------------*/
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	/*-------------SOL_3-1-------------------*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	/*-------------SOL_3-2-------------------*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	/*-------------SOL_3-3-------------------*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	/*-------------SOL_3-4-------------------*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	/*-------------SOL_3-5-------------------*/
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	/*-------------SOL_3-6-------------------*/
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	/*-------------SOL_3-7-------------------*/
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	/*-------------SOL_3-8-------------------*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	DWT_Delay_ms(SOL_RUN_TIME_INFO.SOL_OFF_TIME_-SOL_RUN_TIME_INFO.SOL_ON_TIME_);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

/* ------------------------------Rear RUN/Interrup collback ------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t data[200] = {0};
	uint32_t i_cnt =0, j =2;
	avg = 0;
	float resultData =0;
	uint32_t grade = MAX_GRADE+1;
	uint8_t data_count =0;
	uint8_t avg_cnt =0;
	uint8_t SEL_PACKER = 9;

	if (RUN_ACTIVE_NUM == RUN_ACTIVE)
	{
		for (i_cnt = 0; i_cnt <= 255; i_cnt++)
		{
			if (i_cnt == SOL_RUN_TIME_INFO.SOL_ON_TIME_)
			{
					SOL_RUN();
			}

			if (i_cnt == (SOL_RUN_TIME_INFO.SOL_ON_TIME_+SOL_RUN_TIME_INFO.SOL_OFF_TIME_))
			{
					SOL_RUN_OFF();
			}

			if ( i_cnt >=  MEASUREMENT_INFO.START_TIME && i_cnt <= MEASUREMENT_INFO.END_TIME)
			{
				avg += filtereadValue;
				avg_cnt++;
			}

			if(i_cnt == MEASUREMENT_INFO.END_TIME)
			{
				resultData = avg/avg_cnt;
				resultData = (resultData-LOADCELL_INFO.OFFSET)*LOADCELL_INFO.SPAN;
				resultData = ceil(resultData*10)/10;
			}

			if(i_cnt == MEASUREMENT_INFO.END_TIME+1)
			{
					RearLine[0] =0;
					if(resultData <= GRADE_INFO[GRADE_1].HILIMIT && resultData >= GRADE_INFO[GRADE_1].LOLIMIT)
					{
						grade = GRADE_1;
						///////////////////////SET MARKING//////////////////////////////////////
						switch(GRADE_INFO[grade].PRT_USED)
						{
							case PRT_NO:
								RearLine_Prt[0] = 0;
								break;

							case PRT_A:
								if(GRADE_INFO[grade].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
								}
								else if(GRADE_INFO[grade].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
									GRADE_INFO[grade].PRT_COUNT--;
									if(GRADE_INFO[grade].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[grade].PRT_USED = PRT_NO;
									}
								}
								break;

							case PRT_B:
								if(GRADE_INFO[grade].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
								}
								else if(GRADE_INFO[grade].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
									GRADE_INFO[grade].PRT_COUNT--;
									if(GRADE_INFO[grade].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[grade].PRT_USED = PRT_NO;
									}
								}
								break;
							}
							///////////////SELECT PACKER////////////////////////////////
						  if(GRADE_INFO[grade].PACKER_COUNT > 0)
							{
								SEL_PACKER = GRADE_PACKER[grade][G_PACKER_CNT[grade]];
							}
					}
					else if(resultData <= GRADE_INFO[GRADE_2].HILIMIT &&  resultData >= GRADE_INFO[GRADE_2].LOLIMIT)
					{
						grade = GRADE_2;
						///////////////////////SET MARKING//////////////////////////////////////
						switch(GRADE_INFO[GRADE_2].PRT_USED)
						{
							case PRT_NO:
								RearLine_Prt[0] = 0;
								break;

							case PRT_A:
								if(GRADE_INFO[grade].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
								}
								else if(GRADE_INFO[grade].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
									GRADE_INFO[grade].PRT_COUNT--;
									if(GRADE_INFO[grade].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[grade].PRT_USED = PRT_NO;
									}
								}
								break;

							case PRT_B:
								if(GRADE_INFO[grade].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
								}
								else if(GRADE_INFO[grade].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
									GRADE_INFO[grade].PRT_COUNT--;
									if(GRADE_INFO[grade].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[grade].PRT_USED = PRT_NO;
									}
								}
								break;
							}
							///////////////SELECT PACKER////////////////////////////////
						  if(GRADE_INFO[grade].PACKER_COUNT > 0)
							{
								SEL_PACKER = GRADE_PACKER[grade][G_PACKER_CNT[grade]];
							}
					}
					else if(resultData <= GRADE_INFO[GRADE_3].HILIMIT &&  resultData >= GRADE_INFO[GRADE_3].LOLIMIT)
					{
						grade = GRADE_3;
						///////////////////////SET MARKING//////////////////////////////////////
						switch(GRADE_INFO[grade].PRT_USED)
						{
							case PRT_NO:
								RearLine_Prt[0] = 0;
								break;

							case PRT_A:
								if(GRADE_INFO[grade].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
								}
								else if(GRADE_INFO[grade].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
									GRADE_INFO[grade].PRT_COUNT--;
									if(GRADE_INFO[grade].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[grade].PRT_USED = PRT_NO;
									}
								}
								break;

							case PRT_B:
								if(GRADE_INFO[grade].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
								}
								else if(GRADE_INFO[grade].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
									GRADE_INFO[grade].PRT_COUNT--;
									if(GRADE_INFO[grade].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[grade].PRT_USED = PRT_NO;
									}
								}
								break;
							}
							///////////////SELECT PACKER////////////////////////////////
						  if(GRADE_INFO[grade].PACKER_COUNT > 0)
							{
								SEL_PACKER = GRADE_PACKER[grade][G_PACKER_CNT[grade]];
							}
					}
					else if(resultData <= GRADE_INFO[GRADE_4].HILIMIT &&  resultData >= GRADE_INFO[GRADE_4].LOLIMIT)
					{
						grade = GRADE_4;
						///////////////////////SET MARKING//////////////////////////////////////
						switch(GRADE_INFO[grade].PRT_USED)
						{
							case PRT_NO:
								RearLine_Prt[0] = 0;
								break;

							case PRT_A:
								if(GRADE_INFO[grade].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
								}
								else if(GRADE_INFO[grade].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
									GRADE_INFO[grade].PRT_COUNT--;
									if(GRADE_INFO[grade].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[grade].PRT_USED = PRT_NO;
									}
								}
								break;

							case PRT_B:
								if(GRADE_INFO[grade].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
								}
								else if(GRADE_INFO[grade].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
									GRADE_INFO[grade].PRT_COUNT--;
									if(GRADE_INFO[grade].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[grade].PRT_USED = PRT_NO;
									}
								}
								break;
							}
							///////////////SELECT PACKER////////////////////////////////
						  if(GRADE_INFO[grade].PACKER_COUNT > 0)
							{
								SEL_PACKER = GRADE_PACKER[grade][G_PACKER_CNT[grade]];
							}

					}
					else if(resultData <= GRADE_INFO[GRADE_5].HILIMIT && resultData >= GRADE_INFO[GRADE_5].LOLIMIT)
					{
						grade = GRADE_5;
						///////////////////////SET MARKING//////////////////////////////////////
						switch(GRADE_INFO[grade].PRT_USED)
						{
							case PRT_NO:
								RearLine_Prt[0] = 0;
								break;

							case PRT_A:
								if(GRADE_INFO[grade].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
								}
								else if(GRADE_INFO[grade].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
									GRADE_INFO[grade].PRT_COUNT--;
									if(GRADE_INFO[grade].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[grade].PRT_USED = PRT_NO;
									}
								}
								break;

							case PRT_B:
								if(GRADE_INFO[grade].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
								}
								else if(GRADE_INFO[grade].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
									GRADE_INFO[grade].PRT_COUNT--;
									if(GRADE_INFO[grade].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[grade].PRT_USED = PRT_NO;
									}
								}
								break;
							}
							///////////////SELECT PACKER////////////////////////////////
						  if(GRADE_INFO[grade].PACKER_COUNT > 0)
							{
								SEL_PACKER = GRADE_PACKER[grade][G_PACKER_CNT[grade]];
							}
					}
					else if(resultData <= GRADE_INFO[GRADE_6].HILIMIT && resultData >= GRADE_INFO[GRADE_6].LOLIMIT)
					{
						grade = GRADE_6;
						///////////////////////SET MARKING//////////////////////////////////////
						switch(GRADE_INFO[grade].PRT_USED)
						{
							case PRT_NO:
								RearLine_Prt[0] = 0;
								break;

							case PRT_A:
								if(GRADE_INFO[grade].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
								}
								else if(GRADE_INFO[grade].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
									GRADE_INFO[grade].PRT_COUNT--;
									if(GRADE_INFO[grade].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[grade].PRT_USED = PRT_NO;
									}
								}
								break;

							case PRT_B:
								if(GRADE_INFO[grade].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
								}
								else if(GRADE_INFO[grade].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
									GRADE_INFO[grade].PRT_COUNT--;
									if(GRADE_INFO[grade].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[grade].PRT_USED = PRT_NO;
									}
								}
								break;
							}
							///////////////SELECT PACKER////////////////////////////////
						  if(GRADE_INFO[grade].PACKER_COUNT > 0)
							{
								SEL_PACKER = GRADE_PACKER[grade][G_PACKER_CNT[grade]];
							}
					}
					else if(resultData <= GRADE_INFO[GRADE_7].HILIMIT &&  resultData >= GRADE_INFO[GRADE_7].LOLIMIT)
					{
						grade = GRADE_7;
						///////////////////////SET MARKING//////////////////////////////////////
						switch(GRADE_INFO[grade].PRT_USED)
						{
							case PRT_NO:
								RearLine_Prt[0] = 0;
								break;

							case PRT_A:
								if(GRADE_INFO[grade].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
								}
								else if(GRADE_INFO[grade].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
									GRADE_INFO[grade].PRT_COUNT--;
									if(GRADE_INFO[grade].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[grade].PRT_USED = PRT_NO;
									}
								}
								break;

							case PRT_B:
								if(GRADE_INFO[grade].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
								}
								else if(GRADE_INFO[grade].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
									GRADE_INFO[grade].PRT_COUNT--;
									if(GRADE_INFO[grade].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[grade].PRT_USED = PRT_NO;
									}
								}
								break;
							}
							///////////////SELECT PACKER////////////////////////////////

						  if(GRADE_INFO[grade].PACKER_COUNT > 0)
							{
								SEL_PACKER = GRADE_PACKER[grade][G_PACKER_CNT[grade]];
							}

					}
					else if(resultData <= GRADE_INFO[GRADE_8].HILIMIT && resultData >= GRADE_INFO[GRADE_8].LOLIMIT)
					{
						grade = GRADE_8;
						///////////////////////SET MARKING//////////////////////////////////////
						switch(GRADE_INFO[GRADE_8].PRT_USED)
						{
							case PRT_NO:
								RearLine_Prt[0] = 0;
								break;

							case PRT_A:
								if(GRADE_INFO[GRADE_8].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
								}
								else if(GRADE_INFO[GRADE_8].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
									GRADE_INFO[GRADE_8].PRT_COUNT--;
									if(GRADE_INFO[GRADE_8].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[GRADE_8].PRT_USED = PRT_NO;
									}
								}
								break;

							case PRT_B:
								if(GRADE_INFO[GRADE_8].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
								}
								else if(GRADE_INFO[GRADE_8].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
									GRADE_INFO[GRADE_8].PRT_COUNT--;
									if(GRADE_INFO[GRADE_8].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[GRADE_8].PRT_USED = PRT_NO;
									}
								}
								break;
							}
							///////////////SELECT PACKER////////////////////////////////
							if(GRADE_INFO[grade].PACKER_COUNT > 0)
							{
								SEL_PACKER = GRADE_PACKER[grade][G_PACKER_CNT[grade]];
							}
					}
					else if((resultData >= GRADE_INFO[ETC].HILIMIT || resultData <= GRADE_INFO[ETC].LOLIMIT) && resultData >= 10)
					{
						grade = ETC;
						///////////////////////SET MARKING//////////////////////////////////////
						switch(GRADE_INFO[ETC].PRT_USED)
						{
							case PRT_NO:
								RearLine_Prt[0] = 0;
								break;

							case PRT_A:
								if(GRADE_INFO[ETC].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
								}
								else if(GRADE_INFO[ETC].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_1_BURKET_NUM;
									GRADE_INFO[ETC].PRT_COUNT--;
									if(GRADE_INFO[ETC].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[ETC].PRT_USED = PRT_NO;
									}
								}
								break;

							case PRT_B:
								if(GRADE_INFO[ETC].PRT_COUNT == 0)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
								}
								else if(GRADE_INFO[ETC].PRT_COUNT >= 1)
								{
									RearLine_Prt[0] = MARKING_INFO.MARKING_2_BURKET_NUM;
									GRADE_INFO[ETC].PRT_COUNT--;
									if(GRADE_INFO[ETC].PRT_COUNT == 1)
									{
										RearLine_Prt[0] = 0;
										GRADE_INFO[ETC].PRT_USED = PRT_NO;
									}
								}
								break;
							}
					}
			}


		if( i_cnt == MEASUREMENT_INFO.END_TIME+1)
		{
			if(PACKER_INFO[SEL_PACKER].SOL_COUNT > 0 && SEL_PACKER != 9)
			{
				  RearLine[0] = PACKER_INFO[SEL_PACKER].SOL_NUMBER[P_SOL_CNT[SEL_PACKER]-1];
				  P_SOL_CNT[SEL_PACKER]--;
				  if(P_SOL_CNT[SEL_PACKER] == 0)
					{
						P_SOL_CNT[SEL_PACKER] = PACKER_INFO[SEL_PACKER].SOL_COUNT;

						if(GRADE_INFO[grade].PACKER_COUNT >  1)
						{
							G_PACKER_CNT[grade] +=1 ;
						}

						if(G_PACKER_CNT[grade] > GRADE_INFO[grade].PACKER_COUNT-1)
						{
							G_PACKER_CNT[grade] = 0;
						}

					}
				}
			}
			if(i_cnt == MEASUREMENT_INFO.END_TIME+1) // SEND DATA SET
			{
				switch (grade)
				{
					case 0:
						GRADE_DATA.gNumber[GRADE_1]++;
						GRADE_DATA.gWeight[GRADE_1] += resultData;
						GRADE_DATA.gTWeight += resultData;
						if(GRADE_INFO[GRADE_1].PRT_USED != 0)
						{
							GRADE_DATA.pNumber[GRADE_1]++;
						}
						GRADE_DATA.gTNumber++;
						break;

					case 1:
						GRADE_DATA.gNumber[GRADE_2]++;
						GRADE_DATA.gWeight[GRADE_2] += resultData;
						GRADE_DATA.gTWeight += resultData;
						if(GRADE_INFO[GRADE_2].PRT_USED != 0)
						{
							GRADE_DATA.pNumber[GRADE_2]++;
						}
						GRADE_DATA.gTNumber++;
						break;

					case GRADE_3:
						GRADE_DATA.gNumber[GRADE_3]++;
						GRADE_DATA.gWeight[GRADE_3] += resultData;
						GRADE_DATA.gTWeight += resultData;
						if(GRADE_INFO[GRADE_3].PRT_USED != 0)
						{
							GRADE_DATA.pNumber[GRADE_3]++;
						}
						GRADE_DATA.gTNumber++;
						break;

					case GRADE_4:
						GRADE_DATA.gNumber[GRADE_4]++;
						GRADE_DATA.gWeight[GRADE_4] += resultData;
						GRADE_DATA.gTWeight += resultData;
						if(GRADE_INFO[GRADE_4].PRT_USED != 0)
						{
							GRADE_DATA.pNumber[GRADE_4]++;
						}
						GRADE_DATA.gTNumber++;
						break;

					case 4:
						GRADE_DATA.gNumber[GRADE_5]++;
						GRADE_DATA.gWeight[GRADE_5] += resultData;
						GRADE_DATA.gTWeight += resultData;
						if(GRADE_INFO[GRADE_5].PRT_USED != 0)
						{
							GRADE_DATA.pNumber[GRADE_5]++;
						}
						GRADE_DATA.gTNumber++;
						break;

					case GRADE_6:
						GRADE_DATA.gNumber[GRADE_6]++;
						GRADE_DATA.gWeight[GRADE_6] += resultData;
						GRADE_DATA.gTWeight += resultData;
						if(GRADE_INFO[GRADE_6].PRT_USED != 0)
						{
							GRADE_DATA.pNumber[GRADE_6]++;
						}
						GRADE_DATA.gTNumber++;
						break;

					case GRADE_7:
						GRADE_DATA.gNumber[GRADE_7]++;
						GRADE_DATA.gWeight[GRADE_7] += resultData;
						GRADE_DATA.gTWeight += resultData;
						if(GRADE_INFO[GRADE_7].PRT_USED != 0)
						{
							GRADE_DATA.pNumber[GRADE_7]++;
						}
						GRADE_DATA.gTNumber++;
						break;

					case GRADE_8:
						GRADE_DATA.gNumber[GRADE_8]++;
						GRADE_DATA.gWeight[GRADE_8] += resultData;
						GRADE_DATA.gTWeight += resultData;
						if(GRADE_INFO[GRADE_8].PRT_USED != 0)
						{
							GRADE_DATA.pNumber[GRADE_8]++;
						}
						GRADE_DATA.gTNumber++;
						break;

					case ETC:
						GRADE_DATA.gNumber[ETC]++;
						GRADE_DATA.gWeight[ETC] += resultData;
						GRADE_DATA.gTWeight += resultData;
						if(GRADE_INFO[ETC].PRT_USED != 0)
						{
							GRADE_DATA.pNumber[ETC]++;
						}
						GRADE_DATA.gTNumber++;
						break;
				}
				GRADE_DATA.gRCount=RearLine[0];
				GRADE_DATA.gPCount=RearLine_Prt[0];
				timerCnt++;
				GRADE_DATA.gSpeed = Grade_speed;
			}


			if(i_cnt == MEASUREMENT_INFO.END_TIME+1) //Rear moving
			{
				memmove(RearLine+1,RearLine,sizeof(uint8_t)*REAR_COUNT-1);
				memmove(RearLine_Prt+1,RearLine_Prt,sizeof(uint8_t)*REAR_COUNT-1);
			}

			if(i_cnt == MEASUREMENT_INFO.END_TIME+1)
			{
				data[data_count++] = 0x02;
				data[data_count++] = 137; //128

				data[data_count++] = (uint8_t)(*((uint32_t*)&resultData) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&resultData) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&resultData) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&resultData));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gSpeed) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gSpeed) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gSpeed) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gSpeed));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[0]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[0]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[0]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[0]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[0]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[0]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[0]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)& GRADE_DATA.gWeight[0]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[1]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[1]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[1]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[1]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[1]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[1]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[1]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)& GRADE_DATA.gWeight[1]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[2]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[2]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[2]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[2]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[2]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[2]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[2]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)& GRADE_DATA.gWeight[2]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[3]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[3]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[3]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[3]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[3]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[3]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[3]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)& GRADE_DATA.gWeight[3]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[4]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[4]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[4]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[4]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[4]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[4]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[4]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)& GRADE_DATA.gWeight[4]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[5]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[5]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[5]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[5]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[5]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[5]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[5]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)& GRADE_DATA.gWeight[5]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[6]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[6]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[6]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[6]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[6]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[6]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[6]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)& GRADE_DATA.gWeight[6]));


				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[7]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[7]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[7]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[7]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[7]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[7]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[7]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)& GRADE_DATA.gWeight[7]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[8]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[8]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[8]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gNumber[8]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[8]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[8]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gWeight[8]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)& GRADE_DATA.gWeight[8]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gTNumber) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gTNumber) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gTNumber) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gTNumber));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gTWeight) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gTWeight) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gTWeight) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gTWeight));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gRCount));
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.gPCount));

				data[data_count++] = G_PACKER_CNT[SEL_PACKER];
				data[data_count++] = P_SOL_CNT[SEL_PACKER];

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[0]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[0]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[0]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[0]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[1]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[1]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[1]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[1]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[2]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[2]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[2]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[2]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[3]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[3]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[3]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[3]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[4]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[4]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[4]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[4]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[5]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[5]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[5]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[5]));

			    data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[6]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[6]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[6]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[6]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[7]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[7]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[7]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[7]));

				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[8]) >> 24);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[8]) >> 16);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[8]) >> 8);
				data[data_count++] = (uint8_t)(*((uint32_t*)&GRADE_DATA.pNumber[8]));

				data[data_count++] = GRADE_INFO[0].PRT_USED;
				data[data_count++] = GRADE_INFO[1].PRT_USED;
				data[data_count++] = GRADE_INFO[2].PRT_USED;
				data[data_count++] = GRADE_INFO[3].PRT_USED;
				data[data_count++] = GRADE_INFO[4].PRT_USED;
				data[data_count++] = GRADE_INFO[5].PRT_USED;
				data[data_count++] = GRADE_INFO[6].PRT_USED;
				data[data_count++] = GRADE_INFO[7].PRT_USED;
				data[data_count++] = GRADE_INFO[8].PRT_USED;


				data[data_count++] = 0x03;
				HAL_UART_Transmit(&huart1,data,data_count,20);
			}
			DWT_Delay_ms(1);
		}
	}

	else if (RUN_ACTIVE_NUM == MEASURE_RUN_)
	{
				int i =0;
				while(i < 250)
				{
					resultData = (filtereadValue-LOADCELL_INFO.OFFSET)*LOADCELL_INFO.SPAN;
					resultData = ceil(resultData*10)/10;
					if ( resultData > 20.0 && sendvalue == false && resultData < 100.0 && i <= 50 )
					{
						sendvalue = true;
					}
					data_strem[j++] = (uint8_t)(*((uint32_t*)&resultData ) >> 24);
					data_strem[j++] = (uint8_t)(*((uint32_t*)&resultData ) >> 16);
					data_strem[j++] = (uint8_t)(*((uint32_t*)&resultData ) >> 8);
					data_strem[j++] = (uint8_t)(*((uint32_t*)&resultData ));
					DWT_Delay_ms(1);
					i++;
				}
				if(sendvalue == true && send_count == true)
				{
					data_strem[0]= 0x10;
					data_strem[1]= 0x02;
					data_strem[j++] = 0x10;
					data_strem[j++] = 0x03;
					HAL_UART_Transmit(&huart1,data_strem,sizeof(data_strem),100);
					send_count = false;
					sendvalue = false;
				}
	}

}
/* -----------------------------LOAD CELL SET------------------------*/
unsigned int LOADCELL_ZERO(void)
{
	int i = 0 ;
	unsigned int avg_zero = 0;
	for(i = 0; i <= 100; i++)
  {
		avg_zero += filtereadValue;
		DWT_Delay_ms(1);
	}
	avg_zero = avg_zero/i;
	avg_zero = ceil(avg_zero*10)/10;
	return avg_zero;
}

float LOADCELL_SPAN(unsigned int ZERO)
{
	int i;
	float avg_weight = 0;

	for(i = 0; i <= 100; i++)
	{
		avg_weight += filtereadValue;
		DWT_Delay_ms(1);
	}
	avg_weight = avg_weight/i;
	avg_weight = 50/(avg_weight-ZERO);
	avg_weight = ceil(avg_weight*10000)/10000;
	return avg_weight;
}

float LOADCELL_WEIGHT(unsigned int ZERO, float SPAN)
{
	int i = 0;
	float avg_weight = 0;

	for(i = 0; i <= 100; i++)
	{
		avg_weight += filtereadValue;
		DWT_Delay_ms(1);
	}
    avg_weight = avg_weight/i;
	avg_weight = (avg_weight-ZERO)*SPAN;
	avg_weight = ceil(avg_weight*10)/10;

	return avg_weight;
}

/* -----------------------------Serial receive interrupt ------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		 switch(COM_MOD)
		{
		 case COM_STX :
			 if(rxtemp==0x02)
			 {
					COM_MOD = COM_LEN;
				  //HAL_UART_Transmit(&huart1,&ack_1,1,10);
       }
			 break;
		case COM_LEN :
       RxLength = rxtemp;        COM_MOD = COM_DAT;
		   //HAL_UART_Transmit(&huart1,&ack_2,1,10);
       break;
		case COM_DAT :
			 rx_data[revcnt++] = rxtemp;
			 if(RxLength<=revcnt)
			 {
					COM_MOD = COM_ETX;
				  //HAL_UART_Transmit(&huart1,&ack_3,1,10);
			 }
			break;
		case COM_ETX :
      if(rxtemp==0x03)
			{
				  COM_MOD = COM_STX;
				  revcnt=0;
				  //HAL_UART_Transmit(&huart1,&ack_4,1,10);
				  memcpy(run_data,rx_data,sizeof(rx_data));
				  //HAL_UART_Transmit(&huart1,&ack_5,1,10);
				  //HAL_UART_Transmit(&huart1,run_data,RxLength,10);
			      //COM_REV_ENABLE = 1;
      }
      break;
		}
	}
}
/*
int movingAvg(uint32_t *ptrArrNumbers, long *ptrSum, uint32_t pos, uint32_t len, uint32_t nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //return the average
  return *ptrSum / len;
}


void MOVING_AVG()
{
	filtereadValue = movingAvg(arrNumbers, &sum, pos, len, HAL_ADC_GetValue(&hadc1));
	pos++;
	if (pos >= len)
	{
		pos = 0;
	}
	msTimer++;
	if(timerCnt == 1)
	{
		Grade_speed = msTimer;
		msTimer = 0;
		timerCnt = 0;
	}
}
*/
