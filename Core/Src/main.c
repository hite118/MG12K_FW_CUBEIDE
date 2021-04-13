/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
#include "dwt_stm32_delay.h"
#include "EMG10K.h"
//#include "Flash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
extern uint8_t rxtemp;

extern uint8_t RUN_ACTIVE_NUM;
extern uint8_t init_run_data[64];

extern bool send_count;
extern bool sendvalue;

extern uint8_t send_cnt;
extern uint8_t data_strem[1004];

extern uint8_t rx_data[256],rx_data_[256],run_data[256], rx_data_tmp[256];
extern uint8_t txbuff[100];
extern uint8_t txbuff_SEND[1024];

extern uint32_t SOL_SELECT;

extern uint32_t filtereadValue, sensorValue;
extern uint32_t msTimer;
extern uint32_t timerCnt;
extern uint32_t Grade_speed;
extern uint32_t interuptCnt, startTime , endTime;

extern uint8_t RearLine [REAR_COUNT];
extern uint8_t Rear_Count;

extern uint8_t RearLine_Prt [REAR_COUNT];
extern uint8_t Rear_Count_Prt;

extern uint8_t GRADE_PACKER_SELECT_DOUBLE;
extern uint8_t PACKER_SOL_SELECT_DOUBLE[MAX_PACKER_SOL];

extern uint8_t G_PACKER_CNT[MAX_GRADE];
extern uint8_t P_SOL_CNT[MAX_PACKER];
extern uint8_t GRADE_PACKER[MAX_PACKER][MAX_PACKER];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void BUZZER_ON_OFF();
void FLASH_UPDATE(uint8_t select);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void SOL_RUN_SELECTE(uint8_t selcte1);
void SOL_RUN_(uint8_t selcte1);
void SOL_RUN_OFF();
void SOL_RUN();
void SOL_TEST_RUN();

unsigned int LOADCELL_ZERO(void);
float LOADCELL_SPAN(unsigned int ZERO);
float LOADCELL_WEIGHT(unsigned int ZERO, float SPAN);

/* ------------------Loadcell params--------------*/
extern unsigned int avg;
extern unsigned int avg_real;
extern unsigned int avg_volt;
extern float avg_float;
extern unsigned int time_count;
/* ---------------------END-----------------------*/

/* ------------------------------LoadCell Params--------------------------------  */



/* ---------------------END-----------------------  */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	 uint32_t ReceiveDATA, TX_DATA = 0;//, RX_DATA = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();

  DWT_Delay_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_ADC_Start(&hadc1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);


  for(int jcnt=0;jcnt<40;jcnt++)
  	{
  				if((jcnt%2)==0)
  				{
  					 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); HAL_Delay(50);
  					 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); HAL_Delay(50);
  					 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); HAL_Delay(50);
  				}
  				else
  				{
  					 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); HAL_Delay(50);
  					 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); HAL_Delay(50);
  					 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); HAL_Delay(50);
  				}
  	}
  	BUZZER_ON_OFF();
  	BUZZER_ON_OFF();


	memcpy(&GRADE_INFO ,GRADE_INFO_START  ,sizeof(GRADE_INFO));
	memcpy(&PACKER_INFO ,PACKER_INFO_START  ,sizeof(PACKER_INFO));
	memcpy(&MARKING_INFO  ,MARKING_INFO_START  ,sizeof(MARKING_INFO));
	memcpy(&SOL_RUN_TIME_INFO  ,SOL_RUN_TIME_INFO_START  ,sizeof(SOL_RUN_TIME_INFO));
	memcpy(&MEASUREMENT_INFO ,MEASUREMENT_INFO_START ,sizeof(MEASUREMENT_INFO));
	memcpy(&LOADCELL_INFO  ,LOADCELL_INFO_START   ,sizeof(LOADCELL_INFO));

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

	uint8_t k;
	k = 0;

	for(int i = 0; i < MAX_GRADE-1;i++)
	{
		P_SOL_CNT[i]=PACKER_INFO[i].SOL_COUNT;
		for(int j = 0; j < MAX_PACKER; j++ )
		{
			if(GRADE_INFO[i].PACKER_NUMBER[j]==1)
			{
				GRADE_PACKER[i][k++] = j;
			}
		}
		for(int j = 0; j < MAX_PACKER - GRADE_INFO[i].PACKER_COUNT; j++ )
		{
				GRADE_PACKER[i][k++] = 0;
		}
		k=0;
	}

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == 1)  //AUTO 모드로 수정
	{
		RUN_ACTIVE_NUM = RUN_ACTIVE;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	}
	 //uint8_t size = 0;

	 uint32_t tmp;

	 RUN_ACTIVE_NUM = RUN_ACTIVE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		/*---------------MODE SECTION------------------------*/
	 	switch (run_data[0])
		{
/*---------------READ SECTION START--------------------------*/
			case READ :
				switch (run_data[1])
				{
					case GRADE_INFO_:	 /* GRADE_INFO_ VALUE SEND*/
						RUN_ACTIVE_NUM = GRADE_INFO_;
						txbuff_SEND[STX_NUM] = STX;
						TX_DATA = LEN_NUM;
					  TX_DATA++;
						for(int j = 0; j < MAX_GRADE; j++)
						{
							txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&GRADE_INFO[j].HILIMIT ) >> 24);
							txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&GRADE_INFO[j].HILIMIT ) >> 16);
							txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&GRADE_INFO[j].HILIMIT ) >> 8);
							txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&GRADE_INFO[j].HILIMIT ));

							txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&GRADE_INFO[j].LOLIMIT ) >> 24);
							txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&GRADE_INFO[j].LOLIMIT ) >> 16);
							txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&GRADE_INFO[j].LOLIMIT ) >> 8);
							txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&GRADE_INFO[j].LOLIMIT ));

						    txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&GRADE_INFO[j].PRT_COUNT ) >> 24);
							txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&GRADE_INFO[j].PRT_COUNT ) >> 16);
							txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&GRADE_INFO[j].PRT_COUNT ) >> 8);
							txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&GRADE_INFO[j].PRT_COUNT ));
							txbuff_SEND[TX_DATA++] = GRADE_INFO[j].PRT_USED;

							txbuff_SEND[TX_DATA++] = GRADE_INFO[j].PACKER_COUNT;
							for(int k = 0; k < MAX_PACKER; k++)
							{
								txbuff_SEND[TX_DATA++] = GRADE_INFO[j].PACKER_NUMBER[k];
							}

						}
						for(int k = 0; k < MAX_PACKER; k++)
						{
								txbuff_SEND[TX_DATA++] = PACKER_INFO[k].SOL_COUNT;
						}

						txbuff_SEND[LEN_NUM] = TX_DATA-2;
						txbuff_SEND[TX_DATA++] = ETX;
						HAL_UART_Transmit(&huart1,txbuff_SEND,TX_DATA,50);
						BUZZER_ON_OFF();
						memcpy(run_data,init_run_data,sizeof(init_run_data));
						break;

					case PACKER_INFO_:	   /* PACKER_INFO_ VALUE SEND*/
						RUN_ACTIVE_NUM = PACKER_INFO_;
						txbuff_SEND[STX_NUM] = STX;
						TX_DATA = LEN_NUM;
					  TX_DATA++;
						for(int j = 0; j < MAX_PACKER; j++)
						{
								txbuff_SEND[TX_DATA++] = PACKER_INFO[j].SOL_COUNT;
								for(int k = 0; k < MAX_PACKER_SOL; k++)
								{
									txbuff_SEND[TX_DATA++]=PACKER_INFO[j].SOL_NUMBER[k];
								}
								for(int k = 0; k < MAX_PACKER_SOL; k++)
								{
									txbuff_SEND[TX_DATA++]=PACKER_INFO[j].SOL_CONNECT[k];
								}
						}
						txbuff_SEND[TX_DATA++]=MARKING_INFO.MARKING_1_BURKET_NUM;
						txbuff_SEND[TX_DATA++]=MARKING_INFO.MARKING_1_CONNECT;
						txbuff_SEND[TX_DATA++]=MARKING_INFO.MARKING_2_BURKET_NUM;
						txbuff_SEND[TX_DATA++]=MARKING_INFO.MARKING_2_CONNECT;
						txbuff_SEND[TX_DATA++]=SOL_RUN_TIME_INFO.SOL_ON_TIME_;
						txbuff_SEND[TX_DATA++]=SOL_RUN_TIME_INFO.SOL_OFF_TIME_;

						txbuff_SEND[LEN_NUM] = TX_DATA-2;
						txbuff_SEND[TX_DATA++] = ETX;

						HAL_UART_Transmit(&huart1,txbuff_SEND,TX_DATA,50);
						BUZZER_ON_OFF();
						memcpy(run_data,init_run_data,sizeof(init_run_data));
						break;

					case MEASUREMENT_INFO_:	 /* MEASUREMENT_INFO_ VALUE SEND*/
						RUN_ACTIVE_NUM = MEASUREMENT_INFO_;
						txbuff_SEND[STX_NUM] = STX;
						TX_DATA = LEN_NUM;
						TX_DATA++;
					  txbuff_SEND[TX_DATA++] = MEASUREMENT_INFO.START_TIME;
					  txbuff_SEND[TX_DATA++] = MEASUREMENT_INFO.END_TIME;
						txbuff_SEND[LEN_NUM] = TX_DATA-2;
						txbuff_SEND[TX_DATA++] = ETX;
						HAL_UART_Transmit(&huart1,txbuff_SEND,TX_DATA,50);
						BUZZER_ON_OFF();
						memcpy(run_data,init_run_data,sizeof(init_run_data));
						break;

					case LOADCELL_INFO_:	  /* INIT_CONFIG VALUE SEND*/
						RUN_ACTIVE_NUM = LOADCELL_INFO_;
					  avg_float = LOADCELL_WEIGHT(LOADCELL_INFO.OFFSET,LOADCELL_INFO.SPAN);
					  BUZZER_ON_OFF();
						txbuff_SEND[STX_NUM] = STX;
						TX_DATA = LEN_NUM;
					  TX_DATA++;
					    txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&avg_float ) >> 24);
						txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&avg_float ) >> 16);
						txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&avg_float ) >> 8);
						txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&avg_float ));
					    txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.OFFSET ) >> 24);
						txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.OFFSET ) >> 16);
						txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.OFFSET ) >> 8);
						txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.OFFSET ));
					    txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.SPAN ) >> 24);
						txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.SPAN ) >> 16);
						txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.SPAN ) >> 8);
						txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.SPAN ));
					    txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.MOVING_AVRAGE ) >> 24);
						txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.MOVING_AVRAGE ) >> 16);
						txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.MOVING_AVRAGE ) >> 8);
						txbuff_SEND[TX_DATA++] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.MOVING_AVRAGE ));
						txbuff_SEND[LEN_NUM] = TX_DATA-2;
						txbuff_SEND[TX_DATA++] = ETX;
						HAL_UART_Transmit(&huart1,txbuff_SEND,TX_DATA,50);
						BUZZER_ON_OFF();
						memcpy(run_data,init_run_data,sizeof(init_run_data));
						break;

					case ALL_INFO_:	  /* INIT_CONFIG VALUE SEND*/
						RUN_ACTIVE_NUM = ALL_INFO_;
						BUZZER_ON_OFF();
						BUZZER_ON_OFF();
						memcpy(run_data,init_run_data,sizeof(init_run_data));
						break;
					case COM_CHK_:	  /* INIT_CONFIG VALUE SEND*/
						RUN_ACTIVE_NUM = ALL_INFO_;
						BUZZER_ON_OFF();
					  txbuff_SEND[STX_NUM] = STX;
						TX_DATA = LEN_NUM;
					  TX_DATA++;
          txbuff_SEND[TX_DATA++] = 51;
						txbuff_SEND[LEN_NUM] = TX_DATA-2;
						txbuff_SEND[TX_DATA++] = ETX;
						HAL_UART_Transmit(&huart1,txbuff_SEND,TX_DATA,50);
						BUZZER_ON_OFF();
						memcpy(run_data,init_run_data,sizeof(init_run_data));
						break;
				}
			break;
/*---------------READ SECTION END--------------------------*/

/*---------------SET START--------------------------*/
			case WRITE : // WRITE
				switch (run_data[1])
				{
					case GRADE_INFO_:
							RUN_ACTIVE_NUM = GRADE_INFO_;
					    BUZZER_ON_OFF();
							ReceiveDATA = 2;

							for (int i = 0; i < MAX_GRADE; i++)
							{
								tmp  =   run_data[ReceiveDATA++];
								tmp +=   ((run_data[ReceiveDATA++]&0xFF)<<8);
								tmp +=   ((run_data[ReceiveDATA++]&0xFF)<<16);
								tmp +=   ((run_data[ReceiveDATA++]&0xFF)<<24);
								GRADE_INFO[i].HILIMIT = *((float*)&tmp);
								GRADE_INFO[i].HILIMIT = ceil(GRADE_INFO[i].HILIMIT*10)/10;

								tmp  =   run_data[ReceiveDATA++];
								tmp +=   ((run_data[ReceiveDATA++]&0xFF)<<8);
								tmp +=   ((run_data[ReceiveDATA++]&0xFF)<<16);
								tmp +=   ((run_data[ReceiveDATA++]&0xFF)<<24);
								GRADE_INFO[i].LOLIMIT = *((float*)&tmp);
							    GRADE_INFO[i].LOLIMIT = ceil(GRADE_INFO[i].LOLIMIT*10)/10;
								GRADE_INFO[i].PRT_COUNT = 0;
              GRADE_INFO[i].PRT_COUNT =    (((uint32_t)run_data[ReceiveDATA++] & 0xFF));
								GRADE_INFO[i].PRT_COUNT +=   (((uint32_t)run_data[ReceiveDATA++] & 0xFF) << 8);
								GRADE_INFO[i].PRT_COUNT +=   (((uint32_t)run_data[ReceiveDATA++] & 0xFF) << 16);
								GRADE_INFO[i].PRT_COUNT +=   (((uint32_t)run_data[ReceiveDATA++] & 0xFF) << 24);
								if(GRADE_INFO[i].PRT_COUNT != 0)
								{
									GRADE_INFO[i].PRT_COUNT = GRADE_INFO[i].PRT_COUNT+2;
								}
								GRADE_INFO[i].PRT_USED = run_data[ReceiveDATA++];

								GRADE_INFO[i].PACKER_COUNT = run_data[ReceiveDATA++];
								for(int j = 0 ; j < MAX_PACKER; j++)
								{
									GRADE_INFO[i].PACKER_NUMBER[j] = run_data[ReceiveDATA++];
								}
							}
							for(int j = 0 ; j < MAX_PACKER; j++)
							{
									PACKER_INFO[j].SOL_COUNT = run_data[ReceiveDATA++];
							}
							FLASH_UPDATE(GRADE_INFO_DATA);
							FLASH_UPDATE(PACKER_INFO_DATA);
							k = 0;
							for(int i = 0; i < MAX_PACKER; i++)
							{
								for(int j = 0; j < MAX_PACKER; j++ )
								{
									if(GRADE_INFO[i].PACKER_NUMBER[j]==1)
									{
										GRADE_PACKER[i][k++] = j;
									}
								}
								for(int j = 0; j < MAX_PACKER - GRADE_INFO[i].PACKER_COUNT; j++ )
								{
										GRADE_PACKER[i][k++] = 0;
								}
								k=0;
							}
							memset(G_PACKER_CNT,0,MAX_PACKER);
							memcpy(run_data,init_run_data,sizeof(init_run_data));
						break;

					case PACKER_INFO_:
							RUN_ACTIVE_NUM = PACKER_INFO_;
							BUZZER_ON_OFF();
							ReceiveDATA = 2;
							for (int i = 0; i <MAX_PACKER; i++)
							{
								PACKER_INFO[i].SOL_COUNT = run_data[ReceiveDATA++];

								for(int j = 0 ; j < MAX_PACKER_SOL; j++)
								{
									PACKER_INFO[i].SOL_NUMBER[j] = run_data[ReceiveDATA++];
								}
								for(int j = 0 ; j < MAX_PACKER_SOL; j++)
								{
									PACKER_INFO[i].SOL_CONNECT[j] = run_data[ReceiveDATA++];
								}
							}
							MARKING_INFO.MARKING_1_BURKET_NUM = run_data[ReceiveDATA++];
							MARKING_INFO.MARKING_1_CONNECT = run_data[ReceiveDATA++];
							MARKING_INFO.MARKING_2_BURKET_NUM = run_data[ReceiveDATA++];
							MARKING_INFO.MARKING_2_CONNECT = run_data[ReceiveDATA++];

							SOL_RUN_TIME_INFO.SOL_ON_TIME_ = run_data[ReceiveDATA++];
							SOL_RUN_TIME_INFO.SOL_OFF_TIME_ = run_data[ReceiveDATA++];

							FLASH_UPDATE(PACKER_INFO_DATA);
							FLASH_UPDATE(MARKING_INFO_DATA);
							FLASH_UPDATE(SOL_RUN_TIME_INFO_DATA);
							for(int i = 0; i < MAX_PACKER; i++)
							{
								P_SOL_CNT[i]=PACKER_INFO[i].SOL_COUNT;
							}
							memcpy(run_data,init_run_data,sizeof(init_run_data));
						break;

					case MEASUREMENT_INFO_: /* LOADCELL_CONFIG VALUE MEMORY SAVE [DELAY TIME/END TIME]*/
							RUN_ACTIVE_NUM = MEASUREMENT_INFO_;
							BUZZER_ON_OFF();
							ReceiveDATA = 2;
							MEASUREMENT_INFO.START_TIME = run_data[ReceiveDATA++];
							MEASUREMENT_INFO.END_TIME =  run_data[ReceiveDATA++];
							FLASH_UPDATE(MEASUREMENT_INFO_DATA);
							BUZZER_ON_OFF();
							memcpy(run_data,init_run_data,sizeof(init_run_data));
						break;

					case INIT_DATA_SET_:   /* INIT_DATA*/
						RUN_ACTIVE_NUM = INIT_DATA_SET_;
							for(int i; i < MAX_GRADE; i++)
							{
									GRADE_DATA.gNumber[i] = 0;
									GRADE_DATA.gWeight[i] = 0;
									GRADE_DATA.pNumber[i] = 0;
							}
							GRADE_DATA.gTNumber = 0;
							GRADE_DATA.gTWeight = 0;
							GRADE_DATA.gPCount = 0;
							GRADE_DATA.gRCount = 0;

							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); HAL_Delay(100);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); HAL_Delay(100);
							memcpy(run_data,init_run_data,sizeof(init_run_data));
						break;
				}
			break;
/*---------------WRITE SECTION END--------------------------*/

/*---------------RUN ACTIVE SECTION START--------------------------*/
			case ACTIVE :
				switch (run_data[1])
				{
					case RUN_ACTIVE:	/* RUN_ACTIVE START*/
						RUN_ACTIVE_NUM = RUN_ACTIVE;
						BUZZER_ON_OFF();
					  memcpy(run_data,init_run_data,sizeof(init_run_data));
					break;

					case NORUN_ACTIVE:	/* RUN_ACTIVE STOP */
						RUN_ACTIVE_NUM = NORUN_ACTIVE;
						BUZZER_ON_OFF();
					  memcpy(run_data,init_run_data,sizeof(init_run_data));
					break;
				}
			break;
/*---------------RUN ACTIVE SECTION END--------------------------*/

/*---------------MEASUREMENT SECTION START--------------------------*/
			case MEASUREMENT :
				switch (run_data[1])
				{
					case LOADCELL_INFO_ZERO_: 	/* LOADCELL_ZERO_MEASUREMENT START*/
						RUN_ACTIVE_NUM = LOADCELL_INFO_ZERO_;
						for(int i =0; i<5;i++)
						{
							LOADCELL_INFO.OFFSET=LOADCELL_ZERO();
						}
						BUZZER_ON_OFF();
						txbuff[0] = 0x02;
						txbuff[1] = sizeof(LOADCELL_INFO.OFFSET);
						txbuff[2] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.OFFSET ) >> 24);
						txbuff[3] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.OFFSET ) >> 16);
						txbuff[4] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.OFFSET ) >> 8);
						txbuff[5] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.OFFSET ));
						txbuff[6] = 0x03;
						FLASH_UPDATE(LOADCELL_INFO_DATA);
						HAL_UART_Transmit(&huart1,txbuff,3+sizeof(LOADCELL_INFO.OFFSET),10);
						memcpy(run_data,init_run_data,sizeof(init_run_data));

						break;
					case LOADCELL_INFO_WEIGHT_: 	/* LOADCELL_WEIGHT_MEASUREMENT START */
						RUN_ACTIVE_NUM = LOADCELL_INFO_WEIGHT_;
					  for(int i =0; i<5;i++)
					  {
							LOADCELL_INFO.SPAN = LOADCELL_SPAN(LOADCELL_INFO.OFFSET);
						}
						BUZZER_ON_OFF();
						txbuff[0] = 0x02;
						txbuff[1] =  sizeof(LOADCELL_INFO.SPAN);
						txbuff[2] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.SPAN ) >> 24);
						txbuff[3] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.SPAN ) >> 16);
						txbuff[4] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.SPAN ) >> 8);
						txbuff[5] = (uint8_t)(*((uint32_t*)&LOADCELL_INFO.SPAN ));
						txbuff[6] = 0x03;
						HAL_UART_Transmit(&huart1,txbuff,3+sizeof(LOADCELL_INFO.SPAN),10);
						FLASH_UPDATE(LOADCELL_INFO_DATA);
						memcpy(run_data,init_run_data,sizeof(init_run_data));
						break;

					case MEASURE_RUN_:	/* LOADCELL_WEIGHT_MEASUREMENT START */
						RUN_ACTIVE_NUM = MEASURE_RUN_;
					    send_count = true;
					    sendvalue = false;
					    send_cnt =0;
						BUZZER_ON_OFF();
					    memset (data_strem, 0, sizeof(data_strem));
						memcpy(run_data,init_run_data,sizeof(init_run_data));
						break;

					case SOL_TEST_:	/* SOL_TEST START */
						RUN_ACTIVE_NUM = SOL_TEST_;
					  SOL_SELECT =run_data[2];
					  if (SOL_SELECT < 40)
						{
							SOL_RUN_(SOL_SELECT);
							memcpy(run_data,init_run_data,sizeof(init_run_data));
							SOL_SELECT = 0;
						}
						else if (SOL_SELECT == 41)
						{
							SOL_TEST_RUN();
						}
						else if (SOL_SELECT == 42)
						{
							memcpy(run_data,init_run_data,sizeof(init_run_data));
							SOL_SELECT = 0;
						}
						BUZZER_ON_OFF();
						break;
				}
/*---------------MEASUREMENT SECTION END--------------------------*/
				break;
/*----------------------MODE SECTION END--------------------------*/
			}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  //hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.ExternalTrigConv = ADC1_2_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                           |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                           |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                           |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                           |GPIO_PIN_15|GPIO_PIN_9, GPIO_PIN_RESET);

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

   /*Configure GPIO pins : PC13 PC14 PC15 PC0
                            PC1 PC2 PC3 PC4
                            PC5 PC6 PC7 PC8
                            PC9 PC10 PC11 PC12 */
   GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                           |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                           |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                           |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_PULLUP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   /*Configure GPIO pins : PA3 PA4 PA5 PA6
                            PA7 */
   GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

 	GPIO_InitStruct.Pin =  GPIO_PIN_7;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_PULLUP;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



   /*Configure GPIO pins : PB0 PB1 */
   GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   /*Configure GPIO pins : PB2 PB12 PB13 PB14
                            PB15 PB5 PB6 PB7
                            PB8 PB9 */
   GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                           |GPIO_PIN_15|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                           |GPIO_PIN_8|GPIO_PIN_9;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_PULLUP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   /*Configure GPIO pins : PA8 PA11 PA12 */
   GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_PULLUP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /*Configure GPIO pin : PD2 */
   GPIO_InitStruct.Pin = GPIO_PIN_2;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

   /* EXTI interrupt init*/
   HAL_NVIC_SetPriority(EXTI0_IRQn, 4, 0);
   HAL_NVIC_EnableIRQ(EXTI0_IRQn);

   HAL_NVIC_SetPriority(EXTI1_IRQn, 4, 0);
   HAL_NVIC_EnableIRQ(EXTI1_IRQn);

   HAL_NVIC_SetPriority(EXTI2_IRQn, 4, 0);
   HAL_NVIC_EnableIRQ(EXTI2_IRQn);

   HAL_NVIC_SetPriority(EXTI3_IRQn, 4, 0);
   HAL_NVIC_EnableIRQ(EXTI3_IRQn);

   HAL_NVIC_SetPriority(EXTI4_IRQn, 4, 0);
   HAL_NVIC_EnableIRQ(EXTI4_IRQn);

 //  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
 //  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);


}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
