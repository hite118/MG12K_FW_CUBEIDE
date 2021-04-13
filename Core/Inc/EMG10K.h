/*
 * loadcell.h
 *
 *  Created on: Apr 1, 2021
 *      Author: hite118
 */

#ifndef INC_EMG10K_H_
#define INC_EMG10K_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* ---------------------------------------------------------------------------*/
/* ------------------------------SYSTEM---------------------------------------*/
/* ---------------------------------------------------------------------------*/
#define REAR_COUNT 80
#define RUN  1
#define NORUN 0

/* ---------------------------------------------------------------------------*/
/* ------------------------------MODE-----------------------------------------*/
/* ---------------------------------------------------------------------------*/
#define ACTIVE 0x03
#define READ 0x04
#define WRITE 0x05
#define MEASUREMENT 0x06
/* ---------------------------------------------------------------------------*/
/* ------------------------------ACTION---------------------------------------*/
/* ---------------------------------------------------------------------------*/
#define RUN_ACTIVE 0x01
#define NORUN_ACTIVE 0x02
#define GRADE_INFO_ 0x03
#define PACKER_INFO_ 0x04
#define MARKING_INFO_ 0x05
#define SOL_RUN_TIME_INFO_ 0x06
#define MEASUREMENT_INFO_ 0x07
#define LOADCELL_INFO_ZERO_ 0x08
#define LOADCELL_INFO_WEIGHT_ 0x09
#define INIT_DATA_SET_ 0x11
#define ALL_INFO_ 0x12
#define LOADCELL_INFO_ 0x13
#define SOL_TEST_ 0x15
#define CURENT_WEIGHT_ 0x16
#define MEASURE_RUN_ 0x17
#define COM_CHK_ 0x18

/* ---------------------------------------------------------------------------*/
/* -----------------------User flash memory address --------------------------*/
/* ---------------------------------------------------------------------------*/
#define ADDR_FLASH_PAGE_0     ((uint32_t)0x08000000) // Page 0, 1 Kbyte
#define ADDR_FLASH_PAGE_1     ((uint32_t)0x08000400) // Page 1, 1 Kbyte
#define ADDR_FLASH_PAGE_2     ((uint32_t)0x08000800) // Page 2, 1 Kbyte
#define ADDR_FLASH_PAGE_3     ((uint32_t)0x08000C00) // Page 3, 1 Kbyte
#define ADDR_FLASH_PAGE_4     ((uint32_t)0x08001000) // Page 4, 1 Kbyte
#define ADDR_FLASH_PAGE_5     ((uint32_t)0x08001400) // Page 5, 1 Kbyte
#define ADDR_FLASH_PAGE_6     ((uint32_t)0x08001800) // Page 6, 1 Kbyte
#define ADDR_FLASH_PAGE_7     ((uint32_t)0x08001C00) // Page 7, 1 Kbyte
#define ADDR_FLASH_PAGE_8     ((uint32_t)0x08002000) // Page 8, 1 Kbyte
#define ADDR_FLASH_PAGE_9     ((uint32_t)0x08002400) // Page 9, 1 Kbyte
#define ADDR_FLASH_PAGE_10    ((uint32_t)0x08002800) // Page 10, 1 Kbyte
#define ADDR_FLASH_PAGE_11    ((uint32_t)0x08002C00) // Page 11, 1 Kbyte
#define ADDR_FLASH_PAGE_12    ((uint32_t)0x08003000) // Page 12, 1 Kbyte
#define ADDR_FLASH_PAGE_13    ((uint32_t)0x08003400) // Page 13, 1 Kbyte
#define ADDR_FLASH_PAGE_14    ((uint32_t)0x08003800) // Page 14, 1 Kbyte
#define ADDR_FLASH_PAGE_15    ((uint32_t)0x08003C00) // Page 15, 1 Kbyte
#define ADDR_FLASH_PAGE_16    ((uint32_t)0x08004000) // Page 16, 1 Kbyte
#define ADDR_FLASH_PAGE_17    ((uint32_t)0x08004400) // Page 17, 1 Kbyte
#define ADDR_FLASH_PAGE_18    ((uint32_t)0x08004800) // Page 18, 1 Kbyte
#define ADDR_FLASH_PAGE_19    ((uint32_t)0x08004C00) // Page 19, 1 Kbyte
#define ADDR_FLASH_PAGE_20    ((uint32_t)0x08005000) // Page 20, 1 Kbyte
#define ADDR_FLASH_PAGE_21    ((uint32_t)0x08005400) // Page 21, 1 Kbyte
#define ADDR_FLASH_PAGE_22    ((uint32_t)0x08005800) // Page 22, 1 Kbyte
#define ADDR_FLASH_PAGE_23    ((uint32_t)0x08005C00) // Page 23, 1 Kbyte
#define ADDR_FLASH_PAGE_24    ((uint32_t)0x08006000) // Page 24, 1 Kbyte
#define ADDR_FLASH_PAGE_25    ((uint32_t)0x08006400) // Page 25, 1 Kbyte
#define ADDR_FLASH_PAGE_26    ((uint32_t)0x08006800) // Page 26, 1 Kbyte
#define ADDR_FLASH_PAGE_27    ((uint32_t)0x08006C00) // Page 27, 1 Kbyte
#define ADDR_FLASH_PAGE_28    ((uint32_t)0x08007000) // Page 28, 1 Kbyte
#define ADDR_FLASH_PAGE_29    ((uint32_t)0x08007400) // Page 29, 1 Kbyte
#define ADDR_FLASH_PAGE_30    ((uint32_t)0x08007800) // Page 30, 1 Kbyte
#define ADDR_FLASH_PAGE_31    ((uint32_t)0x08007C00) // Page 31, 1 Kbyte
#define ADDR_FLASH_PAGE_32    ((uint32_t)0x08008000) // Page 32, 1 Kbyte
#define ADDR_FLASH_PAGE_33    ((uint32_t)0x08008400) // Page 33, 1 Kbyte
#define ADDR_FLASH_PAGE_34    ((uint32_t)0x08008800) // Page 34, 1 Kbyte
#define ADDR_FLASH_PAGE_35    ((uint32_t)0x08008C00) // Page 35, 1 Kbyte
#define ADDR_FLASH_PAGE_36    ((uint32_t)0x08009000) // Page 36, 1 Kbyte
#define ADDR_FLASH_PAGE_37    ((uint32_t)0x08009400) // Page 37, 1 Kbyte
#define ADDR_FLASH_PAGE_38    ((uint32_t)0x08009800) // Page 38, 1 Kbyte
#define ADDR_FLASH_PAGE_39    ((uint32_t)0x08009C00) // Page 39, 1 Kbyte
#define ADDR_FLASH_PAGE_40    ((uint32_t)0x08010000) // Page 40, 1 Kbyte
#define ADDR_FLASH_PAGE_41    ((uint32_t)0x08010400) // Page 41, 1 Kbyte
#define ADDR_FLASH_PAGE_42    ((uint32_t)0x08010800) // Page 42, 1 Kbyte
#define ADDR_FLASH_PAGE_43    ((uint32_t)0x08010C00) // Page 43, 1 Kbyte
#define ADDR_FLASH_PAGE_44    ((uint32_t)0x08011000) // Page 44, 1 Kbyte
#define ADDR_FLASH_PAGE_45    ((uint32_t)0x08011400) // Page 45, 1 Kbyte
#define ADDR_FLASH_PAGE_46    ((uint32_t)0x08011800) // Page 46, 1 Kbyte
#define ADDR_FLASH_PAGE_47    ((uint32_t)0x08011C00) // Page 47, 1 Kbyte
#define ADDR_FLASH_PAGE_48    ((uint32_t)0x08012000) // Page 48, 1 Kbyte
#define ADDR_FLASH_PAGE_49    ((uint32_t)0x08012400) // Page 49, 1 Kbyte
#define ADDR_FLASH_PAGE_50    ((uint32_t)0x08012800) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_50    ((uint32_t)0x08012800) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_51    ((uint32_t)0x08012C00) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_52    ((uint32_t)0x08013000) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_53    ((uint32_t)0x08013400) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_54    ((uint32_t)0x08013800) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_55    ((uint32_t)0x08013C00) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_56    ((uint32_t)0x08014000) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_57    ((uint32_t)0x08014400) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_58    ((uint32_t)0x08014800) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_59    ((uint32_t)0x08014C00) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_60    ((uint32_t)0x08015000) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_61    ((uint32_t)0x08015400) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_62    ((uint32_t)0x08015800) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_63    ((uint32_t)0x08015C00) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_64    ((uint32_t)0x08016000) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_65    ((uint32_t)0x08016400) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_66    ((uint32_t)0x08016800) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_67    ((uint32_t)0x08016C00) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_68    ((uint32_t)0x08017000) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_69    ((uint32_t)0x08017400) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_70    ((uint32_t)0x08017800) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_71    ((uint32_t)0x08017C00) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_72    ((uint32_t)0x08018000) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_73    ((uint32_t)0x08018400) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_74    ((uint32_t)0x08018800) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_75    ((uint32_t)0x08018C00) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_76    ((uint32_t)0x08019000) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_77    ((uint32_t)0x08019400) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_78    ((uint32_t)0x08019800) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_79    ((uint32_t)0x08019C00) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_80    ((uint32_t)0x08020000) // Page 50, 1 Kbyte
#define ADDR_FLASH_PAGE_81    ((uint32_t)0x08020400) // Page 50, 1 Kbyte
/* ---------------------------------------------------------------------------*/
/*----------------------------User flash memory config------------------------*/
/* ---------------------------------------------------------------------------*/
#define FLASH_PAGE_SIZE   0x400U // 1024 bytes
#define FLASH_USER_START_ADDR    ADDR_FLASH_PAGE_70 // 0x08007C00
#define FLASH_USER_END_ADDR      ADDR_FLASH_PAGE_70 + FLASH_PAGE_SIZE

#define GRADE_INFO_START_ADDR    ADDR_FLASH_PAGE_71
#define GRADE_INFO_END_ADDR      ADDR_FLASH_PAGE_71 + FLASH_PAGE_SIZE

#define PACKER_INFO_START_ADDR   ADDR_FLASH_PAGE_72
#define PACKER_INFO_END_ADDR     ADDR_FLASH_PAGE_72 + FLASH_PAGE_SIZE

#define MARKING_INFO_START_ADDR  ADDR_FLASH_PAGE_73
#define MARKING_INFO_END_ADDR   ADDR_FLASH_PAGE_73 + FLASH_PAGE_SIZE

#define SOL_RUN_TIME_INFO_START_ADDR   ADDR_FLASH_PAGE_74
#define SOL_RUN_TIME_INFO_END_ADDR     ADDR_FLASH_PAGE_74 + FLASH_PAGE_SIZE

#define MEASUREMENT_INFO_START_ADDR ADDR_FLASH_PAGE_75
#define MEASUREMENT_INFO_END_ADDR   ADDR_FLASH_PAGE_75 + FLASH_PAGE_SIZE

#define LOADCELL_INFO_START_ADDR ADDR_FLASH_PAGE_76
#define LOADCELL_INFO_END_ADDR   ADDR_FLASH_PAGE_76 + FLASH_PAGE_SIZE

#define GRADE_INFO_START ((GRADE_INFO_TYPE*)GRADE_INFO_START_ADDR)
#define PACKER_INFO_START ((PACKER_INFO_TYPE*)PACKER_INFO_START_ADDR)
#define MARKING_INFO_START ((MARKING_INFO_TYPE*)MARKING_INFO_START_ADDR)
#define SOL_RUN_TIME_INFO_START ((SOL_RUN_TIME_INFO_TYPE*)SOL_RUN_TIME_INFO_START_ADDR)
#define MEASUREMENT_INFO_START ((MEASUREMENT_INFO_TYPE*)MEASUREMENT_INFO_START_ADDR)
#define LOADCELL_INFO_START ((LOADCELL_INFO_TYPE*)LOADCELL_INFO_START_ADDR)

#define GRADE_INFO_DATA 1
#define PACKER_INFO_DATA 2
#define MARKING_INFO_DATA 3
#define SOL_RUN_TIME_INFO_DATA 4
#define MEASUREMENT_INFO_DATA 5
#define LOADCELL_INFO_DATA 6
/* ---------------------------------------------------------------------------*/
/*-----------------------------SYSTEM DEFINE----------------------------------*/
/* ---------------------------------------------------------------------------*/
#define     MAX_GRADE               9
#define     MAX_PACKER              8
#define     MAX_PACKER_SOL          6
#define     MAX_PRT                 2

#define GRADE_1 0
#define GRADE_2 1
#define GRADE_3 2
#define GRADE_4 3
#define GRADE_5 4
#define GRADE_6 5
#define GRADE_7 6
#define GRADE_8 7
#define ETC 8

#define PRT_NO 0
#define PRT_A 1
#define PRT_B 2

#define PACKER_1 0
#define PACKER_2 1
#define PACKER_3 2
#define PACKER_4 3
#define PACKER_5 4
#define PACKER_6 5
#define PACKER_7 6
#define PACKER_8 7

#define PACKER_SOL_1 0
#define PACKER_SOL_2 1
#define PACKER_SOL_3 2
#define PACKER_SOL_4 3
#define PACKER_SOL_5 4
#define PACKER_SOL_6 5
/* ---------------------------------------------------------------------------*/
/*--------------------------Communication protocol----------------------------*/
/* ---------------------------------------------------------------------------*/
#define STX 0x02
#define ETX 0x03
#define STX_NUM 0
#define LEN_NUM 1
#define COM_STX 0
#define COM_LEN 1
#define COM_DAT 2
#define COM_ETX 3
/* ---------------------------------------------------------------------------*/
/*-----------------------------User struct setting----------------------------*/
/* ---------------------------------------------------------------------------*/
#pragma pack(push, 1)

typedef struct struct_G_DATA{
    uint8_t gRCount;
	uint8_t gPCount;
    uint32_t gNumber[MAX_GRADE];
    uint32_t pNumber[MAX_GRADE];
    uint32_t gTNumber;
	uint32_t gHNumber;
    uint32_t gSpeed;
    float gWeight[MAX_GRADE];
    float gTWeight;
}G_DATA_TYPE;

typedef struct struct_GRADE_INFO{
    uint8_t PACKER_NUMBER[MAX_PACKER];
    uint8_t PACKER_COUNT;
    uint8_t PRT_USED;
	uint32_t PRT_COUNT;
	float HILIMIT;
    float LOLIMIT;
}GRADE_INFO_TYPE;

typedef struct struct_PACKER_INFO{
    uint8_t SOL_COUNT;
	uint8_t SOL_NUMBER[MAX_PACKER_SOL];
	uint8_t SOL_CONNECT[MAX_PACKER_SOL];
}PACKER_INFO_TYPE;

typedef struct struct_MARKING_INFO{
	uint8_t MARKING_1_BURKET_NUM;
	uint8_t MARKING_1_CONNECT;
	uint8_t MARKING_2_BURKET_NUM;
	uint8_t MARKING_2_CONNECT;
}MARKING_INFO_TYPE;

typedef struct struct_SOL_RUN_TIME_INFO{
    uint8_t SOL_ON_TIME_;
	uint8_t SOL_OFF_TIME_;
}SOL_RUN_TIME_INFO_TYPE;

typedef struct struct_MEASUREMENT_INFO{
    uint8_t START_TIME;
	uint8_t END_TIME;
}MEASUREMENT_INFO_TYPE;

typedef struct struct_LOADCELL_INFO{
//	  uint8_t CHANNEL;
    uint32_t OFFSET;
	uint32_t MOVING_AVRAGE;
    float SPAN;

}LOADCELL_INFO_TYPE;

#pragma pack(pop)

G_DATA_TYPE GRADE_DATA;
GRADE_INFO_TYPE GRADE_INFO[MAX_GRADE];
PACKER_INFO_TYPE PACKER_INFO[MAX_PACKER];
MARKING_INFO_TYPE MARKING_INFO;
SOL_RUN_TIME_INFO_TYPE SOL_RUN_TIME_INFO;
MEASUREMENT_INFO_TYPE MEASUREMENT_INFO;
LOADCELL_INFO_TYPE LOADCELL_INFO;

/* ---------------------------------------------------------------------------*/
/*-------------------------------System functions-----------------------------*/
/* ---------------------------------------------------------------------------*/
void BUZZER_ON_OFF();
void FLASH_UPDATE(uint8_t select);
void SOL_RUN_SELECTE(uint8_t selcte1);
void SOL_RUN_(uint8_t selcte1);
void SOL_RUN_OFF();
void SOL_RUN();
void SOL_TEST_RUN();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

unsigned int LOADCELL_ZERO(void);
float LOADCELL_SPAN(unsigned int ZERO);
float LOADCELL_WEIGHT(unsigned int ZERO, float SPAN);
//int movingAvg(uint32_t *ptrArrNumbers, long *ptrSum, uint32_t pos, uint32_t len, uint32_t nextNum);
//void MOVING_AVG();
/* ---------------------------------------------------------------------------*/
/*-------------------------------END-----------------------------*/
/* ---------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* INC_EMG10K_H_ */
