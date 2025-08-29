/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    my_fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the my_fdcan.c file
  *******************************************************************************/
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MY_FDCAN_H__
#define __MY_FDCAN_H__


/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
extern uint8_t can1ReceiveFlag;
extern uint8_t can3ReceiveFlag;

void FDCAN_ENABLE(void);
/* USER CODE END Prototypes */



#endif /* __MY_FDCAN_H__ */

