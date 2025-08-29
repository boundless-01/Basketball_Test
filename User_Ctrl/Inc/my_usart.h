#ifndef _MY_USART_H
#define _MY_USART_H

#include "usart.h"
#include "stdio.h"
#include <string.h>

#define APP_USART huart1
#define APP_RX_MAX_LEN 64
#define APP_DATA_MIN_LEN 8 //app消息最小字节数
extern uint8_t app_rxbuffer[APP_RX_MAX_LEN];

#define ROBOT_COM_USART huart3
#define ROBOT_COM_RX_MAX_LEN 128

#define ROBOT_COM_UINT8_NUM (0)
#define ROBOT_COM_DATA_NUM (3)
#define ROBOT_COM_DATA_LEN (ROBOT_COM_DATA_NUM*4+ROBOT_COM_UINT8_NUM+4)
typedef union 
{
	int		 dataint[ROBOT_COM_DATA_NUM];
	uint32_t dataint32[ROBOT_COM_DATA_NUM];
	float    dataf[ROBOT_COM_DATA_NUM];
	char     datac[ROBOT_COM_DATA_NUM*4];
}robot_com_data_t;

extern robot_com_data_t robotTxData;
extern robot_com_data_t robotRxData;
extern uint8_t robot_rxbuffer[ROBOT_COM_RX_MAX_LEN];
extern uint8_t robot_txbuffer[ROBOT_COM_DATA_LEN];

#define POWER_UART huart9
#define POWER_RX_LEN 8
extern uint8_t power_rxbuffer[POWER_RX_LEN];

typedef union{
	float dataf;
	uint8_t datac[4];
}trans;

void APP_GET_IDLECallback(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void ROBOT_TxIRQHandler(UART_HandleTypeDef *huart);
void GetPowerState(void);
void ClearAllAPPFlag(void);

void PartnerMsgReicve(void);
void ROBOT_RxIRQHandler(UART_HandleTypeDef *huart);
void Send2Parnter(void);
void USART_ENABLE(void);

extern uint8_t app_data_len;
extern uint8_t robot_com_data_len;
extern HAL_StatusTypeDef status;
#endif 