#ifndef _MY_COMMUNICATION_H
#define _MY_COMMUNICATION_H


#include "main.h"
#include "fdcan.h"
#include "usbd_cdc_if.h"
#include "my_action_control.h"
#include "my_main.h"
#include "string.h"
#include "my_sensor.h"

#define MAX_GET_LEN (4000)

//字节数
#define USB_RX_UINT8_NUM (6)
#define USB_RX_DATA_NUM (36)
#define USB_TX_UINT8_NUM (17)
#define USB_TX_DATA_NUM (42)

#define USB_RX_LEN (USB_RX_DATA_NUM*4+USB_RX_UINT8_NUM+4)//2帧头2帧尾
#define USB_TX_LEN (USB_TX_DATA_NUM*4+USB_TX_UINT8_NUM+4)//2帧头2帧尾

typedef union 
{
	int		 dataint[USB_RX_DATA_NUM];
	uint32_t dataint32[USB_RX_DATA_NUM];
	float    dataf[USB_RX_DATA_NUM];
	char     datac[USB_RX_DATA_NUM*4];
}trans_r;

typedef union
{
	int		 dataint[USB_TX_DATA_NUM];
	uint32_t dataint32[USB_TX_DATA_NUM];
	float    data_float[USB_TX_DATA_NUM];
	char     datac[USB_TX_DATA_NUM*4];
}trans_t;

extern trans_t TX_Data;
extern trans_r RX_Data;

extern uint8_t rxbuf[MAX_GET_LEN];
extern uint8_t txbuf[USB_TX_LEN];

extern uint8_t receiveData[USB_RX_LEN];

extern float target_one_motor;
extern float target_two_motor;
extern float target_thr_motor;

extern uint8_t lastState;
extern int readLen;

void ReadFromPc(void);
void SendToPc(void);
void USB_Receive(void);

#endif
