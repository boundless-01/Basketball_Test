/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    my_fdcan.c
  * @brief   This file provides code for the configuration
  *          of the my_fdcan instances.
  ******************************************************************************/
	
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "my_main.h"
#include "my_fdcan.h"
#include "fdcan.h"
#include "my_action_control.h"
#include "stm32h7xx_hal_fdcan.h"

/* USER CODE BEGIN */
uint8_t can1ReceiveFlag = 0;

FDCAN_RxHeaderTypeDef RxMessage0 = { 0 };
FDCAN_RxHeaderTypeDef RxMessage1 = { 0 };

void FDCAN_ENABLE()//初始化并启用两个 FDCAN 控制器（FDCAN1 和 FDCAN3）
{
	FDCAN_FilterTypeDef FDCAN1_RXFilter;
	FDCAN_FilterTypeDef FDCAN3_RXFilter;

	FDCAN1_RXFilter.IdType = FDCAN_STANDARD_ID;//使用标准 CAN ID（11 位）
	FDCAN1_RXFilter.FilterType = FDCAN_FILTER_MASK;//使用掩码模式过滤
	FDCAN1_RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP;//匹配的报文存入 RXFIFO0 并标记为高优先级
	FDCAN1_RXFilter.FilterID1 = 0x00000000;//接收所有标准 ID 的报文（掩码模式下全 0 表示无过滤）
	FDCAN1_RXFilter.FilterID2 = 0x00000000;
	FDCAN1_RXFilter.FilterIndex = 0;//使用过滤器索引 0
	HAL_FDCAN_ConfigFilter(&hfdcan1,&FDCAN1_RXFilter);//将配置应用到 FDCAN1

	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	//拒绝所有不匹配过滤器的标准帧和扩展帧
	//拒绝所有远程帧（无论是否匹配过滤器）
	HAL_FDCAN_Start(&hfdcan1);//启动 FDCAN1 控制器
	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);//拒绝所有远程帧（无论是否匹配过滤器）

	FDCAN3_RXFilter.IdType = FDCAN_STANDARD_ID;
	FDCAN3_RXFilter.FilterType = FDCAN_FILTER_MASK;
	FDCAN3_RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP;////匹配的报文存入 RXFIFO1 并标记为高优先级
	FDCAN3_RXFilter.FilterID1 = 0x00000000;
	FDCAN3_RXFilter.FilterID2 = 0x00000000;
	FDCAN3_RXFilter.FilterIndex = 0;
	HAL_FDCAN_ConfigFilter(&hfdcan3,&FDCAN3_RXFilter);

	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan3,FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);//监听 RXFIFO1 的新报文
}

//fdcan1
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
//FDCAN 接收 FIFO0 的中断回调函数，当 RXFIFO0 收到新报文时由 HAL 库自动调用
{
	gRobot.rx_can_cnt++;
	uint8_t canNodeId = 0;
	uint8_t data8[9] = {0};

	if(hfdcan == &hfdcan1)
	{
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxMessage0, data8);
		canNodeId = (RxMessage0.Identifier - 0x280);
		switch(canNodeId)
		{
			case UPPER_ARM_ID:
			{
				UpackCalfMsg(&upperArmMsg, data8);
				GetDriverMsg(&upperArmMsg, data8);
				
				HB.upper_arm = 0;
				firstComFlag.upper_arm = 1;
				break;
			}
			
			case FOREARM_ID:
			{
				UpackCalfMsg(&forearmMsg, data8);
				GetDriverMsg(&forearmMsg ,data8);

				HB.forearm = 0;
				firstComFlag.forearm = 1;
				break;
			}
			
			case WRIST_ID:
			{
				UpackCalfMsg(&wristMsg, data8);
				GetDriverMsg(&wristMsg ,data8);
				
				HB.wrist = 0;
				firstComFlag.wrist = 1;
				break;
			}
		}
	}
	memset(&RxMessage0,0,sizeof(RxMessage0));
}

//fdcan3
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	gRobot.rx_can_cnt++;
	uint8_t canNodeId = 0;
	uint8_t data8[9] = {0};
	if(hfdcan == &hfdcan3)
	{
		HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO1, &RxMessage1, data8);
		static uint32_t FifoFillLevel1 = 0;
		static uint32_t FifoFillLevel3 = 0;
		FifoFillLevel1 = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1,FDCAN_RX_FIFO0);
		FifoFillLevel3 = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3,FDCAN_RX_FIFO1);
//		DEBUG("F: %d %d\r\n",(int)FifoFillLevel1,(int)FifoFillLevel3);
		canNodeId = (RxMessage1.Identifier - 0x280);
		switch(canNodeId)
		{
			case WHEEL1_ID:
			{
				//接收can消息，数据转换，存储
				GetSteeringWheelMsg(&wheel1Msg,data8);
				HB.wheel1 = 0;
				firstComFlag.wheel1 = 1;
				break;
			}
			case WHEEL2_ID:
			{
				//接收can消息，数据转换，存储
				GetSteeringWheelMsg(&wheel2Msg,data8);
				HB.wheel2 = 0;
				firstComFlag.wheel2 = 1;
				break;
			}
			case WHEEL3_ID:
			{
				//接收can消息，数据转换，存储
				GetSteeringWheelMsg(&wheel3Msg,data8);
				HB.wheel3 = 0;
				firstComFlag.wheel3 = 1;
				break;
			}
		}
	}
	memset(&RxMessage1,0,sizeof(RxMessage1));


}
/* USER CODE END */

