/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    my_motor_driver.c
  * @brief   This file provides code for the configuration
  *          of the my_motor_driver instances.
  ******************************************************************************/
	
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "my_chassis_driver.h"
#include "my_action_control.h"
#include "my_main.h"
/* USER CODE BEGIN */

/**
* @brief 舵轮控制模式切换
* @param hfdcan: CAN通道编号
* @param DriverNum: 驱动器ID号
* @param DriverMode: 舵轮控制模式
					POS_SPD_CURR_CTRL_MODE: 位置环
					SPD_CURR_CTRL_MODE: 速度环
* @author ACTION
* @note 默认PTP模式
*/
void SetSteeringWheelMode(FDCAN_HandleTypeDef* hfdcan, uint8_t DriverNum, uint8_t DriverMode)
{
	uint16_t identifier = IDENTIFIER_DRIVER_MODE;// 指令标识符（模式设置）
	FDCAN_TxHeaderTypeDef TxMessage = {0};// 初始化 CAN 报文头
	uint8_t data[2];
	TxMessage.Identifier = DriverNum + DRIVER_SERVER_BASE_ID;// 目标节点 CAN ID
	TxMessage.IdType = FDCAN_STANDARD_ID;// 标准 ID（11 位）
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_5;// 数据长度 5 字节（实际只用 2 字节）
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;// 错误状态指示
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;// 禁止比特率切换
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN; // 经典 CAN 帧格式
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0;
	
	data[0] = ((identifier<<4)&0xF0);
	data[1] = ((DriverMode)&0x0F);
	
	// 发送
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
//		Error_Handler();
	}
}


/**
* @brief  舵轮位置速度控制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64   
* @param  vel: 速度，单位：r/s, 范围：-64 ~ 64   单位r/s 
* @param  pos: 位置，单位：圈, 范围：-52428.8 ~ 52428.8  单位°
* @author ACTION
*/
void SteeringWheelCtrl(FDCAN_HandleTypeDef* hfdcan, uint8_t DriverNum, float velAim, float PosAim)
{
	gRobot.tx_can_cnt++;// 发送计数器递增
	uint16_t identifier = IDENTIFIER_POS_SPD_CTRL;
	int16_t vel = 0;
	int32_t pos = 0;
	uint8_t data[5];
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	

	/*限幅*/
	Saturation_float(&velAim,64.f,-64.f);// 速度限幅 ±64
	Saturation_float(&PosAim,52428.8f,-52428.8f);//524288：1000 0000 0000 0000 0000 共20位

	TxMessage.Identifier = DriverNum + DRIVER_SERVER_BASE_ID; // 计算目标节点 CAN ID
	TxMessage.IdType = FDCAN_STANDARD_ID;// 标准 ID（11 位）
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_5;// 数据长度 5 字节
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE; // 错误状态指示
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;// 禁止比特率切换
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;// 经典 CAN 帧格式
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0;

	vel=(int16_t)(velAim*1000.f);//*100是为了提高精度，int16_t：-32768~32767
	pos=PosAim*10.f;
	
	data[0] = (identifier<<4)&0xF0;//identifier<<4: 00010000 0xF0:11110000 data[0] = 00010000 aka 0x20
	if(PosAim >= 0)
	{
		//依次填入4，8，8位，共20位
		data[0] |= pos&0x0F;//0x0F:00001111,填入四位
		data[1] = (pos>>4);
		data[2] = (pos>>12);
	}
	if(PosAim < 0)//负数为补码形式储存，故而做以下操作
	{
		pos = -pos;
		data[0] |= pos&0x0F;
		data[1] = (pos>>4);
		data[2] = (pos>>12)|0x80;//0x80:1000 0000 第8位填1保证为负数
	}
	if(vel >= 0)
	{
		data[3] = vel>>8;
		data[4] = vel>>0;
	}
	else if(vel < 0)
	{
		vel = -vel;
		data[3] = ((vel>>8) & 0x7f) | 0x80;//0x7f：0111 1111
		data[4] = vel & 0xff >> 0;
	}

	// 发送
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
//		Error_Handler();
	}
}

/**
*
* @brief  驱动器控制命令
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，命令：
					PTP_MODE: 点对点模式
					BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  ILIMmode:限流模式，命令：
					TEST_MODE :限流10A，限温35°，用于测试 
					NORMAL_MODE :限流80A，限温45°，正常使用
					EXTREME_MODE :限流150A，限温55°，拒绝平庸，挑战极限！
* @param  wheelstate,turnstate:使能/失能，命令（电机的失能为驱动器停止对电机的驱动，可灵活使用）
					ENABLE_ALL :使能
					DISABLE_ALL :失能
* @author ZBW
* @note	  
*/
void DriverStateChassis(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, ACC_MODE acc_mode,\
				RGB_Cmd rgb_cmd, CurrQLim_Mode ILIMmode, DriverState_Mode wheelstate, DriverState_Mode turnstate)
{
	gRobot.tx_can_cnt++;
	uint16_t identifier = IDENTIFIER_SW_SET;
	uint8_t data[3];
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	

	TxMessage.Identifier = DriverNum+ CMDmode + DRIVER_SERVER_BASE_ID;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_3;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0;
	
	data[0] = (((identifier<<4)&0xF0) | (rgb_cmd&0x07) | ((acc_mode&0x01)<<3));
	data[1] = (((turnstate&0x01)<<6) | ((wheelstate&0x01)<<5) | ((ILIMmode&0x07)<<2) | ((acc_mode&0x06)>>1));
	data[2] = 0;
	
	// 发送
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
//		Error_Handler();
	}
}

/**
*
* @brief  舵轮改变颜色
* @param  CANx：所使用的CAN通道编号
* @param  RGB 颜色
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ZBW
* @note	  
*/
void ChassisChangeColor(FDCAN_HandleTypeDef* hfdcan, uint8_t DriverNum, RGB_Cmd rgb_cmd)
{
	gRobot.tx_can_cnt++;
	uint16_t identifier = IDENTIFIER_COLOR_SET;
	uint8_t data[3];
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	

	TxMessage.Identifier = DriverNum+ PTP_MODE + DRIVER_SERVER_BASE_ID;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_3;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0;
	
	data[0] = (((identifier<<4)&0xF0) | (rgb_cmd&0x07) | ((LITACC&0x01)<<3));
	data[1] = (((ENABLE&0x01)<<6) | ((ENABLE&0x01)<<5) | ((NORMAL_MODE&0x07)<<2) | ((LITACC&0x06)>>1));
	
	// 发送
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
//		Error_Handler();
	}
}
/**
* @brief  舵轮自动回复位置速度
* @param  driverMsg：对应电机结构体的地址
* @param  data8: 接收驱动器消息数组的首地址
* @author ACTION
*/
void GetSteeringWheelMsg(SteeringWheelMsg_t *driverMsg, uint8_t *data8)//接受电驱发的舵轮位置速度信息
{
	if(data8[0]>>4 == IDENTIFIER_POS_SPD_READ_BCK)
	{
		if((data8[2]&0x80) == 0)
		{
			driverMsg->pos = -(float)((data8[2] << 12) | (data8[1] << 4) | (data8[0]&0x0F));
			driverMsg->pos /= 10.f;
		}
		else 
		{
			driverMsg->pos = (float)(((data8[2]&0x7F) << 12) | (data8[1] << 4) | (data8[0]&0x0F));
			driverMsg->pos /= 10.f;
		}
		
		if((data8[4]&0x80) == 0)
		{
			driverMsg->vel = (float)(((int)(data8[4]) << 8)| (int)(data8[3]));
			driverMsg->vel = driverMsg->vel / 100.f ;
		}
		else
		{
			driverMsg->vel = -(float)(((((int)(data8[4]) & 0x7f) << 8) ) | (int)(data8[3]));
			driverMsg->vel = driverMsg->vel / 100.f ;
		}
//		if(gRobot.steeringWheelMode == SPD_CURR_CTRL_MODE){
//			wheel1Msg.vel /= 100;
//			wheel2Msg.vel /= 100;
//			wheel3Msg.vel /= 100;
//		}
	}
}


/**
* @brief  浮点变量限幅函数
* @param  value：需要限幅数据的地址
* @param  upLimit: 上界
* @param  downLimit: 下界
* @author ACTION
*/
void Saturation_float(float *value, float upLimit, float downLimit)
{	
	if(*value >= upLimit)
	{
		*value	= upLimit;
	}
	else if(*value <= downLimit)
	{
		*value = downLimit;
	}
}
/**
* @brief  UINT8变量限幅函数
* @param  value：需要限幅数据的地址
* @param  upLimit: 上界
* @param  downLimit: 下界
* @author ACTION
*/
void Saturation_uint8_t(uint8_t *value, uint8_t upLimit, uint8_t downLimit)
{
	if(*value >= upLimit)
	{
		*value	= upLimit;
	}
	else if(*value <= downLimit)
	{
		*value = downLimit;
	}
}
/**
* @brief  INT变量限幅函数
* @param  value：需要限幅数据的地址
* @param  upLimit: 上界
* @param  downLimit: 下界
* @author ACTION
*/
void Saturation_int(int *value, int upLimit, int downLimit)
{
	if(*value >= upLimit)
	{
		*value	= upLimit;
	}
	else if(*value <= downLimit)
	{
		*value = downLimit;
	}
}
/**
* @brief  INT变量限幅函数
* @param  value：需要限幅数据的地址
* @param  upLimit: 上界
* @param  downLimit: 下界
* @author ACTION
*/
void Saturation_uint32(uint32_t *value, uint32_t upLimit, uint32_t downLimit)
{
	if(*value >= upLimit)
	{
		*value	= upLimit;
	}
	else if(*value <= downLimit)
	{
		*value = downLimit;
	}
}

/**
* @brief  位置环速度环电流环清零
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫安
*/
void Integral_Cleaded(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = IDENTIFIER_SET_INTEGRAL_CLEARED;
	uint8_t data[1];
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };
	
	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_1;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;	

	data[0] = (identifier>>0)&0xFF;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
			Error_Handler();						 
}


/* USER CODE END */

