#include "my_motor_driver.h"
#include "my_chassis_driver.h"
#include "my_action_control.h"
#include "my_main.h"

/*******************************控制驱动器命令************************************/
/**
* @brief  发送标志位，让电驱重新跑复位程序
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @retval 通信是否成功
* @author ACTION
*/
uint8_t ResetFlag(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum) 
{
//		DEBUG("RESET%d!!!\r\n", DriverNum);
    uint16_t identifier = IDENTIFIER_ARM_RESET_FLAG;
    FDCAN_TxHeaderTypeDef TxMessage = {0};
    uint8_t data[2];

    TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    TxMessage.IdType = FDCAN_STANDARD_ID;
    TxMessage.TxFrameType = FDCAN_DATA_FRAME;
    TxMessage.DataLength = FDCAN_DLC_BYTES_2;
    TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
    TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
    TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxMessage.MessageMarker = 0x01;

    data[0] = (identifier >> 0) & 0xFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) == HAL_OK) 
		{
        return 0;
    } else 
	  {
        return 1;
    }
}

/**
* @brief  发送标志位，让电驱重新跑复位程序
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @retval 通信是否成功
* @author ACTION
*/
uint8_t ResetZero(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum) 
{
//		DEBUG("RESET%d!!!\r\n", DriverNum);
    uint16_t identifier = IDENTIFIER_RESET_ZERO;
    FDCAN_TxHeaderTypeDef TxMessage = {0};
    uint8_t data[2];

    TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    TxMessage.IdType = FDCAN_STANDARD_ID;
    TxMessage.TxFrameType = FDCAN_DATA_FRAME;
    TxMessage.DataLength = FDCAN_DLC_BYTES_2;
    TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
    TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
    TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxMessage.MessageMarker = 0x01;

    data[0] = (identifier >> 0) & 0xFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) == HAL_OK) 
		{
        return 0;
    } else 
	  {
        return 1;
    }
}

/**
* @brief  发送标志位，让电驱重新跑复位程序
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @retval 通信是否成功
* @author ACTION
*/
uint8_t IfResetOkFlag(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum) 
{
    uint16_t identifier = 0x40 + IDENTIFIER_RESET_POS_OK;
    FDCAN_TxHeaderTypeDef TxMessage = {0};
    uint8_t data[2];

    TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    TxMessage.IdType = FDCAN_STANDARD_ID;
    TxMessage.TxFrameType = FDCAN_DATA_FRAME;
    TxMessage.DataLength = FDCAN_DLC_BYTES_2;
    TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
    TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
    TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxMessage.MessageMarker = 0x01;

    data[0] = (identifier >> 0) & 0xFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) == HAL_OK) 
		{
        return 0;
    } else 
	  {
        return 1;
    }
}
/**
* @brief  发送标志位，让电驱重新跑复位程序
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @retval 通信是否成功
* @author ACTION
*/
uint8_t EmptyDistanceFlag(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum) 
{
    uint16_t identifier = IDENTIFIER_ELIMINATE_EMPTY_DIATANCE;
    FDCAN_TxHeaderTypeDef TxMessage = {0};
    uint8_t data[2];

    TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    TxMessage.IdType = FDCAN_STANDARD_ID;
    TxMessage.TxFrameType = FDCAN_DATA_FRAME;
    TxMessage.DataLength = FDCAN_DLC_BYTES_2;
    TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
    TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
    TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxMessage.MessageMarker = 0x01;

    data[0] = (identifier >> 0) & 0xFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) == HAL_OK) 
		{
        return 0;
    }else return 1;
}
/**
* @brief  切换电机控制模式
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  mode: 1 vel; 2 pos; 4 torque（pvt）
* @retval 通信是否成功
* @author ACTION
*/
uint8_t ResetControlMode(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint8_t mode) 
{
    uint16_t identifier = IDENTIFIER_SET_CTRL_MODE;
    FDCAN_TxHeaderTypeDef TxMessage = {0};
    uint8_t data[2];

    TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
    TxMessage.IdType = FDCAN_STANDARD_ID;
    TxMessage.TxFrameType = FDCAN_DATA_FRAME;
    TxMessage.DataLength = FDCAN_DLC_BYTES_2;
    TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMessage.BitRateSwitch = FDCAN_BRS_OFF;
    TxMessage.FDFormat = FDCAN_CLASSIC_CAN;
    TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxMessage.MessageMarker = 0x01;

    data[0] = (identifier >> 0) & 0xFF;
    data[1] = mode;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) == HAL_OK) 
		{
        return 0;
    } else 
	  {
        return 1;
    }
}
/**
* @brief  驱动器状态(使能或失能)
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
			PTP_MODE: 点对点模式
			BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  state：状态, 范围: 
			ENABLE:使能
			DISABLE:失能
* @author ZBW
* @note  电机的失能为驱动器停止对电机的驱动，可灵活使用。
*/
void DriverState(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, FunctionalState state)
{
	gRobot.tx_can_cnt++;
	uint16_t identifier = IDENTIFIER_DRIVER_STATE;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t data[2];
	
	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_2;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;	

	data[0] = identifier&0xFF;
	data[1] = state;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
//		Error_Handler();
	}	
}


/**
* @brief  电机速度控制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  vel: 速度，单位：脉冲每秒, 范围：-1024 * 4096 ~ 1024 * 4096
* @author ZBW
*/
void VelCtrl(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, int32_t vel)
{
	uint16_t identifier = IDENTIFIER_VEL_CTRL;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t data[4];
	
	/*限幅*/	
	Saturation_int(&vel,1024 * 4096,-1024 * 4096);
	
	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_4;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;	

	data[0] = (identifier>>0)&0xFF;

	/*发送符号位*/
	if(vel >= 0)
	{
		data[1] = (vel>>0)&0xFF;
		data[2] = (vel>>8)&0xFF;
		data[3] = (vel>>16)&0xFF; 
	}
	else if(vel < 0)
	{
		data[1] = (-vel>>0)&0xFF;
		data[2] = (-vel>>8)&0xFF;
		data[3] = ((-vel>>16)&0xFF) | 0x80;
	}
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
		//Error_Handler();
	}
}

/**
* @brief  电机位置控制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  pos:位置命令，单位：脉冲, 范围：-1024 * 4096 ~ 1024 * 4096
* @author ZBW
*/
void PosCtrl(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, int32_t pos)
{
	gRobot.tx_can_cnt++;
	uint16_t identifier = IDENTIFIER_POS_CTRL_ABS;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t data[4];
	
	/*限幅*/	
	Saturation_int(&pos,1024 * 4096,-1024 * 4096);
	
	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_4;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;	

	data[0] = (identifier>>0)&0xFF;

	/*发送符号位*/
	if(pos >= 0)
	{
		data[1] = (pos>>0)&0xFF;
		data[2] = (pos>>8)&0xFF;
		data[3] = (pos>>16)&0xFF; 
	}
	else if(pos < 0)
	{
		data[1] = (-pos>>0)&0xFF;
		data[2] = (-pos>>8)&0xFF;
		data[3] = ((-pos>>16)&0xFF) | 0x80;
	}
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
		//Error_Handler();
	}	
}

int float_to_uint(float x, float x_min, float x_max, int bits)// 浮点转无符号整数
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span   = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)//浮点转无符号整数
{
    /// converts unsigned int to float, given range and number of bits ///
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief  电机力位速混合控制
 * @param  CANx：所使用的CAN通道编号
 * @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
 * @param  driverMessage: 结构体，包含要发给电机的所有数据
 * @note   从狗移植的，先用着
 */
void PVTCtrl(FDCAN_HandleTypeDef *hfdcan, uint8_t DriverNum, driver_control *driverMessage)
{
    FDCAN_TxHeaderTypeDef TxMessage = {0};
    uint8_t data[8];
    uint16_t identifier;
    float p, v, t, kp, kd;

    p  = driverMessage->k.p_des;
    v  = driverMessage->k.v_des;
    t  = driverMessage->k.t_ff;
    kp = driverMessage->k.kp;
    kd = driverMessage->k.kd;
    /// convert floats to unsigned ints /// 浮点转无符号整数
    uint16_t p_int  = float_to_uint(p, P_K_MIN, P_K_MAX, 16);
    uint16_t v_int  = float_to_uint(v, V_K_MIN, V_K_MAX, 12);
    uint16_t kp_int = float_to_uint(kp, KP_K_MIN, KP_K_MAX, 12);
    uint16_t kd_int = float_to_uint(kd, KD_K_MIN, KD_K_MAX, 12);
    uint16_t t_int  = float_to_uint(t, T_K_MIN, T_K_MAX, 12);

    identifier                    = IDENTIFIER_TORQUE_CTRL;
    TxMessage.Identifier          = DRIVER_SERVER_BASE_ID + DriverNum;
    TxMessage.IdType              = FDCAN_STANDARD_ID;
    TxMessage.TxFrameType         = FDCAN_DATA_FRAME;
    TxMessage.DataLength          = FDCAN_DLC_BYTES_8;
    TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMessage.BitRateSwitch       = FDCAN_BRS_OFF;
    TxMessage.FDFormat            = FDCAN_CLASSIC_CAN;
    TxMessage.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxMessage.MessageMarker       = 0x01;

    data[0] = p_int >> 8;
    data[1] = p_int & 0xFF;
    data[2] = v_int >> 4;
    data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    data[4] = kp_int & 0xFF;
    data[5] = kd_int >> 4;
    data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
		data[7] = t_int & 0xff;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK) {
//		Error_Handler();
    }
}

/**
* @brief  配置电机最大转矩
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  limit：最大转矩, 单位: 毫牛米, 范围：0 ~ 100 * 1000
* @author ACTION
* @note 根据电机的最大输出扭矩（XC5000:2.5N）、机构动作需要的力，来给定。
*/
void SetTorqueLimit(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint32_t limit)
{
	uint16_t identifier = IDENTIFIER_SET_TORQUE_LIMIT;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t data[4];
	
	/*限幅*/	
	Saturation_uint32(&limit,100 * 1000,0);
	
	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_4;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;	

	data[0] = (identifier>>0)&0xFF;
	data[1] = (limit>>0)&0xFF;
	data[2] = (limit>>8)&0xFF;
	data[3] = (limit>>16)&0xFF;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
//		Error_Handler();
	}	
}

/**
* @brief  配置速度环最大期望速度
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  limit：速度限制，单位：脉冲每秒, 范围：0 ~ 1024 * 4096
* @author ACTION
* @note 基本用于限制位置环的最大速度
*/
void SetVelLimit(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint32_t limit)
{
	gRobot.tx_can_cnt++;
	uint16_t identifier = IDENTIFIER_SET_VEL_LIMIT;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t data[4];

	/*限幅*/	
	Saturation_uint32(&limit,1024 * 4096,0);

	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_4;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;	

	data[0] = (identifier>>0)&0xFF;
	data[1] = (limit>>0)&0xFF;
	data[2] = (limit>>8)&0xFF;
	data[3] = (limit>>16)&0xFF;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
//		Error_Handler();
	}
}

/**********************************读取驱动器数据命令*************************************/
/**
* @brief  读取电机速度
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲每秒
*/
//发送读取速度指令时回顺便返给位置信息，使用后无需再次调用ReadPos
void ReadVel(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum)
{
	gRobot.tx_can_cnt++;
	uint16_t identifier = 0x40 + IDENTIFIER_READ_VEL;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t data[1];

	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_1;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;	

	data[0] = (identifier>>0)&0xFF;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
//		Error_Handler();
	}		
}

/**
* @brief  读取电机位置,速度，力矩
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲 (不要在力矩（pvt）控制模式下用这个函数，驱动器在力矩控制模式下会自动发当前电机状态，如果使用会导致驱动器将此函数的data视为发送期望函数的data)
*/
void ReadPVT(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum)
{
	gRobot.tx_can_cnt++;
	uint16_t identifier = 0x40 + 0x32;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t data[1];

	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_1;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;	

	data[0] = (identifier>>0)&0xFF;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
//		Error_Handler();
	}		
}

/**
* @brief  读取电机位置
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲
*/
void ReadPos(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum)
{
	gRobot.tx_can_cnt++;
	uint16_t identifier = 0x40 + IDENTIFIER_READ_POS;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t data[1];

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
	{
//		Error_Handler();
	}			
}

/**
* @brief  读取驱动器母线电流
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：安培 * 1000
*/

void ReadCurrG(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_CURR_G;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t data[1];
	
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
	{
//		Error_Handler();
	}

	
}

/**
* @brief  读取驱动器Q轴电流
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：安培 * 1000
*/

void ReadCurrQ(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_CURR_Q;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t data[1];
	
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
	{
//		Error_Handler();
	}

	
}

/**
* @brief  delta电机调参
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  pos:位置命令，单位：脉冲, 范围：-1024 * 4096 ~ 1024 * 4096
* @author ZBW
*/
void DeltaParaCtrl(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint8_t paraStep)
{
	uint16_t identifier = IDENTIFIER_DELTA_PARA;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t data[4];
	
	/*限幅*/	
//	Saturation_int(&pos,1024 * 4096,-1024 * 4096);
	
	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_4;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;	

	data[0] = (identifier>>0)&0xFF;

	/*发送符号位*/
	if(paraStep >= 0)
	{
		data[1] = (paraStep>>0)&0xFF;
		data[2] = (paraStep>>8)&0xFF;
		data[3] = (paraStep>>16)&0xFF; 
	}
	else if(paraStep < 0)
	{
		data[1] = (-paraStep>>0)&0xFF;
		data[2] = (-paraStep>>8)&0xFF;
		data[3] = ((-paraStep>>16)&0xFF) | 0x80;
	}
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
//		Error_Handler();
	}	
}
void TurnParaCtrl(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint8_t paraStep)
{
	uint16_t identifier = IDENTIFIER_DELTA_PARA;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t data[4];
	
	/*限幅*/	
//	Saturation_int(&pos,1024 * 4096,-1024 * 4096);
	
	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_4;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;	

	data[0] = (identifier>>0)&0xFF;

	/*发送符号位*/
	if(paraStep >= 0)
	{
		data[1] = (paraStep>>0)&0xFF;
		data[2] = (paraStep>>8)&0xFF;
		data[3] = (paraStep>>16)&0xFF; 
	}
	else if(paraStep < 0)
	{
		data[1] = (-paraStep>>0)&0xFF;
		data[2] = (-paraStep>>8)&0xFF;
		data[3] = ((-paraStep>>16)&0xFF) | 0x80;
	}
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
//		Error_Handler();
	}	
}

void UpackCalfMsg(driverMsg_t *driverMsg, uint8_t *pvd_data8)//pvt模式获取电机消息的值
{
    int16_t pos_int;
    int16_t spd_int;
    int16_t cur_int;
		int16_t ex_cur_int;
	
		pos_int = (pvd_data8[0] << 8 | (pvd_data8[1]));
		spd_int = (pvd_data8[2] << 8 | (pvd_data8[3]));
		cur_int = (pvd_data8[4] << 8 | (pvd_data8[5]));
		ex_cur_int = (pvd_data8[6] << 8 | (pvd_data8[7]));
//	DEBUG("%d %d %d\r\n", pos_int, spd_int, cur_int);
	
//	  pos_int = pos_int / 0.00078125f / 2097152.f;
//	  spd_int = spd_int / 0.06501587302f / 180;

		driverMsg->pos = ((float)pos_int / 100.f); // 电机位置 rad

		//driverMsg->vel = (float)(spd_int / 180 * PI); // 电机速度 rad/s 
		driverMsg->vel = (float)(spd_int);

		driverMsg->torque = (float)(cur_int / 10.f) ; // 实际电流 A
	
		driverMsg->ex_curr = (float)(ex_cur_int / 10.f);	
}

/**
* @brief  获取电机消息的值
* @param  *driverMsg：对应电机消息结构体的地址
* @param  *data8: 接收驱动器消息数组的首地址
* @author ACTION
* @note   方便用户使用 
*/
void GetDriverMsg(driverMsg_t *driverMsg, uint8_t *data8)
{
	union receive
	{
		uint8_t data8[4];
		int data32;
		float dataf;
	}receiveMsg;
	
	for(int i=0; i<4; i++)
		receiveMsg.data8[i] = data8[i];
	if(receiveMsg.data8[0] == IDENTIFIER_ENABLE_FDBCK_SIGN)
	{
		//已使能標志位
		driverMsg->Enable_Sign = 1;
		driverMsg->enableState = TransformValue(receiveMsg.data32);
	}	
	if(receiveMsg.data8[0] == IDENTIFIER_READ_VEL)
	{
		//获取脉冲速度
		driverMsg->vel = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_POS)
	{
		//获取脉冲位置
		driverMsg->pos = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_ENCODER_POS)
	{
		//获取编码器脉冲
		driverMsg->encoder = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_TORQUE)
	{
		//获取电磁转矩
		driverMsg->torque = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_VOL_Q)
	{
		//获取交轴电压
		driverMsg->volQ = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_CURR_G)
	{
		driverMsg->currG = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_CURR_Q)
	{
		//获取转矩电流
		driverMsg->currQ = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_CURR_D)
	{
		//获取转矩电流
		driverMsg->currD = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_VOL_D)
	{
		//获取转矩电流
		driverMsg->volD = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_SPD_LOOP_OUTPUT)
	{
		//获取转矩电流
		driverMsg->velLoopOut = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_POS_LOOP_OUTPUT)
	{
		//获取转矩电流
		driverMsg->posLoopOut = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_ENCODER_ERROR)
	{
		driverMsg->encoderErr = 1;
	}
	if(receiveMsg.data8[0] == IDENTIFIER_HARD_FAULT)
	{
		driverMsg->hardFault = 1;
	}
	if(receiveMsg.data8[0] == IDENTIFIER_RESET_POS_OK)
	{
		//DEBUG("%d, %d\r\n",receiveMsg.data8[1], receiveMsg.data8[2]);
		driverMsg->reset_pos_ok_flag = receiveMsg.data8[1];
		driverMsg->reset_true_flag = receiveMsg.data8[2];
		driverMsg->empty_distance_ok_flag = receiveMsg.data8[3];
	}
	
}
/*将三个高字节的数据移到低三字节并提取符号位*/
int TransformValue(int data32)
{
	if((data32&0x80000000) == 0)
	{
		return ((data32>>8)&0x7FFFFF);
	}
	else
	{
		return(-((data32>>8)&0x7FFFFF)); 
	}
}

/**
* @brief  清除速度环积分
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note
*/
void SetClearIntegral(FDCAN_HandleTypeDef* hfdcan, uint8_t DriverNum)
{
	uint16_t identifier = IDENTIFIER_SET_CLEAR_INTEGRAL;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t data[4];

	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}

	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + 0x00 + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_4;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;	

	data[0] = (identifier>>0)&0xFF;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
//		Error_Handler();
	}			
}

/**
* @brief  配置速度环kp
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  p_speed：速度环kp值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/

void SetSpeedKP(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_speed)
{
	uint16_t identifier = IDENTIFIER_SPD_KP;
	uint32_t data = p_speed;	
	uint8_t mbox;
	uint16_t timeout = 0;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t Data[4];
	
	/*限幅*/
	//fix me 添加正确的限幅
	if(data > 100 * 1000)
	{
		data = 100 * 1000;
	}
	
	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_4;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;
	
	Data[0] = (identifier>>0)&0xFF;
	Data[1] = (data>>0)&0xFF;
	Data[2] = (data>>8)&0xFF;
	Data[3] = (data>>16)&0xFF;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, Data) != HAL_OK)
	{
//		Error_Handler();
	}		
  
}

/**
* @brief  配置速度环ki
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  i_speed：速度环ki值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetSpeedKI(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint32_t i_speed)
{
	uint16_t identifier = IDENTIFIER_SPD_KI;
	uint32_t data = i_speed;	
	uint8_t mbox;
	uint16_t timeout = 0;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t Data[4];
	
	/*限幅*/
	//fix me 添加正确的限幅
	if(data > 100 * 1000)
	{
		data = 100 * 1000;
	}
	
	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_4;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;
	
	Data[0] = (identifier>>0)&0xFF;
	Data[1] = (data>>0)&0xFF;
	Data[2] = (data>>8)&0xFF;
	Data[3] = (data>>16)&0xFF;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, Data) != HAL_OK)
	{
//		Error_Handler();
	}		
}

/**
* @brief  驱动器累计里程清零
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ZJY
* @note  电机的累积里程进行清零。
*/
void DriverClearPos(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = IDENTIFIER_CLEAR_POS;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t data[2];
	
	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_2;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;	

	data[0] = identifier&0xFF;
	data[1] = 0u;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, data) != HAL_OK)
	{
//		Error_Handler();
	}	
}

/**
* @brief  配置位置环kp
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  p_pos：位置环kp值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note   乘60
*/
void SetPosKP(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_pos)
{
	uint16_t identifier = IDENTIFIER_POS_KP;
	uint32_t data = p_pos;	
	uint8_t mbox;
	uint16_t timeout = 0;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t Data[4];
	
	/*限幅*/
	//fix me 添加正确的限幅
	if(data > 100 * 1000 * POSITION_CONTROL_KP)
	{
		data = 100 * 1000 * POSITION_CONTROL_KP;
	}
	
	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_4;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;
	
	Data[0] = (identifier>>0)&0xFF;
	Data[1] = (data>>0)&0xFF;
	Data[2] = (data>>8)&0xFF;
	Data[3] = (data>>16)&0xFF;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, Data) != HAL_OK)
	{
//		Error_Handler();
	}		
  
}

/**
* @brief  配置位置环kd
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  d_pos：位置环kd值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note   乘0.1
*/
void SetPosKD(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint32_t d_pos)
{
	uint16_t identifier = IDENTIFIER_POS_KP;
	uint32_t data = d_pos;	
	uint8_t mbox;
	uint16_t timeout = 0;
	FDCAN_TxHeaderTypeDef TxMessage = { 0 };	
	uint8_t Data[4];
	
	/*限幅*/
	//fix me 添加正确的限幅
	if(data > 100 * 1000)
	{
		data = 100 * 1000;
	}
	
	TxMessage.Identifier = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.IdType = FDCAN_STANDARD_ID;
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;
	TxMessage.DataLength = FDCAN_DLC_BYTES_4;
	TxMessage.ErrorStateIndicator =	FDCAN_ESI_ACTIVE;
	TxMessage.BitRateSwitch =	FDCAN_BRS_OFF;
	TxMessage.FDFormat = 	FDCAN_CLASSIC_CAN;
	TxMessage.TxEventFifoControl = 	FDCAN_NO_TX_EVENTS;
	TxMessage.MessageMarker = 0x01;
	
	Data[0] = (identifier>>0)&0xFF;
	Data[1] = (data>>0)&0xFF;
	Data[2] = (data>>8)&0xFF;
	Data[3] = (data>>16)&0xFF;
	
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, Data) != HAL_OK)
	{
//		Error_Handler();
	}		
  
}