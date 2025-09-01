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
//	uint8_t canNodeId = 0;
//	uint8_t data8[9] = {0};

	if(hfdcan == &hfdcan1)
	{
		FDCAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data);
		
		uint8_t motor_id = rx_header.Identifier - 0x50;
		if(motor_id < 4)
		{
			R80_ParseFeedback(rx_header.Identifier, rx_data, &motors[motor_id]);
		}
//		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxMessage0, data8);
//		canNodeId = (RxMessage0.Identifier - 0x280);
//		switch(canNodeId)
//		{
//			case UPPER_ARM_ID:
//			{
//				UpackCalfMsg(&upperArmMsg, data8);
//				GetDriverMsg(&upperArmMsg, data8);
//				
//				HB.upper_arm = 0;
//				firstComFlag.upper_arm = 1;
//				break;
//			}
//			
//			case FOREARM_ID:
//			{
//				UpackCalfMsg(&forearmMsg, data8);
//				GetDriverMsg(&forearmMsg ,data8);

//				HB.forearm = 0;
//				firstComFlag.forearm = 1;
//				break;
//			}
//			
//			case WRIST_ID:
//			{
//				UpackCalfMsg(&wristMsg, data8);
//				GetDriverMsg(&wristMsg ,data8);
//				
//				HB.wrist = 0;
//				firstComFlag.wrist = 1;
//				break;
//			}
//		}
	}
//	memset(&RxMessage0,0,sizeof(RxMessage0));
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

/*******************************R80电机FDCAN通信************************************/

FDCAN_HandleTypeDef R80_FDCAN;
FDCAN_TxHeaderTypeDef r80_tx_message;
uint8_t r80_fdcan_send_data[8];
#define ENCODER_COUNTS_PER_REV 8192    // 编码器每圈计数
r80_feedback_t motors[4];

//电机使能控制
void R80_Enable(uint8_t motor1_enable, uint8_t motor2_enable, uint8_t motor3_enable, uint8_t motor4_enable)
{ 
    r80_tx_message.Identifier = CAN_ID_ENABLE_CTRL;
    r80_tx_message.IdType = FDCAN_STANDARD_ID;
    r80_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    r80_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    r80_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    r80_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    r80_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    r80_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    
    r80_fdcan_send_data[0] = motor1_enable;
    r80_fdcan_send_data[1] = motor2_enable;
    r80_fdcan_send_data[2] = motor3_enable;
    r80_fdcan_send_data[3] = motor4_enable;
    r80_fdcan_send_data[4] = 0;
    r80_fdcan_send_data[5] = 0;
    r80_fdcan_send_data[6] = 0;
    r80_fdcan_send_data[7] = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&R80_FDCAN, &r80_tx_message, r80_fdcan_send_data);
}

//电机驱动控制
void R80_Drive(uint16_t motor1_val, uint16_t motor2_val, uint16_t motor3_val, uint16_t motor4_val)
{
	r80_tx_message.Identifier = CAN_ID_DRIVE_CTRL;
  r80_tx_message.IdType = FDCAN_STANDARD_ID;
  r80_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  r80_tx_message.DataLength = FDCAN_DLC_BYTES_8;
  r80_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  r80_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
  r80_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
  r80_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	
	r80_fdcan_send_data[0] = (motor1_val >> 8) & 0xFF;
	r80_fdcan_send_data[1] = motor1_val & 0xFF;
	r80_fdcan_send_data[2] = (motor2_val >> 8) & 0xFF;
	r80_fdcan_send_data[3] = motor2_val & 0xFF;
	r80_fdcan_send_data[4] = (motor3_val >> 8) & 0xFF;
	r80_fdcan_send_data[5] = motor3_val & 0xFF;
	r80_fdcan_send_data[6] = (motor4_val >> 8) & 0xFF;
	r80_fdcan_send_data[7] = motor4_val & 0xFF;
	
	HAL_FDCAN_AddMessageToTxFifoQ(&R80_FDCAN, &r80_tx_message, r80_fdcan_send_data);
}

//电机保存指令
void R80_Save(uint8_t save_params, uint8_t save_position)
{
	r80_tx_message.Identifier = CAN_ID_SAVE_CTRL;
  r80_tx_message.IdType = FDCAN_STANDARD_ID;
  r80_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  r80_tx_message.DataLength = FDCAN_DLC_BYTES_8;
  r80_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  r80_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
  r80_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
  r80_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	
	r80_fdcan_send_data[0] = save_params;
	r80_fdcan_send_data[1] = save_position;
	r80_fdcan_send_data[2] = 0;
	r80_fdcan_send_data[3] = 0;
	r80_fdcan_send_data[4] = 0;
	r80_fdcan_send_data[5] = 0;
	r80_fdcan_send_data[6] = 0;
	r80_fdcan_send_data[7] = 0;
	
	HAL_FDCAN_AddMessageToTxFifoQ(&R80_FDCAN, &r80_tx_message, r80_fdcan_send_data);
}

//参数设置
void R80_SetParam(uint8_t motor_id, uint8_t param_id, uint8_t set_val1, uint8_t set_val2, uint8_t set_val3, uint8_t set_val4, uint8_t set_val5, uint8_t set_val6)
{
	r80_tx_message.Identifier = CAN_ID_PARAM_CTRL;
  r80_tx_message.IdType = FDCAN_STANDARD_ID;
  r80_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  r80_tx_message.DataLength = FDCAN_DLC_BYTES_8;
  r80_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  r80_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
  r80_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
  r80_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	
	r80_fdcan_send_data[0] = motor_id;
	r80_fdcan_send_data[1] = param_id;
	r80_fdcan_send_data[2] = set_val1;
	r80_fdcan_send_data[3] = set_val2;
	r80_fdcan_send_data[4] = set_val3;
	r80_fdcan_send_data[5] = set_val4;
	r80_fdcan_send_data[6] = set_val5;
	r80_fdcan_send_data[7] = set_val6;
	
	HAL_FDCAN_AddMessageToTxFifoQ(&R80_FDCAN, &r80_tx_message, r80_fdcan_send_data);
}

//PID参数转换
void pid_to_data(float kp, float ki, float kd, uint8_t* data)
{
	kp = (kp < 0) ? 0 : kp;
	kp = (kp > 50) ? 50 : kp;
	ki = (ki < 0) ? 0 : ki;
	ki = (ki > 50) ? 50 : ki;
	kd = (kd < 0) ? 0 : kd;
	kd = (kd > 50) ? 50 : kd;
	
	uint16_t kp_val =(uint16_t)((kp / 50.0f) * 65535.0f);
	uint16_t ki_val =(uint16_t)((ki / 50.0f) * 65535.0f);
	uint16_t kd_val =(uint16_t)((kd / 50.0f) * 65535.0f);
	
	data[0] = (kp_val >> 8) & 0xFF;
	data[1] = kp_val & 0xFF;
	data[2] = (ki_val >> 8) & 0xFF;
	data[3] = ki_val & 0xFF;
	data[4] = (kd_val >> 8) & 0xFF;
	data[5] = kd_val & 0xFF;
}

//设置电机ID
void R80_SetMotorID(uint8_t current_id, uint8_t new_id)
{
    uint8_t data[6] = {new_id, 0, 0, 0, 0, 0};
    R80_SetParam(current_id, PARAM_ID_MOTOR_ID, data[0], data[1], data[2], data[3], data[4], data[5]);
}

//设置过流保护值
void R80_Set_CurrentLimit(uint8_t motor_id, uint8_t current_amps)
{
    uint8_t data[6] = {current_amps, 0, 0, 0, 0, 0};
    R80_SetParam(motor_id, PARAM_ID_OVER_CURRENT, data[0], data[1], data[2], data[3], data[4], data[5]);
}

//设置电机工作模式
void R80_Set_Mode(uint8_t motor_id, uint8_t mode)
{
    uint8_t data[6] = {mode, 0, 0, 0, 0, 0};
    R80_SetParam(motor_id, PARAM_ID_RUN_MODE, data[0], data[1], data[2], data[3], data[4], data[5]);
}

//设置速度环PID
void R80_Velocity_PID(uint8_t motor_id, float kp, float ki, float kd)
{
    uint8_t data[6];
    pid_to_data(kp, ki, kd, data);
    R80_SetParam(motor_id, PARAM_ID_VEL_PID, data[0], data[1], data[2], data[3], data[4], data[5]);
}

//设置位置环PID
void R80_Position_PID(uint8_t motor_id, float kp, float ki, float kd)
{
		uint8_t data[6];
		pid_to_data(kp, ki, kd, data);
		R80_SetParam(motor_id, PARAM_ID_POS_PID, data[0], data[1], data[2], data[3], data[4], data[5]);
}

//保存参数
void R80_Save_Param(uint8_t motor_id)
{
    R80_Save(R80_SAVE_PARAMS, 0);
}

//保存位置
void R80_Save_Pos(uint8_t motor_id)
{
    R80_Save(0, R80_SAVE_POS);
}

//单个电机控制
void R80_SingleMotor_Enable(uint8_t motor_id)
{
    uint8_t enable_data[4] = { 0 };
    enable_data[motor_id - 1] = MOTOR_CMD_ENABLE;
    R80_Enable(enable_data[0], enable_data[1], enable_data[2], enable_data[3]);
}

void R80_SingleMotor_Disable(uint8_t motor_id)
{
    uint8_t disable_data[4] = { 0 };
    disable_data[motor_id - 1] = MOTOR_CMD_DISABLE;
    R80_Enable(disable_data[0], disable_data[1], disable_data[2], disable_data[3]);
}

void R80_SingleMotor_Calib(uint8_t motor_id)
{
    uint8_t calib_data[4] = { 0 };
    calib_data[motor_id - 1] = MOTOR_CMD_CALIB;
    R80_Enable(calib_data[0], calib_data[1], calib_data[2], calib_data[3]);
}

//所有电机控制
void R80_AllMotor_Enable(void)
{
    R80_Enable(MOTOR_CMD_ENABLE, MOTOR_CMD_ENABLE, MOTOR_CMD_ENABLE, MOTOR_CMD_ENABLE);
}

void R80_AllMotor_Disable(void)
{
    R80_Enable(MOTOR_CMD_DISABLE, MOTOR_CMD_DISABLE, MOTOR_CMD_DISABLE, MOTOR_CMD_DISABLE);
}

void R80_AllMotor_Calib(void)
{
    R80_Enable(MOTOR_CMD_CALIB, MOTOR_CMD_CALIB, MOTOR_CMD_CALIB, MOTOR_CMD_CALIB);
}

//浮点数转换为指令值
uint16_t Change_ValueForm(float value, float Limit)
{
	int32_t scaled = (int32_t)(32767.0f * value / Limit);
	
	scaled = (scaled < -32768) ? -32768 : scaled;
  scaled = (scaled > 32767) ? 32767 : scaled;
	
	return (uint16_t)(scaled & 0xFFFF);
}

//电流控制指令
void R80_Current_cmd(float motor1_Current, float motor2_Current, float motor3_Current, float motor4_Current)
{
	uint16_t current_values[4] = {
		Change_ValueForm(motor1_Current, 30.0f),
		Change_ValueForm(motor2_Current, 30.0f),
		Change_ValueForm(motor3_Current, 30.0f),
		Change_ValueForm(motor4_Current, 30.0f)
	};
	
	R80_Drive(current_values[0], current_values[1], current_values[2], current_values[3]);
}

//速度控制指令
void R80_Velocity_cmd(float motor1_Velocity, float motor2_Velocity, float motor3_Velocity, float motor4_Velocity)
{
	uint16_t velocity_values[4] = {
		Change_ValueForm(motor1_Velocity, 3.3f),
		Change_ValueForm(motor2_Velocity, 3.3f),
		Change_ValueForm(motor3_Velocity, 3.3f),
		Change_ValueForm(motor4_Velocity, 3.3f)
	};
	
	R80_Drive(velocity_values[0], velocity_values[1], velocity_values[2], velocity_values[3]);
}

//位置控制指令
void R80_Position_cmd(float motor1_Position, float motor2_Position, float motor3_Position, float motor4_Position)
{
	uint16_t position_values[4] = {
		Change_ValueForm(motor1_Position, 50.0f),
		Change_ValueForm(motor2_Position, 50.0f),
		Change_ValueForm(motor3_Position, 50.0f),
		Change_ValueForm(motor4_Position, 50.0f)
	};
	
	R80_Drive(position_values[0], position_values[1], position_values[2], position_values[3]);
}

// ==================== 反馈解析函数 ====================
void R80_ParseFeedback(uint32_t std_id, const uint8_t data[8], r80_feedback_t* feedback)
{
    if (feedback == NULL) return;
    
    // 提取电机ID
    feedback->motor_id = (std_id >= 0x50 && std_id <= 0x5F) ? (std_id - 0x50) : 0xFF;
    
    // 解析原始数据
    int16_t raw_vel = (int16_t)(((data[0] << 8) & 0xFF00) | (data[1] & 0x00FF));
    int16_t raw_cur = (int16_t)(((data[2] << 8) & 0xFF00) | (data[3] & 0x00FF));
    feedback->raw_position = (int16_t)(((data[4] << 8) & 0xFF00) | (data[5] & 0x00FF));
    
    feedback->velocity_rps = raw_vel / 100.0f;
    feedback->current_a = raw_cur / 100.0f;
    feedback->error_code = (r80_error_code_t)data[6];
    feedback->bus_voltage = data[7];
    
    // 更新位置信息
    R80_UpdatePosition(feedback);
}

const char* R80_ErrorToString(r80_error_code_t error_code)
{
    switch (error_code) {
        case R80_ERROR_UNDER_VOLTAGE: return "低压";
        case R80_ERROR_OVER_VOLTAGE:  return "过压";
        case R80_ERROR_OVER_CURRENT:  return "过流";
        case R80_ERROR_OVER_TEMP:     return "过温";
        case R80_ERROR_NONE:
        default: return "无错误";
    }
}

void R80_InitFeedback(r80_feedback_t* feedback, uint8_t motor_id)
{
    memset(feedback, 0, sizeof(r80_feedback_t));
    feedback->motor_id = motor_id;
}

void R80_UpdatePosition(r80_feedback_t* feedback)
{
    // 计算位置增量，处理编码器溢出
    int32_t delta = feedback->raw_position - feedback->last_raw_position;
    if (delta > 32767) delta -= 65536;
    else if (delta < -32767) delta += 65536;
    
    // 更新累计位置
    feedback->total_count += delta;
    feedback->total_revolutions = (float)feedback->total_count / ENCODER_COUNTS_PER_REV;
    feedback->total_angle_deg = feedback->total_revolutions * 360.0f;
    
    // 保存当前原始位置
    feedback->last_raw_position = feedback->raw_position;
}
/* USER CODE END */

