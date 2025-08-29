#ifndef __DRIVER_H
#define __DRIVER_H

#include "fdcan.h"
#include "stdint.h"

/*驱动器发送ID基地址*/
#define DRIVER_CLIENT_BASE_ID	0x280

/*驱动器接收ID基地址*/
#define DRIVER_SERVER_BASE_ID	0x300

/*控制标识符*/
#define IDENTIFIER_DRIVER_STATE				0x01
#define IDENTIFIER_CURR_KP_Q				0x02
#define IDENTIFIER_CURR_KI_Q				0x03
#define IDENTIFIER_SPD_KP					0x04
#define IDENTIFIER_SPD_KI					0x05
#define IDENTIFIER_POS_KP					0x06
#define IDENTIFIER_POS_KD					0x07
#define IDENTIFIER_TORQUE_CTRL				0x08
#define IDENTIFIER_VEL_CTRL					0x09
#define IDENTIFIER_POS_CTRL_ABS				0x0A
#define IDENTIFIER_POS_CTRL_REL				0x0B
#define IDENTIFIER_SET_CTRL_MODE			0x0C
#define IDENTIFIER_SET_ACC					0x0D
#define IDENTIFIER_SET_DEC					0x0E
#define IDENTIFIER_SET_TORQUE_LIMIT			0x0F
#define IDENTIFIER_SET_VEL_LIMIT			0x10
#define IDENTIFIER_SET_POS_LIMIT_UP			0x11
#define IDENTIFIER_SET_POS_LIMIT_LOW		0x12
#define IDENTIFIER_SET_INTEGRAL_CLEARED     0x14
#define IDENTIFIER_LIGHTING_STATUS			0x31
#define IDENTIFIER_SET_CLEAR_INTEGRAL		0x1A
#define IDENTIFIER_DELTA_PARA               0x1B
#define IDENTIFIER_CLEAR_POS		        0x1F
#define IDENTIFIER_ARM_RESET_FLAG           0xEF
#define IDENTIFIER_RESET_ZERO               0xFE
#define IDENTIFIER_ELIMINATE_EMPTY_DIATANCE 0xFD
#define IDENTIFIER_RESET_POS_OK				0x9F
/*读取标识符*/	                               
#define IDENTIFIER_READ_TORQUE				0x20
#define IDENTIFIER_READ_VEL					0x21
#define IDENTIFIER_READ_POS					0x22
#define IDENTIFIER_READ_ENCODER_POS			0x23
#define IDENTIFIER_READ_VOL_D				0x24
#define IDENTIFIER_READ_CURR_D				0x25
#define IDENTIFIER_READ_VOL_Q				0x26
#define IDENTIFIER_READ_CURR_Q				0x27
#define IDENTIFIER_READ_SPD_LOOP_OUTPUT		0x28
#define IDENTIFIER_READ_POS_LOOP_OUTPUT		0x29
#define IDENTIFIER_READ_CURR_G              0x2C

#define IDENTIFIER_ENABLE_FDBCK_SIGN		0x50


/*错误标识符*/
#define IDENTIFIER_ENCODER_ERROR		0xEE
#define IDENTIFIER_HARD_FAULT			0xFF

//TODO Value Limits // ptv控制限值 ，需与电驱方面比对数据是否相同
#define P_K_MIN -100.0f * 1000  // 单位rad
#define P_K_MAX 100.0f * 1000
#define V_K_MIN -400.0f * 1000 // 单位rad/s
#define V_K_MAX 400.0f * 1000     
#define KP_K_MIN 0.0f * 1000    //自测
#define KP_K_MAX 500.0f * 1000  //自测
#define KD_K_MIN 0.0f * 1000   //自测
#define KD_K_MAX 100.0f * 1000  //自测
#define T_K_MIN -150.0f * 10000  // 单位N.m
#define T_K_MAX 150.0f * 10000

/*pos loop*/
#define POSITION_CONTROL_KP 60.0f
#define POSITION_CONTROL_KD 0.1f

//#define Other 0x2D
/*驱动器指令模式*/
typedef enum
{
	PTP_MODE = 0x00, 
	BROADCAST_MODE = 0x40
} CommandMode;

typedef struct  //单位：p：rad, v：rad/s, t：N/m
{
  float p_des;
  float	v_des;
  float	kp;
  float	kd;
  float	t_ff;
} joint_control;

typedef struct 
{
    joint_control a, k, h;//对应肩，小腿，大腿
} driver_control;

/*主控接收驱动器消息内容结构体*/
//VEL_MODE/POSMODE/PVT_MODE都能用
typedef struct
{
	float vel;//之前是int,脉冲
	float pos;//之前是int,脉冲
	float ex_curr;//pvt模式下电流
	
	float height;
	int loaded;
	float angle;	
	float torque;//之前是int,脉冲
	int volQ;
	int currQ;
	int currG;
	int volD;
	int currD;
	int encoder;
	int velLoopOut;
	int posLoopOut;
	int enableState;
	uint8_t encoderErr;
	uint8_t hardFault;
	uint8_t Enable_Sign;
	uint8_t reset_success_flag;
	uint8_t reset_pos_ok_flag;
	uint8_t reset_true_flag;
	uint8_t empty_distance_ok_flag;
}driverMsg_t;


void DriverState(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, FunctionalState state);
void VelCtrl(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, int32_t vel);
void PosCtrl(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, int32_t pos);
void SetTorqueLimit(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint32_t limit);
void SetVelLimit(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint32_t limit);
void ReadPVT(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum);
void ReadVel(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum);
void ReadPos(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum);
void ReadCurrG(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum);
void ReadCurrQ(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum);
void GetDriverMsg(driverMsg_t *driverMsg, uint8_t *data8);
int TransformValue(int data32);
void DeltaParaCtrl(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint8_t paraStep);
void TurnParaCtrl(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint8_t paraStep);
void SetClearIntegral(FDCAN_HandleTypeDef* hfdcan, uint8_t DriverNum);
void DriverClearPos(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum);
uint8_t ResetControlMode(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint8_t mode);
void PVTCtrl(FDCAN_HandleTypeDef *hfdcan, uint8_t DriverNum, driver_control *driverMessage);
void UpackCalfMsg(driverMsg_t *driverMsg, uint8_t *data8);
uint8_t ResetFlag(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);
uint8_t IfResetOkFlag(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);
uint8_t EmptyDistanceFlag(FDCAN_HandleTypeDef *hfdcan, CommandMode CMDmode, uint8_t DriverNum);

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
void SetSpeedKP(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_speed);

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
void SetSpeedKI(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint32_t i_speed);

/*test*/
void SetPosKD(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint32_t d_pos);
void SetPosKP(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_pos);

#endif


