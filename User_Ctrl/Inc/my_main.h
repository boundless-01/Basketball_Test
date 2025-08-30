#ifndef _MY_MAIN_H
#define _MY_MAIN_H

#include "main.h"
#include "string.h"

#include "my_communication.h"
#include "my_motor_driver.h"
#include "usart.h"
#include "stdio.h"
#include "my_usart.h"

//(要调试可用，现在用不到)
#define DEBUG_UART_HANDLER     huart8
#define DEBUG_BUFFER_SIZE          4096
#define DIV10(x)    (((int64_t)x*0x66666667L)>>34)
extern uint8_t debug_buffer[DEBUG_BUFFER_SIZE];

//DEBUG重载发数
 #define DEBUG(...) {\
 	int len = 0;\
 	len = snprintf((char *)debug_buffer,DEBUG_BUFFER_SIZE,__VA_ARGS__);\
 	if(len>0){\
 		HAL_UART_Transmit_IT(&DEBUG_UART_HANDLER, (uint8_t *)debug_buffer, len);\
 	}\
 }

//app相关结构体
typedef struct{
	uint8_t stopFlag;		//停止标志位
	float rc_X;				//摇杆x轴
	float rc_Y; 			//摇杆y轴
	uint8_t clspinFlag;		//顺时针旋转标志位
	uint8_t acspinFlag;		//逆时针旋转标志位
	uint8_t disableFlag;
	uint8_t stopSpinFlag; 	//停止旋转标志位
	uint8_t stopMoveFlag;	//停止平动标志位
	uint8_t controlModeFlag; //控制模式标志位（0：AUTO, 1: RC)
	uint8_t shootFlag;
	uint8_t selfCheckFlag;	//自检标志位
	uint8_t resetFlag;
	uint8_t passFlag;       //传球标志位
	uint8_t yoloeFlag; //视觉标框标志位
	uint8_t switchModeFlag;
	uint8_t dribbleFlag;
	uint8_t stillDribbleFlag;
	int32_t sneak_or_fool;	//突防动作标志数
	uint8_t defenceFlag;
	uint8_t attackFlag;
	uint8_t RorB;
	int32_t trickCnt;
	
	uint8_t zeroFlag;

}g_APPdata;

//当前位置通信结构体
typedef struct{
	float pps_x;
	float pps_y;
	float pps_yaw;
}g_RobotComData;

//电驱相关数据结构体
typedef struct{
	uint8_t _5v_state;		//小电状态
	uint8_t _24v_state;		//大电状态
	uint16_t voltage_adc;	//adc采样值
	float voltage_value;	//电池电压
	uint8_t lowPowFlag;		//低电量标志位
}g_power;


//机器人相关结构体
typedef struct
{
	uint8_t startGameFlag;		//开始标志位
	uint8_t runSucceedFlag;
	uint8_t selfCheckFlag;
	uint8_t onlyNeedStart;		//万事俱备，只欠东风
	uint8_t arm_reset_flag;   //机械臂复位
	uint8_t arm_reset_beep_flag;

	uint8_t canErrorFlag;		//CAN通讯断连标志位
	uint8_t get_pcErrorFlag;	//PC-MCU通讯错误标志位
	uint8_t mcuErrorFlag;		//主控错误标志位
	uint8_t pcErrorFlag;		//小电脑错误标志位
	uint8_t pcComCorrect;		//有正常控制标志位，5ms清零
	
	uint8_t getAppFlag;			//收到APP发数标志位
	uint8_t getPartnerMsgFlag;   //收到友方射频消息标志位

	uint8_t startCheckFlag;//大电开了3s后置1，随后才检测通信
	uint8_t restartSuccessFlag;//上层can重启
	uint8_t reset_pos_ok_flag;
	
	uint8_t periodCntMCU;
	uint8_t periodCntPC;

	g_APPdata app;
	g_power power;

	g_RobotComData selfComData;//自己信息，通信用
	g_RobotComData partnerComData;//伙伴信息，通信用
	
	int tx_can_cnt;
	int rx_can_cnt;

	uint8_t wheelColor;//记录当前轮子颜色
	
	uint8_t steeringWheelMode;//舵轮控制模式: 1 位控 2 速控
	uint8_t valveStatus;//气阀状态
	uint8_t pump_enable_flag;
	uint8_t changeModeFalseFlag;//切换失败标志位
	uint8_t rollFlag;//手上辊子启动标志位
	uint8_t read_reset_flag;
	uint8_t forceshoot_flag;
	
	uint8_t driverState[3];//机械臂标志位
//	uint8_t arm_reset_flag;//投球复位结束标志，若是1发给电驱重新启动复位程序
	int dataLen;
	
	float rollerSpeed1;
	float rollerSpeed2;
	
	float add_dis;
	float add_angle;
	
	uint8_t lightStripColor;//灯带颜色
}gRobot_t;

extern gRobot_t gRobot;

#define BEEP_ON (HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_SET))
#define BEEP_OFF (HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,GPIO_PIN_RESET))

/***************************************************************************/
void Start_Info(void);
void Get_App_Info(void);
void main_loop(void);
uint8_t Beep(uint32_t time);
void CheckBallIn(void);
void FlashColor(uint8_t on_or_off);
void ErrorCheck(void);
void FakeHeartBeat();

void dribblingInit(void);
void Dribble(void);

void Reset_Arm(void);
#endif
