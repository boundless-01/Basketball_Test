#ifndef __MY_CHASSIS_DRIVER_H__
#define __MY_CHASSIS_DRIVER_H__

#include "my_motor_driver.h"


/*驱动器发送ID基地址*/
#define DRIVER_CLIENT_BASE_ID	0x280

/*驱动器接收ID基地址*/
#define DRIVER_SERVER_BASE_ID	0x300

/*常发标识符*/
#define IDENTIFIER_POS_SPD_CTRL		0x01
#define IDENTIFIER_POS_SPD_READ		0x02

/*设置标识符*/
#define IDENTIFIER_PID_SET			0x03
#define IDENTIFIER_SW_SET				0x04
#define IDENTIFIER_COLOR_SET 		0x08

/*错误标识符*/
#define IDENTIFIER_HARD_FAULT		0xFF

/*回复标识符*/
#define IDENTIFIER_POS_SPD_READ_BCK		0x0F

/*舵轮控制模式*/
#define IDENTIFIER_DRIVER_MODE           	0x0A
#define POS_SPD_CURR_CTRL_MODE  					0x01
#define SPD_CURR_CTRL_MODE      					0x02

/*限流挡位设置*/
typedef enum{
				TEST_MODE = 0x01,
				NORMAL_MODE = 0x02,
				EXTREME_MODE = 0x03,
				ORIGINAL_L= 0x00, 
			} CurrQLim_Mode;	

/*轮子加速度设置*/
typedef enum{
				HALFACC = 0x01,
				FULLACC = 0x02,
				LITACC = 0x03,
			} ACC_MODE;				

typedef enum{
				ENABLE_ALL = 0x00,
				DISABLE_ALL = 0x01,
			} DriverState_Mode;

typedef enum{
				COLOR1 = 0x00,//无颜色		
				COLOR2 = 0x01,//橙色			小电脑的问题
				COLOR3 = 0x02,//粉偏红  	红场
				COLOR4 = 0x03,//黄绿			canERROR
				COLOR5 = 0x04,//深绿  		可以射球
				COLOR6 = 0x05,//淡蓝绿		dis too big
				COLOR7 = 0x06,//蓝				蓝场
				COLOR8 = 0x07,//淡蓝紫色 	用于指示闪灯
				COLOR9 = 0x08,//栗色			
				COLOR10 = 0x09,//橄榄色
				COLOR11 = 0x010,//紫色
				COLOR12 = 0x011,//蓝绿色
			} RGB_Cmd;

/*主控接收舵轮消息内容结构体*/
typedef struct
{
	float vel;
	float pos;	
}SteeringWheelMsg_t;


void GetSteeringWheelMsg(SteeringWheelMsg_t *driverMsg, uint8_t *data8);
void Saturation_int(int *value, int upLimit, int downLimit);
void SteeringWheelCtrl(FDCAN_HandleTypeDef* hfdcan, uint8_t DriverNum, float velAim, float PosAim);
void SetSteeringWheelMode(FDCAN_HandleTypeDef* hfdcan, uint8_t DriverNum, uint8_t DriverMode);
void Saturation_float(float *value, float upLimit, float downLimit);
void DriverStateChassis(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum, ACC_MODE acc_mode,\
				RGB_Cmd rgb_cmd, CurrQLim_Mode ILIMmode, DriverState_Mode wheelstate, DriverState_Mode turnstate);
void ChassisChangeColor(FDCAN_HandleTypeDef* hfdcan, uint8_t DriverNum, RGB_Cmd rgb_cmd);
void Saturation_uint32(uint32_t *value, uint32_t upLimit, uint32_t downLimit);
void ReadCurrG(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum);
void Integral_Cleaded(FDCAN_HandleTypeDef* hfdcan, CommandMode CMDmode, uint8_t DriverNum);


#endif

