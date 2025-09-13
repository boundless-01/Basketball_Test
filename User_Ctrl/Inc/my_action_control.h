#ifndef _MY_ACTION_CONTROL_H_
#define _MY_ACTION_CONTROL_H_

#include "my_motor_driver.h"
#include "my_chassis_driver.h"

/* 圆周率 */
#ifndef PI
    #define PI 3.14159265358979f
#endif

//上下层对应的can号
#define UP_FDCAN	(hfdcan3)
#define WHEEL_FDCAN (hfdcan1)

/*各电机ID宏定义*/
//舵轮电机
//CAN1
#define WHEEL1_ID		(1)
#define WHEEL2_ID		(2)
#define WHEEL3_ID		(3)
//CAN3

//机械臂
#define UPPER_ARM_ID 	 (1)
#define FOREARM_ID 	 (2)
#define WRIST_ID 	 (3)
#define TURNTABLE_ID 	(4)

//蹦蹦
#define LEG1_ID 	 (5)
#define LEG2_ID 	 (6)
#define LEG3_ID 	 (7)


#define ALLOW_LOST_TIME_PC (20)//允许丢包次数

#define DISABLE_MODE 0
#define VEL_MODE 1
#define POS_MODE 2
#define PVT_MODE 4
#define HB_MODE 3

#define ARM_REDUCTION_RATIO (63.f * 6.f / 22.f)

//电机期望数据结构体
typedef struct{
	float vel;
	float pos;
	int velPulse;
	int posPulse;
	
	/*!!以下仅PVT模式使用!!*/
	float tor;
	float pvt_kp;
	float pvt_kd;
	
	
	float Vel_kp;
	float Vel_ki;
	
	uint8_t reset_flag;
	uint8_t empty_distance_flag;
}motorTarget_t;

//电机，PC心跳包结构体
typedef struct{
	uint8_t PC;

	uint8_t wheel1;
	uint8_t wheel2;
	uint8_t wheel3;

	uint8_t turntable;
	uint8_t leg1;
	uint8_t leg2;
	uint8_t leg3;
	uint8_t upper_arm;
	uint8_t forearm;
	uint8_t wrist;
	
	uint8_t rc;
}Heart_t;

extern Heart_t HB,firstComFlag;

extern SteeringWheelMsg_t wheel1Msg,wheel2Msg,wheel3Msg;

extern driverMsg_t upperArmMsg, forearmMsg, wristMsg, turntableMsg;
 
extern uint16_t disableCnt;

//舵轮期望
extern motorTarget_t wheel1Tar;
extern motorTarget_t wheel2Tar;
extern motorTarget_t wheel3Tar;

//机械臂期望
extern motorTarget_t upperArmTar;
extern motorTarget_t forearmTar;
extern motorTarget_t wristTar;

//转盘期望
extern motorTarget_t turntableTar;

//蹦蹦期望
extern motorTarget_t leg1Tar;
extern motorTarget_t leg2Tar;
extern motorTarget_t leg3Tar;

//机械臂发给驱动器的期望
extern driver_control *upperArmCtrlTar;
extern driver_control *forearmCtrlTar;
extern driver_control *wristCtrlTar;

void ChassisControl(void);
void Arm_reset(void);
void ArmControl(void);
void LegControl(void);
void ValveControl(void);
void RollerControl(void);

void GetMotorMsg(void);
void CheckCanCom(void);
void CheckPCCom(void);
void ErrorStop(void);
void DisableAll(void);
void SpeedKpKiCtl(void);

extern uint8_t driver_state[3];

#endif
