/**
 ******************************************************************************
 * @file    my_main.c
 * @author  Geollay
 * @version
 * @date    2024/12/13
 * @brief   用户的main函数
 ******************************************************************************
 **/

#include "my_main.h"
#include "my_tim.h"
#include "my_action_control.h"
#include "my_communication.h"
#include "my_sensor.h"
#include "my_fdcan.h"

uint8_t debug_buffer[DEBUG_BUFFER_SIZE];
gRobot_t gRobot = {.steeringWheelMode = SPD_CURR_CTRL_MODE,
				   .wheelColor = 0x00             			};
uint8_t runOKFlag = 0; // 允许开始标志位
static char light_buffer[6];
volatile uint8_t uart8_tx_done = 1;
/**
 * @brief 用户程序主逻辑
 */

void main_loop(void)
{
	if (timer_5ms_Flag) // 5ms定时器
	{
		//CheckCanCom();
//		HB.upper_arm = HB.upper_arm < 200 ? HB.upper_arm + 1 : 200;
		
		timer_5ms_Flag = 0;
		
		static int tim_cnt = 0;
		static int setballtim_cnt = 0;                //捡球阶段计时器
		static int speed_up_cnt = 0;
		static int setoutpos_cnt = 0;
		static int speed_down_cnt = 0;
		static int mode = 0;
		static int setballOK = 0;                     //捡球成功标志位
		
		static float expPos = 0;
		static float expVel = 0;
		
		float tarPos = -120.f;                        //目标位置  -120.f
		float setPos = 13.f;                          //捡球位置  13.f
		float tarVel = 20.f;                          //目标速度  20.f
		float speedUpTime = 0.2f;                     //加速控制周期 0.2f
		float setoutposTime = 2.0f;                   //运动到捡球位置控制周期 2.0f
    float speedDownTime = 0.5f;		                //减速控制周期 0.5f
				
		tim_cnt++;
		
//		ReadPVT(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID);

		
		if (tim_cnt == 500)
		{
			BEEP_ON;
			R80_Set_Mode(1, MODE_POSITION);
			R80_Velocity_cmd(0.1, 0, 0, 0);
			R80_Save_Param(1);
		}
		else if (tim_cnt == 600)
		{
			BEEP_OFF;
		}
		else if (tim_cnt >= 600)
		{
//		DEBUG("%d, %f, %f, %f, %f, %f\r\n", mode, upperArmMsg.pos / 6.f / PI * 180.f, upperArmMsg.pos, expVel / 4096.f, upperArmMsg.vel / 2.f / PI, upperArmMsg.torque);
			DEBUG("%d, %f, %f, %d, %f\r\n", mode, expPos, motors[0].total_angle_deg / 6.f / PI * 180.f, setballtim_cnt, motors[0].current_a);
			if(setballOK == 0)
			{
				if(motors[0].total_angle_deg <= setPos)
				{
					mode = 1;
					
					setoutpos_cnt++;
				}
				if(motors[0].total_angle_deg - setPos >= -0.015f && motors[0].total_angle_deg - setPos <= 0.015f)
				{
					setballtim_cnt++;
				}
				if(setballtim_cnt == 1000)
				{
					
					BEEP_ON;
					/*
					
						操控夹球
					
					*/
				}
				else if(setballtim_cnt == 1200)
				{
					BEEP_OFF;
					R80_Set_Mode(1, MODE_VELOCITY);
					R80_Save_Param(1);
					setballOK = 1;
				}
			}
			if(setballOK == 1)
			{
				if(motors[0].total_angle_deg >= tarPos)
				{
					mode = 2;
					
					speed_up_cnt++;
				}
				else
				{
					mode = 3;
					
					speed_down_cnt++;
				}
			}
		
/*******************************机械臂操作************************************/
			if(mode == 1)      //捡球阶段
			{
				if(setoutpos_cnt <= setoutposTime / 0.005)
				{
					R80_Position_PID(1, 10.0f, 0.0f, 0.0f);
					expPos = 13.f / 360.f * sin(PI * setoutpos_cnt / 400 / 2);
					R80_Position_cmd(expPos, 0, 0, 0);
					R80_Save_Param(1);
				}
			}
			if(mode == 2)      //加速阶段
			{
				if (speed_up_cnt <= speedUpTime / 0.005)
				{
					if(speed_up_cnt <= 0.08f / 0.005)
					{
						R80_Velocity_PID(1, 1.0f, 1.0f, 0.0f);
						expVel = -(tarVel / speedUpTime) * speed_up_cnt * 0.005 + (tarVel / PI / 2.f) * sin(PI * 2.f * speed_up_cnt * 0.005 / speedUpTime);
						R80_Velocity_cmd(expVel, 0, 0, 0);
						R80_Save_Param(1);
					}
					else
					{
					}
						
						R80_Velocity_PID(1, 1.0f, 1.0f, 0.0f);
						expVel = -(tarVel / speedUpTime) * speed_up_cnt * 0.005 + (tarVel / PI / 2.f) * sin(PI * 2.f * speed_up_cnt * 0.005 / speedUpTime);
						R80_Velocity_cmd(expVel, 0, 0, 0);
						R80_Save_Param(1);
				}
				else
				{
					expVel = -tarVel;
					R80_Velocity_cmd(expVel, 0, 0, 0);
					R80_Save_Param(1);
				}
			}
			if(mode == 3)      //减速阶段
			{
				if (speed_down_cnt <= speedDownTime / 0.005)
				{
					R80_Velocity_PID(1, 1.0f, 1.0f, 0.0f);
					expVel = -tarVel + (tarVel / speedDownTime) * speed_down_cnt * 0.005 + (tarVel / PI / 2.f) * sin(PI * 2.f * speed_down_cnt * 0.005 / speedDownTime);
					R80_Velocity_cmd(expVel, 0, 0, 0);
					R80_Save_Param(1);
				}
				else
				{
					R80_Velocity_PID(1, 1.0f, 1.0f, 0.0f);
					R80_Velocity_cmd(0, 0, 0, 0);
					R80_Save_Param(1);
				}
			}
		}
	}
}

/**
 * @brief 启动指示
 */
void Start_Info(void)
{
	static uint8_t i = 0;
	if (i < 2)
	{
		if (Beep(100) == 1)
			i++; // 定时器响两声(每次100ms)表示上位机运行正常
	}
	else
	{
		runOKFlag = 1;
	}
}

/**
 * @brief APP接收指示
 */
void Get_App_Info(void)
{
	if (gRobot.getAppFlag) // APP收数标志位
	{
		static uint8_t cnt = 0;
		if (cnt < 100)
		{
			if (timer_1ms_Flag) // 1ms定时器标志位
			{
				timer_1ms_Flag = 0;
				cnt++;
			}
			// Beep(150);
		}
		else
		{
			gRobot.getAppFlag = 0;
			cnt = 0;
		}
	}
}

uint8_t Beep(uint32_t time)
{
	static uint8_t flag = 0;
	static uint32_t lastTime = 0;

	if (lastTime != time)
		flag = 0; // 如果跟上次不一样，重新计时

	lastTime = time;

	if (!flag) // 判断是否为第一次进入
	{
		flag = 1;
		beep_Cnt = 0; // 清零计时器(每一毫秒自加一)
	}

	if (beep_Cnt < time)
		BEEP_ON;
	else if (beep_Cnt < time * 2)
		BEEP_OFF;
	else
	{
		BEEP_OFF;
		flag = 0; // 清零等待下一次进入
		return 1; // 响完一轮返回1
	}
	return 0;
}