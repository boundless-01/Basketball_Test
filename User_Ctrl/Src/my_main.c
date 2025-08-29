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
		timer_5ms_Flag = 0;
		
		static int tim_cnt = 0;
		static int speed_up_cnt = 0;
		static int speed_down_cnt = 0;
		static int mode = 0;
		
		static float expPos = 0;
		static float expVel = 0;
		
		float tarPos = 120.f / 180.f * PI;            //目标位置  120.f / 360.f * 4096.f * 6.f
		float tarVel = 20.f * 4096;                   //目标速度
		float speedUpTime = 0.5f;                     //加速控制周期 0.2f 
    float speedDownTime = 0.5f;		                //减速控制周期 0.5f 
		uint32_t p_pos = 1.5 * 60.f * 1000.f;         //位置环KP 1.5 * 60.f * 1000.f
		uint32_t d_pos = 0.2 * 0.1f * 1000.f;         //位置环KD 0.2 * 0.1f * 1000.f

    uint32_t p_speed =  3.5 * 1000.f;             //速度环KP 1.5*1000.f
		uint32_t i_speed =  3.0 * 1000.f;             //速度环KI 3.0*1000.f
		
		tim_cnt++;
		
		ReadPVT(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID);
		
		if (tim_cnt == 1500)
		{
			BEEP_ON;
			ResetControlMode(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, VEL_MODE);
			SetVelLimit(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, 30 * 4096);
		}
		else if (tim_cnt == 1600)
		{
			BEEP_OFF;
		}
		else if (tim_cnt >= 1600)
		{	
			DEBUG("%d, %f, %f, %f, %f, %f\r\n", mode, upperArmMsg.pos / 6.f / PI * 180.f, upperArmMsg.pos, expVel / 4096.f, upperArmMsg.vel / 2.f / PI, upperArmMsg.torque);
			
			if (upperArmMsg.pos / 6.f <= tarPos)
			{
				mode = 1;
				
				speed_up_cnt++;
				
				if (speed_up_cnt <= speedUpTime / 0.005)
				{
					if(speed_up_cnt <= 0.2f / 0.005)
					{
						p_speed = 2.55 * 1000.f;
						
					}
					else
					{
						p_speed = 6.0 * 1000.f;
					}
						SetSpeedKP(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, p_speed);  //速度的KP
						SetSpeedKI(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, i_speed);  //速度的KI
						
						expVel = (tarVel / speedUpTime) * speed_up_cnt * 0.005 - (tarVel / PI / 2.f) * sin(PI * 2.f * speed_up_cnt * 0.005 / speedUpTime);
						
						VelCtrl(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, (int)expVel);
				}
				else
				{
					expVel = tarVel;
					
					VelCtrl(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, (int)expVel);
				}
			}
			else
			{
				mode = 2;
				
				speed_down_cnt++;
				
				if (speed_down_cnt <= speedDownTime / 0.005)
				{
					p_speed = 2.8 * 1000.f;
					SetSpeedKP(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, p_speed);  //速度的KP
					SetSpeedKI(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, i_speed);  //速度的KI
					
					expVel = tarVel - (tarVel / speedDownTime) * speed_down_cnt * 0.005 - (tarVel / PI / 2.f) * sin(PI * 2.f * speed_down_cnt * 0.005 / speedDownTime);
					
					VelCtrl(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, (int)expVel);
				}
				else
				{
					VelCtrl(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, 0);
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