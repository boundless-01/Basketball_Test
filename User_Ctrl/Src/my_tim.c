/**
  ******************************************************************************
  * @file    my_tim.c
  * @author  Geollay
  * @version
  * @date    2024/12/13
  * @brief   定时器
  ******************************************************************************
**/

#include "my_tim.h"
#include "main.h"
#include "my_action_control.h"
#include "my_main.h"

uint8_t timer_1ms_Flag = 0; //定时器1ms
uint8_t timer_5ms_Flag = 0; //定时器5ms
uint32_t beep_Cnt = 0;		// 蜂鸣器使用计数器

/**
 * @brief 1ms定时器回调函数
 * @param  
 * @return 
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//1ms定时器
	if (htim->Instance == htim3.Instance)
	{
		/*5ms一个控制周期*/
		timer_1ms_Flag = 1;
		static uint8_t _5ms_cnt = 0;

		//5ms定时器
		if (_5ms_cnt < 4) _5ms_cnt++;
		else 
		{
			timer_5ms_Flag = 1;
			_5ms_cnt = 0;
		}
		
		gRobot.periodCntMCU++;//记录主控跑了多少ms，用来看两次跑主程序之间的时间间隔是否正常
		
		if (beep_Cnt < 10 * 1000) beep_Cnt++; //蜂鸣器，最大值是十秒
		//gRobot.power._24v_state = 1;//因为没有电压检测，直接开
		if (gRobot.power._24v_state)//大电开完过3s再检测通信，power._24v_state是大电状态（在GetPowerState()中赋值的）
		{
			static int cnt = 0;
			if (cnt < 3000)
			{
				if (cnt == 2995 || cnt == 2990 || cnt == 2985 || cnt == 2980 || cnt == 2975)
					//开始检测前给舵轮发5次消息以判断是否连上,间隔5ms：避免总线拥堵，同时确保舵轮有足够时间回复
				{
					SteeringWheelCtrl(&WHEEL_FDCAN, WHEEL1_ID, 0.f, 0.f);
					SteeringWheelCtrl(&WHEEL_FDCAN, WHEEL2_ID, 0.f, 0.f);
					SteeringWheelCtrl(&WHEEL_FDCAN, WHEEL3_ID, 0.f, 0.f);
				}
				cnt++;
			}
			else if (cnt == 3000)//开始检测通信
			{
				gRobot.startCheckFlag = 1;//用于检测通信（即给心跳包赋值）

				cnt++;
			}
		}

		gRobot.periodCntMCU++;
	}
}

//启动定时器
void TIM_ENABLE(void)
{
	//TIM3
	__HAL_TIM_ENABLE(&htim3);
	__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);
	 HAL_TIM_Base_Start_IT(&htim3);

	//TIM1
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4); 
}
