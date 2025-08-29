/**
 ******************************************************************************
 * @file		my_movements_control.c
 * @author		Geollay
 * @version
 * @date		2024/12/13
 * @brief		电机与动作控制与读取信息相关
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "my_action_control.h"
#include "my_main.h"
#include "tim.h"

SteeringWheelMsg_t wheel1Msg, wheel2Msg, wheel3Msg;
driverMsg_t upperArmMsg, forearmMsg, wristMsg, turntableMsg;

// 机械臂发给驱动器的期望
driver_control *upperArmCtrlTar, *forearmCtrlTar, *wristCtrlTar;

Heart_t HB, firstComFlag; // firsComFlag由第一次接收到驱动器赋值

// 舵轮期望
motorTarget_t wheel1Tar;
motorTarget_t wheel2Tar;
motorTarget_t wheel3Tar;

// 机械臂期望
motorTarget_t upperArmTar;
motorTarget_t forearmTar;
motorTarget_t wristTar;

// 蹦蹦期望
motorTarget_t leg1Tar;
motorTarget_t leg2Tar;
motorTarget_t leg3Tar;

/* 预设控制模式及参数 */
uint8_t driver_state[3] = {POS_MODE, POS_MODE, POS_MODE};

// 舵轮控制
void ChassisControl(void)
{
	// 如果app要停车
	//	if (gRobot.app.stopFlag)
	//	{
	//		wheel1Tar.vel = 0.f;
	//		wheel2Tar.vel = 0.f;
	//		wheel3Tar.vel = 0.f;
	//	}
	// 
ChassisChangeColor(&WHEEL_FDCAN, WHEEL1_ID, gRobot.wheelColor);
ChassisChangeColor(&WHEEL_FDCAN, WHEEL2_ID, gRobot.wheelColor);
ChassisChangeColor(&WHEEL_FDCAN, WHEEL3_ID, gRobot.wheelColor);

	// 切环判断
	static uint8_t steeringWheelMode = SPD_CURR_CTRL_MODE;
	if (gRobot.steeringWheelMode != steeringWheelMode) // 舵轮模式由小电脑发来
	{	
		if(gRobot.steeringWheelMode != 0){
			if(steeringWheelMode == 0)
			{
				DriverStateChassis(&WHEEL_FDCAN, PTP_MODE, WHEEL1_ID, LITACC, gRobot.wheelColor, NORMAL_MODE, ENABLE_ALL, ENABLE_ALL);
				DriverStateChassis(&WHEEL_FDCAN, PTP_MODE, WHEEL2_ID, LITACC, gRobot.wheelColor, NORMAL_MODE, ENABLE_ALL, ENABLE_ALL);
				DriverStateChassis(&WHEEL_FDCAN, PTP_MODE, WHEEL3_ID, LITACC, gRobot.wheelColor, NORMAL_MODE, ENABLE_ALL, ENABLE_ALL);
			}
			SetSteeringWheelMode(&WHEEL_FDCAN, WHEEL1_ID, gRobot.steeringWheelMode); // 设置舵轮模式，并用can通信发给舵轮
			SetSteeringWheelMode(&WHEEL_FDCAN, WHEEL2_ID, gRobot.steeringWheelMode);
			SetSteeringWheelMode(&WHEEL_FDCAN, WHEEL3_ID, gRobot.steeringWheelMode);
		}else{
			DriverStateChassis(&WHEEL_FDCAN, PTP_MODE, WHEEL1_ID, LITACC, gRobot.wheelColor, NORMAL_MODE, DISABLE_ALL, DISABLE_ALL);
			DriverStateChassis(&WHEEL_FDCAN, PTP_MODE, WHEEL2_ID, LITACC, gRobot.wheelColor, NORMAL_MODE, DISABLE_ALL, DISABLE_ALL);
			DriverStateChassis(&WHEEL_FDCAN, PTP_MODE, WHEEL3_ID, LITACC, gRobot.wheelColor, NORMAL_MODE, DISABLE_ALL, DISABLE_ALL);
		}
		steeringWheelMode = gRobot.steeringWheelMode;
		return;
	}
	//	static int flag = 0;
	//	if(!flag){
	//		flag = 1;
	//		SetSteeringWheelMode(&WHEEL_FDCAN, WHEEL2_ID, 1);
	//	}
	// DEBUG("%d %d\r\n",gRobot.steeringWheelMode,steeringWheelMode);
	// if(cnt >= 200)wheel1Tar.vel = 0.15;
	// 三舵轮控制
	if (HB.wheel1 <= 20 && HB.wheel2 <= 20 && HB.wheel3 <= 20)
	{
		// 三舵轮控制（设置位置和速度，这里如果位置环：vel指轮向，pos指航向）
		SteeringWheelCtrl(&WHEEL_FDCAN, WHEEL1_ID, -wheel1Tar.vel, -wheel1Tar.pos);
		SteeringWheelCtrl(&WHEEL_FDCAN, WHEEL2_ID, -wheel2Tar.vel, -wheel2Tar.pos);
		SteeringWheelCtrl(&WHEEL_FDCAN, WHEEL3_ID, +wheel3Tar.vel, -wheel3Tar.pos);
	}
}

void SpeedKpKiCtl(){
		static float lastkp = 0.f;
	
		if(lastkp != forearmTar.Vel_kp){
			SetSpeedKP(&UP_FDCAN,PTP_MODE,UPPER_ARM_ID,(upperArmTar.Vel_kp*1000));
			SetSpeedKP(&UP_FDCAN,PTP_MODE,FOREARM_ID,(forearmTar.Vel_kp*1000));
			SetSpeedKP(&UP_FDCAN,PTP_MODE,WRIST_ID,(wristTar.Vel_kp*1000));
			SetSpeedKI(&UP_FDCAN,PTP_MODE,UPPER_ARM_ID,(upperArmTar.Vel_ki*1000));
			SetSpeedKI(&UP_FDCAN,PTP_MODE,FOREARM_ID,(forearmTar.Vel_ki*1000));
			SetSpeedKI(&UP_FDCAN,PTP_MODE,WRIST_ID,(wristTar.Vel_ki*1000));
		}
		
		lastkp = forearmTar.Vel_kp;
}
void Data_Transport(driver_control *motor, float pos, float vel, float tor , float kp, float kd) // 用于向PVT结构体中输入期望
{
	motor->k.p_des = pos * 1000;
	motor->k.v_des = vel * 1000;
	motor->k.t_ff = tor * 10000;
	motor->k.kp = kp * 1000;
	motor->k.kd = kd * 1000;
}

// 机械臂复位函数
void Arm_reset()
{
	// 判断是否给驱动器发复位
	if (upperArmTar.reset_flag)
	{
		upperArmMsg.reset_pos_ok_flag = 0;
		ResetFlag(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID);
		driver_state[0] = POS_MODE;
	}
	if (forearmTar.reset_flag)
	{
		forearmMsg.reset_pos_ok_flag = 0;
		ResetFlag(&UP_FDCAN, PTP_MODE, FOREARM_ID);
		driver_state[1] = POS_MODE;
	}
	if (wristTar.reset_flag)
	{
		wristMsg.reset_pos_ok_flag = 0;
		ResetFlag(&UP_FDCAN, PTP_MODE, WRIST_ID);
		driver_state[2] = POS_MODE;
	}

	// 询问驱动器是否复位成功
	if ((!upperArmTar.reset_flag) && (!upperArmTar.empty_distance_flag))
	{
		IfResetOkFlag(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID);
	}
	if ((!forearmTar.reset_flag) && (!forearmTar.empty_distance_flag))
	{
		IfResetOkFlag(&UP_FDCAN, PTP_MODE, FOREARM_ID);
	}
	if ((!wristTar.reset_flag) && (!wristTar.empty_distance_flag))
	{
		IfResetOkFlag(&UP_FDCAN, PTP_MODE, WRIST_ID);
	}

	// 给驱动器发消空程
	if (upperArmTar.empty_distance_flag)
	{
		upperArmMsg.reset_pos_ok_flag = 0;
		EmptyDistanceFlag(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID);
	}
	if (forearmTar.empty_distance_flag)
	{
		forearmMsg.reset_pos_ok_flag = 0;
		EmptyDistanceFlag(&UP_FDCAN, PTP_MODE, FOREARM_ID);
	}
	if (wristTar.empty_distance_flag)
	{
		wristMsg.reset_pos_ok_flag = 0;
		EmptyDistanceFlag(&UP_FDCAN, PTP_MODE, WRIST_ID);
		gRobot.app.resetFlag = 0;
	}
}

void ArmControl(void)
{
	// 控制模式切换
	if (driver_state[0] != gRobot.driverState[0]) // 收到的小电脑的数据里会改变gRobot.driverState
	{
		if (gRobot.driverState[0] == DISABLE_MODE)
		{
			DriverState(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, DISABLE); // 禁用驱动器
		}
		else
		{
			DriverState(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, ENABLE); // 启用驱动器
			switch (gRobot.driverState[0])
			{
			case VEL_MODE: // 速度模式
			{
				ResetControlMode(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, VEL_MODE);
				break;
			}
			case POS_MODE: // 位置模式
			{
				ResetControlMode(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, POS_MODE);
				break;
			}
			case PVT_MODE: // PVT 模式
			{
				//					DEBUG("SET PVT!!!\r\n");
				ResetControlMode(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, PVT_MODE);
				break;
			}
			case HB_MODE: // 心跳恢复模式
				// 断了重连
				{
					if (driver_state[0] != DISABLE_MODE)
						ResetControlMode(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, driver_state[0]);
					break;
				}
			}
		}
		driver_state[0] = gRobot.driverState[0]; // 更新当前模式
	}

	if (driver_state[1] != gRobot.driverState[1]) // 同上
	{
		if (gRobot.driverState[1] == DISABLE_MODE)
		{
			DriverState(&UP_FDCAN, PTP_MODE, FOREARM_ID, DISABLE);
		}
		else
		{
			DriverState(&UP_FDCAN, PTP_MODE, FOREARM_ID, ENABLE);
			switch (gRobot.driverState[1])
			{
			case VEL_MODE:
			{
				ResetControlMode(&UP_FDCAN, PTP_MODE, FOREARM_ID, VEL_MODE);
				break;
			}
			case POS_MODE:
			{
				ResetControlMode(&UP_FDCAN, PTP_MODE, FOREARM_ID, POS_MODE);
				break;
			}
			case PVT_MODE:
			{
				ResetControlMode(&UP_FDCAN, PTP_MODE, FOREARM_ID, PVT_MODE);
				break;
			}
			case HB_MODE:
				// 断了重连
				{
					if (driver_state[1] != DISABLE_MODE)
						ResetControlMode(&UP_FDCAN, PTP_MODE, FOREARM_ID, driver_state[1]);
					break;
				}
			}
		}
		driver_state[1] = gRobot.driverState[1];
	}

	if (driver_state[2] != gRobot.driverState[2]) // 同上
	{
		if (gRobot.driverState[2] == DISABLE_MODE)
		{
			DriverState(&UP_FDCAN, PTP_MODE, WRIST_ID, DISABLE);
		}
		else
		{
			DriverState(&UP_FDCAN, PTP_MODE, WRIST_ID, ENABLE);
			switch (gRobot.driverState[2])
			{
			case VEL_MODE:
			{
				ResetControlMode(&UP_FDCAN, PTP_MODE, WRIST_ID, VEL_MODE);
				break;
			}
			case POS_MODE:
			{
				ResetControlMode(&UP_FDCAN, PTP_MODE, WRIST_ID, POS_MODE);
				break;
			}
			case PVT_MODE:
			{
				ResetControlMode(&UP_FDCAN, PTP_MODE, WRIST_ID, PVT_MODE);
				break;
			}
			case HB_MODE:
				// 断了重连
				{
					if (driver_state[2] != DISABLE_MODE)
						ResetControlMode(&UP_FDCAN, PTP_MODE, WRIST_ID, driver_state[2]);
					break;
				}
			}
		}
		driver_state[2] = gRobot.driverState[2];
	}

	// 臂电机控制
	switch (driver_state[0])
	{
	case VEL_MODE:
	{
		break;
	}
	case POS_MODE:
	{
		// 限速
		SetVelLimit(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, upperArmTar.posPulse);
		// 控制
		if (!upperArmTar.reset_flag)
			PosCtrl(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, (int32_t)(upperArmTar.pos / 2 / PI * 4096));
		break;
	}
	case PVT_MODE:
	{
		//			DEBUG("PVT CTRL!!! %f %f %f %f %f\r\n", upperArmTar.pos, upperArmTar.vel, upperArmTar.tor, upperArmTar.pvt_kp, upperArmTar.pvt_kd);
		Data_Transport(upperArmCtrlTar, upperArmTar.pos, upperArmTar.vel, upperArmTar.tor, upperArmTar.pvt_kp, upperArmTar.pvt_kd);
		if (!upperArmTar.reset_flag)
			PVTCtrl(&UP_FDCAN, UPPER_ARM_ID, upperArmCtrlTar);
		break;
	}
	}

	switch (driver_state[1])
	{
	case VEL_MODE:
	{
		SetSpeedKP(&UP_FDCAN,PTP_MODE,FOREARM_ID,(4.0*1000));
		VelCtrl(&UP_FDCAN, PTP_MODE, FOREARM_ID, (int32_t)(forearmTar.vel * 4096));
		break;
	}
	case POS_MODE:
	{
		// 限速
		SetVelLimit(&UP_FDCAN, PTP_MODE, FOREARM_ID, forearmTar.posPulse);
		// 控制
		if (!forearmTar.reset_flag)
			PosCtrl(&UP_FDCAN, PTP_MODE, FOREARM_ID, (int32_t)(forearmTar.pos / 2 / PI * 4096));
		break;
	}
	case PVT_MODE:
	{
		Data_Transport(forearmCtrlTar, forearmTar.pos, forearmTar.vel, forearmTar.tor, forearmTar.pvt_kp, forearmTar.pvt_kd);
		if (!forearmTar.reset_flag)
			PVTCtrl(&UP_FDCAN, FOREARM_ID, forearmCtrlTar);
		break;
	}
	}

	switch (driver_state[2])
	{
	case VEL_MODE:
	{
		break;
	}
	case POS_MODE:
	{
		// 限速
		SetVelLimit(&UP_FDCAN, PTP_MODE, WRIST_ID, wristTar.posPulse);
		// 控制
		if (!wristTar.reset_flag)
			PosCtrl(&UP_FDCAN, PTP_MODE, WRIST_ID, (int32_t)(wristTar.pos / 2 / PI * 4096));
		break;
	}
	case PVT_MODE:
	{
		Data_Transport(wristCtrlTar, wristTar.pos, wristTar.vel, wristTar.tor, wristTar.pvt_kp, wristTar.pvt_kd);
		if (!wristTar.reset_flag)
			PVTCtrl(&UP_FDCAN, WRIST_ID, wristCtrlTar);
		break;
	}
	}
}

void ValveControl(void)
{
	if (gRobot.valveStatus == 0) // 也是小电脑发来的
	{
		HAL_GPIO_WritePin(VALVE1_GPIO_Port, VALVE1_Pin, GPIO_PIN_RESET); // 气阀低电平是关
		HAL_GPIO_WritePin(VALVE2_GPIO_Port, VALVE2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(VALVE1_GPIO_Port, VALVE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(VALVE2_GPIO_Port, VALVE2_Pin, GPIO_PIN_SET);
	}
	if (gRobot.pump_enable_flag == 0) // 也是小电脑发来的
	{
		HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_SET);
	}
}

void RollerControl(void)
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, gRobot.rollerSpeed1); // 速度油门
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, gRobot.rollerSpeed2); // 速度油门
}

// 读取电机位置，速度信息
void GetMotorMsg(void)
{
	switch (gRobot.driverState[0])
	{
	case PVT_MODE:
	{
		break;
	}
	default:
	{
		ReadPVT(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID);
		break;
	}
	}
	switch (gRobot.driverState[1])
	{
	case PVT_MODE:
	{
		break;
	}
	default:
	{
		ReadPVT(&UP_FDCAN, PTP_MODE, FOREARM_ID);
		break;
	}
	}
	switch (gRobot.driverState[2])
	{
	case PVT_MODE:
	{
		break;
	}
	default:
	{
		ReadPVT(&UP_FDCAN, PTP_MODE, WRIST_ID);
		break;
	}
	}
}

//TODO：测试有问题
//void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
//{
//	static uint16_t errTimes1 = 0;
//	static uint16_t errTimes2 = 0;
//	if(hfdcan == &hfdcan1)
//	{
//		//SendBuf();
//		if(errTimes1++ <= 100)
//		{
//			errTimes1++;
//			
//			HAL_FDCAN_DeInit(&hfdcan1);
//			HAL_FDCAN_Init(&hfdcan1);
//			
//			if (FDCAN1->IR & FDCAN_IR_RF0N) // 检查 FIFO0 新消息标志
//			{
//				FDCAN1->IR = FDCAN_IR_RF0N; // 清除 FIFO0 新消息标志
//			}
//			
//			//__HAL_FDCAN_CLEAR_FLAG(&hfdcan1, CAN_FLAG_FOV0);
//			
//			HAL_FDCAN_ActivateNotification(
//				&hfdcan1,                                  // FDCAN 句柄
//				FDCAN_IT_RX_FIFO0_NEW_MESSAGE,             // 中断类型
//				FDCAN_IT_RX_FIFO0_NEW_MESSAGE     // 回调ID（新增参数）
//			);
//			
//			HAL_FDCAN_Start(&hfdcan1);
//			HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);  // 确保NVIC中断已开启
//		}
//	}else if(hfdcan == &hfdcan3)
//	{
//		//SendBuf();
//		if(errTimes2++ <= 100)
//		{
//			errTimes2++;
//			
//			HAL_FDCAN_DeInit(&hfdcan3);
//			HAL_FDCAN_Init(&hfdcan3);
//			
//			if (FDCAN3->IR & FDCAN_IR_RF0N) // 检查 FIFO0 新消息标志
//			{
//				FDCAN3->IR = FDCAN_IR_RF0N; // 清除 FIFO0 新消息标志
//			}
//			
//			//__HAL_FDCAN_CLEAR_FLAG(&hfdcan3, CAN_FLAG_FOV1);
//			
//			HAL_FDCAN_ActivateNotification(
//				&hfdcan3,                                  // FDCAN 句柄
//				FDCAN_IT_RX_FIFO0_NEW_MESSAGE,             // 中断类型
//				FDCAN_IT_RX_FIFO0_NEW_MESSAGE     // 回调ID（新增参数）
//			);
//			
//			HAL_FDCAN_Start(&hfdcan3);
//			HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);  // 确保NVIC中断已开启
//		}
//	}
//}

// 电机心跳包自增,检测通信是否正常
void CheckCanCom(void)
{
#define ALLOW_LOST_TIME (20)		   // 允许丢包周期
	static uint8_t restartCanFlag = 0; // CAN重启标志位

	// 心跳包每周期自增，设置上限200防止溢出（flag是从can通信发来的）
	if (firstComFlag.wheel1)
		HB.wheel1 = HB.wheel1 < 200 ? HB.wheel1 + 1 : 200;
	if (firstComFlag.wheel2)
		HB.wheel2 = HB.wheel2 < 200 ? HB.wheel2 + 1 : 200;
	if (firstComFlag.wheel3)
		HB.wheel3 = HB.wheel3 < 200 ? HB.wheel3 + 1 : 200;
	if (firstComFlag.turntable)
		HB.turntable = HB.turntable < 200 ? HB.turntable + 1 : 200;
	if (firstComFlag.upper_arm)
		HB.upper_arm = HB.upper_arm < 200 ? HB.upper_arm + 1 : 200;
	if (firstComFlag.forearm)
		HB.forearm = HB.forearm < 200 ? HB.forearm + 1 : 200;
	if (firstComFlag.wrist)
		HB.wrist = HB.wrist < 200 ? HB.wrist + 1 : 200;
	HB.rc = HB.rc < 50 ? HB.rc + 1 : 50;
	if (HB.rc >= 50)
	{
		gRobot.app.rc_X = 0;
		gRobot.app.rc_Y = 0;
	}

	// TODO:加上其他电机心跳包
	// HB.arm...
	// HB.leg...

	if ((HB.wheel1 >= 5 || HB.wheel2 >= 5 || HB.PC >= 5 ||
		 HB.wheel3 >= 5) &&
		!gRobot.pcComCorrect) // gRobot.pcComCorrect在my_main中，USBD_OK之后赋1
	{
		ChassisControl(); // 给驱动器发指令抢救
	}

	if ((HB.upper_arm >= 5 || HB.forearm >= 5 || HB.wrist >= 5 ||
		 HB.PC >= 5 || HB.turntable >= 5) &&
		!gRobot.pcComCorrect)
	{
		ArmControl(); // 给驱动器发指令抢救
	}
	gRobot.startCheckFlag = 1;
	if (gRobot.startCheckFlag)
	{
		if (HB.wheel1 >= ALLOW_LOST_TIME || HB.wheel2 >= ALLOW_LOST_TIME || HB.PC >= ALLOW_LOST_TIME ||
			HB.wheel3 >= ALLOW_LOST_TIME)
		{
			gRobot.canErrorFlag = 1; // can通信异常标志位
		}
		else if (HB.wheel1 < ALLOW_LOST_TIME && HB.wheel2 < ALLOW_LOST_TIME && HB.PC < ALLOW_LOST_TIME &&
				 HB.wheel3 < ALLOW_LOST_TIME)
		{
			gRobot.canErrorFlag = 0;
			restartCanFlag = 0;
		}

		// 重启FDCAN
		// DEBUG("%d %d %d %d %d %d\r\n",restartCanFlag, gRobot.canErrorFlag,HB.wheel1,HB.wheel2,HB.wheel3,HB.PC);
		if (restartCanFlag <= 0 && gRobot.canErrorFlag)
		{
			gRobot.restartSuccessFlag = gRobot.restartSuccessFlag ^ 1;
			restartCanFlag ++;
			//WHEELFDCAN
			if(HAL_FDCAN_Stop(&WHEEL_FDCAN) != HAL_OK){
				restartCanFlag --;
				return;
			}
			FDCAN_FilterTypeDef FDCAN_RXFilter;
	  	FDCAN_RXFilter.IdType = FDCAN_STANDARD_ID;
			FDCAN_RXFilter.FilterType = FDCAN_FILTER_MASK;
			FDCAN_RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP;
			FDCAN_RXFilter.FilterID1 = 0x00000000;
			FDCAN_RXFilter.FilterID2 = 0x00000000;
			FDCAN_RXFilter.FilterIndex = 0;
			HAL_FDCAN_ConfigFilter(&WHEEL_FDCAN, &FDCAN_RXFilter);

			HAL_FDCAN_ConfigGlobalFilter(&WHEEL_FDCAN, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
			HAL_FDCAN_Start(&WHEEL_FDCAN);
			HAL_FDCAN_ActivateNotification(&WHEEL_FDCAN, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
		}
		static uint8_t arm_can_restart_flag= 0;
		if (HB.upper_arm >= ALLOW_LOST_TIME || HB.forearm >= ALLOW_LOST_TIME ||
			(HB.wrist >= ALLOW_LOST_TIME && !arm_can_restart_flag)){
				arm_can_restart_flag = 1;
			}
		if(arm_can_restart_flag == 1){
				arm_can_restart_flag ++;
				HAL_FDCAN_Stop(&UP_FDCAN);
				FDCAN_FilterTypeDef FDCAN_RXFilter;
				FDCAN_RXFilter.IdType = FDCAN_STANDARD_ID;
				FDCAN_RXFilter.FilterType = FDCAN_FILTER_MASK;
				FDCAN_RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP;
				FDCAN_RXFilter.FilterID1 = 0x00000000;
				FDCAN_RXFilter.FilterID2 = 0x00000000;
				FDCAN_RXFilter.FilterIndex = 0;
				HAL_FDCAN_ConfigFilter(&UP_FDCAN, &FDCAN_RXFilter);
	
				HAL_FDCAN_ConfigGlobalFilter(&UP_FDCAN, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
				HAL_FDCAN_Start(&UP_FDCAN);
				HAL_FDCAN_ActivateNotification(&UP_FDCAN, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
		}
		if(HB.upper_arm < ALLOW_LOST_TIME && HB.forearm < ALLOW_LOST_TIME &&
				 HB.wrist < ALLOW_LOST_TIME){
						arm_can_restart_flag = 0;
				 }
	}
}

void CheckPCCom(void)
{
	// 心跳包每周期自增，设置上限200防止溢出（runSucceedFlag在my_main.c()里一开始就置 1 了）
	if (gRobot.runSucceedFlag)
		HB.PC = HB.PC < 200 ? HB.PC + 1 : 200;

	if (gRobot.startCheckFlag)
	{
		if (HB.PC > ALLOW_LOST_TIME_PC)
		{
			gRobot.get_pcErrorFlag = 1;
			ErrorStop();
		}
		else
		{
			gRobot.get_pcErrorFlag = 0;
		}
	}
}
uint16_t disableCnt = 0;

// 出问题停车（舵轮和臂的速度都改为0）
void ErrorStop(void)
{
	SteeringWheelCtrl(&WHEEL_FDCAN, WHEEL1_ID, 0.f, -wheel1Tar.pos);
	SteeringWheelCtrl(&WHEEL_FDCAN, WHEEL2_ID, 0.f, -wheel2Tar.pos);
	SteeringWheelCtrl(&WHEEL_FDCAN, WHEEL3_ID, 0.f, -wheel3Tar.pos);
	wheel1Tar.vel = 0.f;
	wheel2Tar.vel = 0.f;
	wheel3Tar.vel = 0.f;
}

void DisableAll(void) // 给舵轮和机械臂都失能
{
	DriverStateChassis(&WHEEL_FDCAN, PTP_MODE, WHEEL1_ID, LITACC, gRobot.wheelColor, NORMAL_MODE, DISABLE_ALL, DISABLE_ALL);
	DriverStateChassis(&WHEEL_FDCAN, PTP_MODE, WHEEL2_ID, LITACC, gRobot.wheelColor, NORMAL_MODE, DISABLE_ALL, DISABLE_ALL);
	DriverStateChassis(&WHEEL_FDCAN, PTP_MODE, WHEEL3_ID, LITACC, gRobot.wheelColor, NORMAL_MODE, DISABLE_ALL, DISABLE_ALL);
	DriverState(&UP_FDCAN, PTP_MODE, TURNTABLE_ID, DISABLE);
	DriverState(&UP_FDCAN, PTP_MODE, UPPER_ARM_ID, DISABLE);
	DriverState(&UP_FDCAN, PTP_MODE, FOREARM_ID, DISABLE);
	DriverState(&UP_FDCAN, PTP_MODE, WRIST_ID, DISABLE);
}
