/**
 ******************************************************************************
 * @file    my_pccom.c
 * @author  Geollay
 * @version
 * @date    2024/12/13
 * @brief   与上位机通信，数据读取和转换
 ******************************************************************************
 **/

#include "my_communication.h"

#include <stdbool.h>

#include "my_main.h"

// 接收帧帧头帧尾
const char frame_header_read1 = 'S';
const char frame_header_read2 = 'P';
const char frame_tail_read1 = '\r';
const char frame_tail_read2 = '\n';

// 发送帧帧头帧尾
const char frame_header_write1 = 'Q';
const char frame_header_write2 = 'R';
const char frame_tail_write1 = '\r';
const char frame_tail_write2 = '\n';

trans_t TX_Data; // 发送联合体
trans_r RX_Data; // 接收联合体

uint8_t rxbuf[MAX_GET_LEN] = {'\0'}; // 实际从usb接收到的数组，去除帧头帧尾接收联合体中的内容
uint8_t txbuf[USB_TX_LEN];			 // 实际给usb发送的数组，即上面发送联合体中的内容加上帧头帧尾
uint8_t receiveData[USB_RX_LEN];	 // 接受缓冲区

/*USB*/
uint8_t lastState = USBD_FAIL; // USB状态
int readLen = 0;			   // 从usbd_cdc_if.c中int8_t CDC_Receive_HS（）函数得到数据长度

/*
 * @brief 遍历从主控接收到的数据,检查数据格式
 * @return true--成功分析并赋值给数组 false--数据格式错误
 */
uint8_t CheckRxData(void)
{
	int rxLen = USB_RX_LEN;
	while (rxLen < readLen && rxbuf[rxLen - 1] != '\n')
	{
		rxLen++;
	}
	for (int i = rxLen - USB_RX_LEN; i >= 0; i--) // 匹配帧头帧尾 从后往前找
	{
		if (rxbuf[i] == frame_header_read1 && rxbuf[i + 1] == frame_header_read2 &&
			rxbuf[i + USB_RX_LEN - 2] == frame_tail_read1 &&
			rxbuf[i + USB_RX_LEN - 1] == frame_tail_read2)
		{
			for (int j = 0; j < USB_RX_LEN - 4; j++)
			{
				receiveData[j] = rxbuf[i + 2 + j];
			}
			return true;
		}
	}
	memset(receiveData, 0, sizeof(receiveData)); // 清空存数数组
	memset(rxbuf, 0, sizeof(rxbuf));
	return false;
}

/**
 * @brief  将机器人所有信息存入USB发送数组
 * @note float在前uint8_t在后
 */
void USB_Send(void) // 给小电脑返回数据
{
	txbuf[0] = frame_header_write1; // 帧头2位
	txbuf[1] = frame_header_write2;
	txbuf[USB_TX_LEN - 2] = frame_tail_write1; // 帧尾2位
	txbuf[USB_TX_LEN - 1] = frame_tail_write2;

	gRobot.mcuErrorFlag = gRobot.canErrorFlag | gRobot.get_pcErrorFlag;

	txbuf[2] = HB.PC;
	txbuf[3] = HB.wheel1;
	txbuf[4] = HB.wheel2;
	txbuf[5] = HB.wheel3;
	txbuf[6] = HB.leg1;
	txbuf[7] = HB.leg2;
	txbuf[8] = HB.leg3;
	txbuf[9] = HB.upper_arm;
	txbuf[10] = HB.forearm;
	txbuf[11] = HB.wrist;

	txbuf[12] = (gRobot.startGameFlag & 0x01) << 0 | (gRobot.selfCheckFlag & 0x01) << 1 |
				(gRobot.canErrorFlag & 0x01) << 2 | (gRobot.power._24v_state & 0x01) << 3 |
				(gRobot.valveStatus & 0x01) << 4 | (gRobot.restartSuccessFlag & 0x01) << 5|(gRobot.steeringWheelMode& 0x01) << 6 ;
	txbuf[13] = (gRobot.app.selfCheckFlag & 0x01) << 7 | (gRobot.app.resetFlag & 0x01) << 6 | (gRobot.app.shootFlag & 0x01) << 5 |
				(gRobot.app.controlModeFlag & 0x01) << 4 | (gRobot.app.passFlag & 0x01) << 3 |
				(gRobot.app.stopFlag & 0x01) << 2 | (gRobot.app.acspinFlag & 0x01) << 1 |
				(gRobot.app.clspinFlag & 0x01) << 0;
//	DEBUG("%d, %d, %d, %d, %d, %d\r\n", upperArmMsg.reset_true_flag, forearmMsg.reset_true_flag, upperArmTar.empty_distance_flag, forearmTar.empty_distance_flag, upperArmMsg.empty_distance_ok_flag, forearmMsg.empty_distance_ok_flag);
	txbuf[14] = (driver_state[0] & 0b111) << 0 | (upperArmMsg.reset_pos_ok_flag & 0x01) << 3 | (upperArmMsg.reset_true_flag & 0x01) << 4 | (upperArmMsg.empty_distance_ok_flag & 0x01) << 5;
	txbuf[15] = (driver_state[1] & 0b111) << 0 | (forearmMsg.reset_pos_ok_flag & 0x01) << 3 | (forearmMsg.reset_true_flag & 0x01) << 4 | (forearmMsg.empty_distance_ok_flag & 0x01) << 5;
	txbuf[16] = (driver_state[2] & 0b111) << 0 | (wristMsg.reset_pos_ok_flag & 0x01) << 3 | ( 1 & 0x01) << 4 | (wristMsg.empty_distance_ok_flag & 0x01) << 5 | (gRobot.forceshoot_flag & 0x01) << 6;
	
	txbuf[17] = (gRobot.app.dribbleFlag & 0x01) << 0 | (gRobot.app.switchModeFlag & 0x01) << 1 |
				(gRobot.app.defenceFlag & 0x01) << 2 | (gRobot.app.attackFlag & 0x01) << 3 | (gRobot.app.stillDribbleFlag & 0x01) << 4 |
				(gRobot.app.disableFlag & 0x01) << 5 | (gRobot.app.RorB & 0x01) << 6 | (gRobot.app.zeroFlag & 0x01) << 7 ;
	
	TX_Data.dataint32[0] = gRobot.periodCntMCU;
	TX_Data.data_float[1] = wheel1Msg.vel;
	TX_Data.data_float[2] = wheel1Msg.pos;
	TX_Data.data_float[3] = wheel2Msg.vel;
	TX_Data.data_float[4] = wheel2Msg.pos;
	TX_Data.data_float[5] = wheel3Msg.vel;
	TX_Data.data_float[6] = wheel3Msg.pos;


	TX_Data.dataint[7] = gRobot.tx_can_cnt;
	TX_Data.dataint[8] = gRobot.rx_can_cnt;

	TX_Data.data_float[9] = upperArmMsg.pos;
	TX_Data.data_float[10] = upperArmMsg.vel;
	TX_Data.data_float[11] = upperArmMsg.torque;

	TX_Data.data_float[12] = forearmMsg.pos;
	TX_Data.data_float[13] = forearmMsg.vel;
	TX_Data.data_float[14] = forearmMsg.torque;

	TX_Data.data_float[15] = wristMsg.pos;
	TX_Data.data_float[16] = wristMsg.vel;
	TX_Data.data_float[17] = wristMsg.torque;

	TX_Data.dataint[18] = upperArmTar.posPulse;
	TX_Data.dataint[19] = forearmTar.posPulse;
	TX_Data.dataint[20] = wristTar.posPulse;

	TX_Data.data_float[21] = upperArmTar.pvt_kp;
	TX_Data.data_float[22] = upperArmTar.pvt_kd;
	TX_Data.data_float[23] = forearmTar.pvt_kp;
	TX_Data.data_float[24] = forearmTar.pvt_kd;
	TX_Data.data_float[25] = wristTar.pvt_kp;
	TX_Data.data_float[26] = wristTar.pvt_kd;

	TX_Data.data_float[27] = gRobot.app.rc_X;
	TX_Data.data_float[28] = gRobot.app.rc_Y;
	//DEBUG("%f\r\n",gRobot .app.rc_X);

	TX_Data.data_float[29] = gRobot.rollerSpeed1;
	TX_Data.data_float[30] = gRobot.rollerSpeed2;

	TX_Data.data_float[31] = upperArmMsg.ex_curr;
	TX_Data.data_float[32] = forearmMsg.ex_curr;
	TX_Data.data_float[33] = wristMsg.ex_curr;

	TX_Data.data_float[34] = filter_press;

	TX_Data.dataint[35] = gRobot.app.sneak_or_fool;
	TX_Data.dataint[36] = gRobot.app.trickCnt;
	
	TX_Data.data_float[37] = gRobot.add_angle;
	TX_Data.data_float[38] = gRobot.add_dis;
	// 联合体发送
	for (uint8_t i = 2 + USB_TX_UINT8_NUM; i < (USB_TX_LEN - 2); i++)
	{
		txbuf[i] = TX_Data.datac[i - (2 + USB_TX_UINT8_NUM)];
	}
}
/**
 * @brief  将主控板接收到的数据存入float中
 */
void USB_Receive(void) // 收并配对小电脑发来的数据
{
	// 遍历从PC接收到的数据 找到帧头帧尾
	if (CheckRxData())
	{
		// TODO:加上其他的数据
		HB.PC = 0; // 心跳包归零表示收到
		//		DEBUG("OK!\r\n");

		gRobot.runSucceedFlag = (receiveData[0] >> 0) & 0x01;
		gRobot.pcErrorFlag = (receiveData[0] >> 1) & 0x01;
		gRobot.onlyNeedStart = (receiveData[0] >> 2) & 0x01;
		gRobot.valveStatus = (receiveData[0] >> 3) & 0x01;
		gRobot.steeringWheelMode = (receiveData[0] >> 4) & 0x03;
		gRobot.arm_reset_flag = (receiveData[0] >> 6) & 0x01;
		gRobot.arm_reset_beep_flag = (receiveData[0] >> 7) & 0x01;
		gRobot.pump_enable_flag = (receiveData[1] >> 0) & 0x01;
		gRobot.read_reset_flag = (receiveData[1] >> 1) & 0x01;

		gRobot.driverState[0] = (receiveData[2] >> 0) & 0b111;
		gRobot.driverState[1] = (receiveData[3] >> 0) & 0b111;
		gRobot.driverState[2] = (receiveData[4] >> 0) & 0b111;
		
		
		upperArmTar.empty_distance_flag = (receiveData[2] >> 3) & 0x01;
		forearmTar.empty_distance_flag = (receiveData[3] >> 3) & 0x01;
		wristTar.empty_distance_flag = (receiveData[4] >> 3) & 0x01;

		upperArmTar.reset_flag = (receiveData[2] >> 4) & 0x01;
		forearmTar.reset_flag = (receiveData[3] >> 4) & 0x01;
		wristTar.reset_flag = (receiveData[4] >> 4) & 0x01;

		gRobot.lightStripColor = receiveData[5];

		// 联合体接收
		for (uint8_t i = USB_RX_UINT8_NUM; i < USB_RX_LEN - 2; i++)
		{
			RX_Data.datac[i - USB_RX_UINT8_NUM] = receiveData[i];
		}

		gRobot.periodCntPC = RX_Data.dataint32[0]; // PC运行时间

		// 舵轮
		wheel1Tar.vel = RX_Data.dataf[1];
		wheel1Tar.pos = RX_Data.dataf[2];
		wheel2Tar.vel = RX_Data.dataf[3];
		wheel2Tar.pos = RX_Data.dataf[4];
		wheel3Tar.vel = RX_Data.dataf[5];
		wheel3Tar.pos = RX_Data.dataf[6];

		// arm
		upperArmTar.pos = -RX_Data.dataf[7];
		upperArmTar.vel = -RX_Data.dataf[8];
		upperArmTar.tor = -RX_Data.dataf[9];
		upperArmTar.pvt_kp = RX_Data.dataf[10];
		upperArmTar.pvt_kd = RX_Data.dataf[11];

		forearmTar.pos = -RX_Data.dataf[12];
		forearmTar.vel = -RX_Data.dataf[13];
		forearmTar.tor = -RX_Data.dataf[14];
		forearmTar.pvt_kp = RX_Data.dataf[15];
		forearmTar.pvt_kd = RX_Data.dataf[16];

		wristTar.pos = -RX_Data.dataf[17];
		wristTar.vel = -RX_Data.dataf[18];
		wristTar.tor = -RX_Data.dataf[19];
		wristTar.pvt_kp = RX_Data.dataf[20];
		wristTar.pvt_kd = RX_Data.dataf[21];

		upperArmTar.posPulse = RX_Data.dataint[22];
		forearmTar.posPulse = RX_Data.dataint[23];
		wristTar.posPulse = RX_Data.dataint[24];

		gRobot.rollerSpeed1 = RX_Data.dataint[25];
		gRobot.rollerSpeed2 = RX_Data.dataint[26];
		
		upperArmTar.Vel_kp = RX_Data.dataf[27];
		forearmTar.Vel_kp = RX_Data.dataf[28];
		wristTar.Vel_kp = RX_Data.dataf[29];

		upperArmTar.Vel_ki = RX_Data.dataf[30];
		forearmTar.Vel_ki = RX_Data.dataf[31];
		wristTar.Vel_ki = RX_Data.dataf[32];

		memset(receiveData, 0, sizeof(receiveData));
	}
	memset(rxbuf, 0, sizeof(rxbuf));
}

/**
 * @brief  上下位机通信
 */
void ReadFromPc(void)
{
	// 处理小电脑数据
	USB_Receive();
	lastState = USBD_FAIL;
}

void SendToPc(void)
{
	// 将机器人所有信息存入USB发送数组
	USB_Send();
	// 暂时清除app标志位
	ClearAllAPPFlag();
	// 调用USB发送函数，第一次先主控发数使小电脑和主控周期同步
	CDC_Transmit_HS(txbuf, USB_TX_LEN);
}
