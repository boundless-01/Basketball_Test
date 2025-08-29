/**
  ******************************************************************************
  * @file    my_usart.c
  * @author  Geollay
  * @version
  * @date    2024/12/13
  * @brief   USART传输相关
  ******************************************************************************
**/
#include "my_usart.h"
#include "dma.h"
#include "my_main.h"

uint8_t app_rxbuffer[APP_RX_MAX_LEN];//APP接收缓冲区
uint8_t app_cnt = 0;
uint8_t app_data_len = 0;
trans apptrans;//APP收发数据联合体

uint8_t power_rxbuffer[POWER_RX_LEN] = {};//数字电源板信息缓冲区
HAL_StatusTypeDef status = 0;
typedef union {
    uint8_t bytes[6];  // 6字节数组
    float   value;     // 4字节 float（注意：会覆盖后2字节）
}SixUint8ToFloat;
SixUint8ToFloat UnionSixUint8ToFloat;
int fputc(int ch, FILE *f)//重写printf
{
	HAL_UART_Transmit(&DEBUG_UART_HANDLER, (uint8_t *)&ch, 1, 0xffff);
	return ch;
}

//APP空闲中断收数回调
void APP_GET_IDLECallback(void)
{
	app_data_len = APP_RX_MAX_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); // 这个函数返回一个uint16_t类型的值，表示当前 DMA 通道中剩余的传输数据项数
	if (app_data_len >= APP_DATA_MIN_LEN)
	{
		for (int i = app_data_len - 1; i - (APP_DATA_MIN_LEN - 1) >= 0; i--) // 从后往前匹配，一帧12字节
		{
			if (app_rxbuffer[i] == '\n' && app_rxbuffer[i - 1] == '\r') // 第11,12字节
			{
//				DEBUG("1111\r\n");
				gRobot.getAppFlag = 1;
				// Beep(500);

				if (app_data_len == 12)
				{
					//Beep(200);
					gRobot.getAppFlag = 1;
					gRobot.app.stopMoveFlag = 0;
					if (app_rxbuffer[i - 6] == 'Y' && app_rxbuffer[i - 11] == 'X')
					{
						//						Beep(200);
						for (int j = 3; j >= 0; j--)
						{
							apptrans.datac[3 - j] = app_rxbuffer[j + (i - 11) + 1];
						}
						gRobot.app.rc_X = apptrans.dataf;
//						DEBUG("%f \r\n",gRobot.app.rc_X);
						// memset(&apptrans, 0, sizeof(apptrans));

						for (int j = 3; j >= 0; j--)
						{
							apptrans.datac[3 - j] = app_rxbuffer[j + (i - 6) + 1];
						}
						gRobot.app.rc_Y = -apptrans.dataf;
						memset(&apptrans, 0, sizeof(apptrans));

						HB.rc = 0;//心跳包
					}
					DEBUG("%f %f\r\n",gRobot.app.rc_X,gRobot.app.rc_Y);
				}

				uint8_t resetOK = 0;
				if (app_data_len == 8)
				{

					// Beep(200);
					switch (app_rxbuffer[i - 7])
					{
						case 'B':
						{
							if (app_rxbuffer[i - 6] == 'L')
							{
								gRobot.app.RorB = 1;
							}
							break;
						}
						case 'R':
						{							// 复位
							if (app_rxbuffer[i - 6] == 'E')
							{
								gRobot.app.resetFlag = 1;
							}
							else if (app_rxbuffer[i - 6] == 'L')
							{
								gRobot.app.sneak_or_fool = 3;
								gRobot.app.trickCnt++;
							}
							else if (app_rxbuffer[i - 6] == 'R')
							{
								gRobot.app.sneak_or_fool = 4;
								gRobot.app.trickCnt++;
							}
							else if (app_rxbuffer[i - 6] == 'D')
							{
								gRobot.app.RorB = 0;
							}
							break;
						}
						case 'C': // 顺时针转
						{
							if (app_rxbuffer[i - 6] == 'L')
							{
								gRobot.app.clspinFlag = 1;
								gRobot.app.stopSpinFlag = 0;
							}
							else if(app_rxbuffer[i-6] == 'R'){
								for(int j = 1;j<=6;++j){
									UnionSixUint8ToFloat.bytes[j - 1] = app_rxbuffer[i-6+j];
								}
								gRobot.add_angle = UnionSixUint8ToFloat.value;
							}
							break;
						}

						case 'A': // 逆时针转
						{
							if (app_rxbuffer[i - 6] == 'C')
							{
								gRobot.app.acspinFlag = 1;
								gRobot.app.stopSpinFlag = 0;
							}
							else if (app_rxbuffer[i - 6] == 'U')
							{
								gRobot.app.controlModeFlag = !gRobot.app.controlModeFlag;
								gRobot.app.switchModeFlag = 1;
							}
							break;
						}

						case 'T': // 停车
						{
							if (app_rxbuffer[i - 6] == 'C')
							{
								gRobot.app.stopMoveFlag = 1;
								gRobot.app.rc_Y = 0.0f;
								gRobot.app.rc_X = 0.0f;
							}
							else if (app_rxbuffer[i - 6] == 'Z')
							{
								gRobot.app.acspinFlag = 0;
								gRobot.app.clspinFlag = 0;
								gRobot.app.stopSpinFlag = 1;
							}
							break;
						}

						case 'S':
						{
							if (app_rxbuffer[i - 6] == 'H')
							{
								gRobot.app.shootFlag = 1;
							}
							else if(app_rxbuffer[i - 6] == 'C')
							{
								gRobot.app.selfCheckFlag = 1;
							}
							else if (app_rxbuffer[i - 6] == 'E')
							{
								gRobot.app.defenceFlag = 1;
							}
							else if (app_rxbuffer[i - 6] == 'T')
							{
								gRobot.app.disableFlag = 1;
							}
							break;
						}

						case 'F':
						{
							if (app_rxbuffer[i - 6] == 'L')
							{
								gRobot.app.sneak_or_fool = 1;
								gRobot.app.trickCnt++;
							}
							else if (app_rxbuffer[i - 6] == 'R')
							{
								gRobot.app.sneak_or_fool = 2;
								gRobot.app.trickCnt++;
							}
							else if (app_rxbuffer[i - 6] == 'N')
							{
								gRobot.app.attackFlag = 1;
							}
							else if (app_rxbuffer[i - 6] == 'B'){
								for(int j = 1;j<=6;++j){
									UnionSixUint8ToFloat.bytes[j - 1] = app_rxbuffer[i-6+j];
								}
								gRobot.add_dis = UnionSixUint8ToFloat.value;
							}
							else if (app_rxbuffer[i - 6] == 'S'){
								gRobot.forceshoot_flag = 1;
							}
							break;
						}
						
						case 'P':
						{
							if (app_rxbuffer[i - 6] == 'B')
							{
								gRobot.app.passFlag = 1;
							}
							break;
						}

						case 'D':
						{	
							if(app_rxbuffer[i - 6] == 'R')
							{
								gRobot.app.dribbleFlag = 1;
							}
							break;
						}
						
						case 'Z':
						{
							if (app_rxbuffer[i - 6] == 'E')
							{
								gRobot.app.zeroFlag = 1;
							}
							break;
						}
						
						case 'd':
						{	
							if(app_rxbuffer[i - 6] == 'r')
							{
								gRobot.app.stillDribbleFlag = 1;
							}
							break;
						}
					}
				}
			}
			else
			{
				gRobot.getAppFlag = 0;
			}
		}
	}
				DEBUG("%d %d %d %d %d %d %f %f\r\n",\
			app_rxbuffer[0], gRobot.app.clspinFlag, gRobot.app.acspinFlag, gRobot.app.stopSpinFlag, gRobot.app.disableFlag,\
			gRobot.app.shootFlag, gRobot.app.rc_X, gRobot.app.rc_Y);
}

//用户编写的串口空闲中断的回调
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	if (USART1 == huart->Instance)//如果是串口1
	{
		if (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) != RESET)
		{
			if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)//判断是否空闲中断
			{
				__HAL_UART_CLEAR_IDLEFLAG(huart);//清除空闲中断标志,否则会一直进入
				HAL_UART_AbortReceive(huart);
				HAL_UART_DMAStop(&APP_USART);//停止本次DMA传输
				
				APP_GET_IDLECallback();
				
				memset(app_rxbuffer,0,sizeof(app_rxbuffer));//清空收数数组
				SCB_InvalidateDCache_by_Addr((uint32_t *)app_rxbuffer,APP_RX_MAX_LEN);//用于将cache数据无效化，否则app_rxbuffer无法更新
				HAL_UART_Receive_DMA(&APP_USART,(uint8_t *)app_rxbuffer,APP_RX_MAX_LEN);//重新开启DMA收数
			}
		}
	}
}

//读取数字电源板发送的电压等信息
void GetPowerState(void)
{
	static uint8_t lowCnt = 0;//低电压计数器（（避免瞬时波动误触发报警）

	//gRobot.power.voltage_adc = (uint16_t)atoi((char*)&power_rxbuffer[2]);
	gRobot.power.voltage_adc = (power_rxbuffer[2] << 8) | power_rxbuffer[1];
	
	//float vol_ans = (gRobot.power.voltage_adc * 3.3 / 4096) * 48.3 /3;
	float vol_ans = (gRobot.power.voltage_adc) / 1000.0f;
	if (vol_ans >= 22.f && vol_ans <= 25.5f)
	{
		//小于阈值电压报警
		gRobot.power.voltage_value = vol_ans;
		if (gRobot.power.voltage_value < 24.3f && lowCnt < 100) lowCnt++;//电压低于 23.5V：递增计数器 lowCnt
		else if (gRobot.power.voltage_value >= 24.3f && lowCnt > 0) lowCnt--;//电压恢复至 24.0V 以上：递减计数器
		if (lowCnt >= 100) gRobot.power.lowPowFlag = 1;
		//DEBUG("%f\r\n",gRobot.power.voltage_value);
	}
		
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//UART 接收完成中断回调函数，（HAL_UART_Receive_IT）接收到指定长度的数据后，自动调用此函数
{
	UNUSED(huart);//显式标记参数 huart 未使用，避免编译器警告
	if(huart == &POWER_UART)//POWER_UART为huart9
	{
		if(power_rxbuffer[0] ==  0x01 && power_rxbuffer[7] == 0xFE) //类似于判断帧头帧尾
		{
			static uint8_t lowCnt = 0;//低电压计数器（（避免瞬时波动误触发报警）

			gRobot.power.voltage_adc = (power_rxbuffer[2] << 8) | power_rxbuffer[1];
	
			float vol_ans = (gRobot.power.voltage_adc) / 1000.0f;
			if (vol_ans >= 22.f && vol_ans <= 25.5f)
			{
				//小于阈值电压报警
				gRobot.power.voltage_value = vol_ans;
				if (gRobot.power.voltage_value < 24.3f && lowCnt < 100) lowCnt++;//电压低于 23.5V：递增计数器 lowCnt
				else if (gRobot.power.voltage_value >= 24.3f && lowCnt > 0) lowCnt--;//电压恢复至 24.0V 以上：递减计数器
				if (lowCnt >= 100) gRobot.power.lowPowFlag = 1;
			}
		}
		memset(power_rxbuffer, 0x00, sizeof(power_rxbuffer)); //清空数组
	}
	
}

void ClearAllAPPFlag(void)
{
	gRobot.app.shootFlag = 0;
	gRobot.app.resetFlag = 0;
	gRobot.app.selfCheckFlag = 0;
	gRobot.app.passFlag = 0;
	gRobot.app.dribbleFlag = 0;
	gRobot.app.switchModeFlag = 0;
	gRobot.app.defenceFlag = 0;
	gRobot.app.attackFlag = 0;
	gRobot.app.stillDribbleFlag = 0;
	gRobot.app.zeroFlag = 0;
	gRobot.forceshoot_flag = 0;
//	gRobot.app.disableFlag = 0;
}

//TODO:两车通信
/*两车通信部分 */
uint8_t robot_rxbuffer[ROBOT_COM_RX_MAX_LEN];//两车通信接收缓冲区
uint8_t robot_txbuffer[ROBOT_COM_DATA_LEN];
uint8_t robot_com_data_len = 0;
trans robot_com_trans;

//接收帧帧头帧尾
const char tx_header1  = 'W';
const char tx_header2  = 'Z';
const char tx_tail1    = '\r';
const char tx_tail2    = '\n';

//发送帧帧头帧尾
const char rx_header1 = 'W';
const char rx_header2 = 'Z'; 
const char rx_tail1   = '\r';
const char rx_tail2   = '\n';

robot_com_data_t robotTxData;
robot_com_data_t robotRxData;

void Send2Parnter(void)
{
    robot_txbuffer[0] = tx_header1;
    robot_txbuffer[1] = tx_header2;
    robot_txbuffer[ROBOT_COM_DATA_LEN - 2] = tx_tail1;
    robot_txbuffer[ROBOT_COM_DATA_LEN - 1] = tx_tail2;
	
    robotTxData.dataf[0] = gRobot.selfComData.pps_x;
    robotTxData.dataf[1] = gRobot.selfComData.pps_y;
    robotTxData.dataf[2] = gRobot.selfComData.pps_yaw;

    for (uint8_t i = 0; i < sizeof(robotTxData.datac); i++)
    {
        robot_txbuffer[2 + i] = robotTxData.datac[i];
    }
	
	//使发送RAM和cache一致
	//SCB_InvalidateDCache_by_Addr((uint32_t *)robot_txbuffer, ROBOT_COM_DATA_LEN);
    HAL_UART_Transmit_IT(&huart3, robot_txbuffer, ROBOT_COM_DATA_LEN);
}


void PartnerMsgReicve()
{
	robot_com_data_len = ROBOT_COM_RX_MAX_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
	if (robot_com_data_len >= ROBOT_COM_DATA_LEN)
	{
//		Beep(50);

		//DEBUG("robot_com_data_len:	%d\r\n ",robot_com_data_len);
		for (int i = robot_com_data_len - 1; i - (ROBOT_COM_DATA_LEN - 1) >= 0; i--) // 从后往前匹配，一帧16字节
		{

			if (robot_rxbuffer[i] == rx_header1 && robot_rxbuffer[i + 1] == rx_header2 && \
				robot_rxbuffer[i + ROBOT_COM_DATA_LEN - 2] == rx_tail1 && robot_rxbuffer[i + ROBOT_COM_DATA_LEN - 1] == rx_tail2)
			{
				// Beep(50);
				for (int j = 0; j < ROBOT_COM_DATA_LEN - 4; j++)
				{
					robotRxData.datac[j] = robot_rxbuffer[i + 2 + j];
				}

				gRobot.partnerComData.pps_x = robotRxData.dataf[0];
				gRobot.partnerComData.pps_y = robotRxData.dataf[1];
				gRobot.partnerComData.pps_yaw = robotRxData.dataf[2];
			}

			memset(robotRxData.datac, 0, sizeof(robotRxData.datac));
		}
	}
}

void ROBOT_RxIRQHandler(UART_HandleTypeDef *huart)
{
	if (USART3 == huart->Instance)//如果是串口3
	{
		if (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE) != RESET)
		{
			if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
			{
				__HAL_UART_CLEAR_IDLEFLAG(huart);
				HAL_UART_AbortReceive(huart);
				HAL_UART_DMAStop(&ROBOT_COM_USART);

				//TODO:加机器人通信相关变量
				PartnerMsgReicve();
				
				memset(robot_rxbuffer,0,sizeof(robot_rxbuffer));
				SCB_InvalidateDCache_by_Addr((uint32_t *)robot_rxbuffer,ROBOT_COM_RX_MAX_LEN);
				HAL_UART_Receive_DMA(&ROBOT_COM_USART,(uint8_t *)robot_rxbuffer,ROBOT_COM_RX_MAX_LEN);
			}
		}
	}
}

void USART_ENABLE(void)
{
	HAL_UART_Receive_IT(&POWER_UART, (uint8_t *)power_rxbuffer, POWER_RX_LEN);//开启串口接收中断

	HAL_UART_Receive_DMA(&APP_USART,(uint8_t *)app_rxbuffer,APP_RX_MAX_LEN);//开启dma接收中断
	__HAL_UART_ENABLE_IT(&APP_USART,UART_IT_IDLE);//使能空闲中断
	HAL_UART_AbortReceive(&APP_USART);
	SCB_InvalidateDCache_by_Addr((uint32_t *)app_rxbuffer,APP_RX_MAX_LEN);//cache数据无效化，清除缓存

	HAL_UART_Receive_DMA(&ROBOT_COM_USART,(uint8_t *)robot_rxbuffer,ROBOT_COM_RX_MAX_LEN);//开启dma接收中断
	__HAL_UART_ENABLE_IT(&ROBOT_COM_USART,UART_IT_IDLE);
	HAL_UART_AbortReceive(&ROBOT_COM_USART);
	SCB_InvalidateDCache_by_Addr((uint32_t *)robot_rxbuffer,ROBOT_COM_RX_MAX_LEN);
	
}