// Copy from 奶龙
#ifndef _MY_SENSOR_H_
#define _MY_SENSOR_H_

#include "adc.h"
#include "tmwtypes.h"

/*****************电流传感器 ********************************/

/*****************滤波 ******************/
#define WINDOW_SIZE 10
 
// 定义算术平均滤波器结构体
typedef struct {
    float window[WINDOW_SIZE];
    int index;
} AverageFilter;

/*****************滤波 ******************/

extern float prim_pump_curr;
extern float pump_curr;
extern float filter_press;
extern AverageFilter myFilter;

void ReadPress(void);
//初始化滤波器
void initializeFilter(AverageFilter* filter);
//算术平均滤波
float filterValue(AverageFilter* filter, float input);
/*****************电流传感器 ********************************/

/**************************气压传感器**************************************
 * DEFINITIONS
 */
#define GZP6847D_SLAVE_ADDR         0x6D

#define GZP6847D_WRITE_BIT          0x00
#define GZP6847D_READ_BIT           0x01

#define GZP6847D_DATA_MSB_ADDR      0x06
#define GZP6847D_DATA_CSB_ADDR      0x07
#define GZP6847D_DATA_LSB_ADDR      0x08
#define GZP6847D_TEMP_MSB_ADDR      0x09
#define GZP6847D_TEMP_LSB_ADDR      0x0A
#define GZP6847D_CMD_ADDR           0x30
#define GZP6847D_SYS_CONFIG_ADDR    0xA5
#define GZP6847D_P_CONFIG_ADDR      0xA6

#define GZP6847D_ONE_TEMP           0x08
#define GZP6847D_ONE_PRESS          0x09
#define GZP6847D_COM                0x0A
#define GZP6847D_DORMANT            0x0B

#define GZP6847D_K_VALUE            64

#define DATA_LEN 5  // 3字节压力数据 + 2字节温度数据
/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * API FUNCTIONS
 */
 
void GZP6847D_ReadCombinedModeData(); 
void byte_to_bin_str(uint8_t byte, char *str);
void int32_to_bin_str(int32_t value, char *str);
void I2C_DMA_Start();
void Process_SensorData();
void CheckIfLock();
 
extern uint8_t cmd;
extern uint8_t result;
extern float press_adc;
extern float press_32;
extern int16_t temp_adc;
extern float temp_32;
extern uint8_t sensorData[DATA_LEN];


/*切比雪夫滤波器*/
#define MWSPT_NSEC 19
extern const int NL[MWSPT_NSEC];
extern const int DL[MWSPT_NSEC];
extern const real64_T NUM[MWSPT_NSEC][3];
extern const real64_T DEN[MWSPT_NSEC][3];
void IIRFilter_Init(void);
double IIRFilter_Update(double input);
/*切比雪夫滤波器*/

#endif