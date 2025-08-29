// Copy from Geollay
#include "my_sensor.h"
#include <string.h>

/*****************电流传感器 ********************************/
float prim_pump_curr = 0;
float pump_curr = 0;
float filter_press = 0;
AverageFilter myFilter;

void ReadPress(void)
{
   //	pump_curr = prim_pump_curr * 3.3 * 1000 / 4096 / 66; //模拟量 / 分辨率 / 66 毫伏每安
   press_32 = ((press_adc / 65536 * 3.3) - 2.7) / 0.000025;
}

// ADC 转换完成回调函数
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
   if (hadc->Instance == ADC1)
   {
      //       prim_pump_curr = HAL_ADC_GetValue(&hadc1);
      //    }
      press_adc = HAL_ADC_GetValue(&hadc1);
   }
}

void initializeFilter(AverageFilter *filter)
{
   for (int i = 0; i < WINDOW_SIZE; ++i)
   {
      filter->window[i] = 0;
   }
   filter->index = 0;
}

// 算术平均滤波函数
float filterValue(AverageFilter *filter, float input)
{
   // 更新缓存区
   filter->window[filter->index] = input;
   filter->index = (filter->index + 1) % WINDOW_SIZE;

   // 计算平均值
   float sum = 0;
   for (int i = 0; i < WINDOW_SIZE; ++i)
   {
      sum += filter->window[i];
   }
   float average = sum / WINDOW_SIZE;

   return average;
}
/*****************电流传感器 ********************************/

/**************************气压传感器************************/
uint8_t cmd = GZP6847D_COM;
uint8_t result = 0;
float press_adc = 0;
float press_32 = 0;
int16_t temp_adc = 0;
float temp_32 = 0;
// 步骤数
uint8_t i2c_step = 0;
uint8_t reg_a5_value = 0;
uint8_t sensorData[DATA_LEN]; // 用来存储数据
/**************************气压传感器************************/

/*切比雪夫滤波器*/

// 每个二阶节的状态缓存（Direct Form II 结构只需两个状态变量）
const int NL[MWSPT_NSEC] = {1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 2, 1};
const real64_T NUM[MWSPT_NSEC][3] = {
    {0.7192427516457, 0, 0},
    {1, -1.85838971009, 1},
    {0.6841618167765, 0, 0},
    {1, -1.848613501455, 1},
    {0.6336564317826, 0, 0},
    {1, -1.826242518619, 1},
    {0.5636420493829, 0, 0},
    {1, -1.783788934305, 1},
    {0.4698179430348, 0, 0},
    {1, -1.703097903442, 1},
    {0.3511241291712, 0, 0},
    {1, -1.5357307653, 1},
    {0.2179785891126, 0, 0},
    {1, -1.127795653483, 1},
    {0.1030543801599, 0, 0},
    {1, 0.07485230071267, 1},
    {0.2364308954281, 0, 0},
    {1, 1, 0},
    {1, 0, 0}};
const int DL[MWSPT_NSEC] = {1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 2, 1};
const real64_T DEN[MWSPT_NSEC][3] = {
    {1, 0, 0},
    {1, -1.867797229316, 0.9696494038923},
    {1, 0, 0},
    {1, -1.804888137827, 0.908460999707},
    {1, 0, 0},
    {1, -1.731400293297, 0.8415028389439},
    {1, 0, 0},
    {1, -1.640953561158, 0.7628192093262},
    {1, 0, 0},
    {1, -1.527592538372, 0.6670824706597},
    {1, 0, 0},
    {1, -1.389368948175, 0.5523850789106},
    {1, 0, 0},
    {1, -1.23732985129, 0.4274517241615},
    {1, 0, 0},
    {1, -1.107381192464, 0.3212038102369},
    {1, 0, 0},
    {1, -0.5271382091437, 0},
    {1, 0, 0}};

static double w[MWSPT_NSEC][2];

// 初始化所有状态为 0
void IIRFilter_Init(void)
{
   memset(w, 0, sizeof(w));
}

// 处理单个输入点
double IIRFilter_Update(double input)
{
   double x = input;
   double y = 0;

   for (int i = 0; i < MWSPT_NSEC; ++i)
   {
      double w_new = x - DEN[i][1] * w[i][0] - DEN[i][2] * w[i][1];

      y = NUM[i][0] * w_new + NUM[i][1] * w[i][0] + NUM[i][2] * w[i][1];

      // 更新状态
      w[i][1] = w[i][0];
      w[i][0] = w_new;

      // 输出作为下一个 stage 的输入
      x = y;
   }

   return y;
}
/*切比雪夫滤波器*/