/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    my_fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the my_fdcan.c file
  *******************************************************************************/
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MY_FDCAN_H__
#define __MY_FDCAN_H__


/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */
#define CAN_ID_DRIVE_CTRL        0x32     //�������֡ID
#define CAN_ID_ENABLE_CTRL       0x38     //���ʹ��֡ID
#define CAN_ID_SAVE_CTRL         0x39     //�������֡ID
#define CAN_ID_PARAM_CTRL        0x35     //�����޸�֡ID
#define CAN_ID_PWR_ON_FEEDBACK   0x450    //�ϵ練��֡��ID
#define CAN_ID_MOTOR_FEEDBACK    0x50     //�������֡��ID

// ����ָ�����ʹ��֡��
typedef enum {
    MOTOR_CMD_DISABLE = 1,                //ʧ��
    MOTOR_CMD_ENABLE  = 2,                //ʹ��
    MOTOR_CMD_CALIB   = 3                 //У׼
} motor_cmd_t;

// ����ָ�� �����ڱ���֡��
#define R80_SAVE_PARAMS      1            //�������
#define R80_SAVE_POS         1            //����λ��

// ���ģʽ
typedef enum {
    MODE_CURRENT = 3,                     //������
    MODE_VELOCITY = 4,                    //�ٶȻ�
    MODE_POSITION = 5                     //λ�û�
} motor_mode_t;

// ����ID
typedef enum {
    PARAM_ID_MOTOR_ID = 0x01,             //���ID
    PARAM_ID_OVER_CURRENT = 0x02,         //�������ֵ
    PARAM_ID_RUN_MODE = 0x03,             //�������ģʽ
    PARAM_ID_VEL_PID = 0x04,              //�ٶȻ�Kp Ki��Kd
    PARAM_ID_POS_PID = 0x05               //λ�û�Kp Ki��Kd
} param_id_t;

// PID�����ṹ��
typedef struct {
    float kp;
    float ki;
    float kd;
} pid_params_t;

// �����붨��
typedef enum {
    R80_ERROR_NONE          = 0x00,
    R80_ERROR_UNDER_VOLTAGE = 0x01,
    R80_ERROR_OVER_VOLTAGE  = 0x02,
    R80_ERROR_OVER_CURRENT  = 0x04,
    R80_ERROR_OVER_TEMP     = 0x08
} r80_error_code_t;

//����������ݽṹ��
typedef struct {
    uint8_t motor_id;           // ���ID (1-4)
    
    // ԭʼ��������
    float velocity_rps;         // �ٶ� (ת/��)
    float current_a;            // ���� (����)
    int16_t raw_position;       // ԭʼ������ֵ
    r80_error_code_t error_code;// ������
    uint8_t bus_voltage;        // ���ߵ�ѹ
    
    // ����õ���λ����Ϣ
    int16_t last_raw_position;  // ��һ��ԭʼλ��
    int32_t total_count;        // �ۼƱ�����������ʵʱλ�ã�
    float total_revolutions;    // �ۼ�Ȧ��
    float total_angle_deg;      // �ۼƽǶȣ��ȣ�
    
    uint32_t timestamp;         // ʱ���
} r80_feedback_t;

extern r80_feedback_t motors[4];
/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
extern uint8_t can1ReceiveFlag;
extern uint8_t can3ReceiveFlag;

void FDCAN_ENABLE(void);

/*******************************R80���FDCANͨ��************************************/
void R80_Enable(uint8_t motor1_enable, uint8_t motor2_enable, uint8_t motor3_enable, uint8_t motor4_enable);
void R80_Drive(uint16_t motor1_val, uint16_t motor2_val, uint16_t motor3_val, uint16_t motor4_val);
void R80_Save(uint8_t save_params, uint8_t save_position);
void R80_SetParam(uint8_t motor_id, uint8_t param_id, uint8_t set_val1, uint8_t set_val2, uint8_t set_val3, uint8_t set_val4, uint8_t set_val5, uint8_t set_val6);
void pid_to_data(float kp, float ki, float kd, uint8_t* data);
void R80_SetMotorID(uint8_t current_id, uint8_t new_id);
void R80_Set_CurrentLimit(uint8_t motor_id, uint8_t current_amps);
void R80_Set_Mode(uint8_t motor_id, uint8_t mode);
void R80_Velocity_PID(uint8_t motor_id, float kp, float ki, float kd);
void R80_Position_PID(uint8_t motor_id, float kp, float ki, float kd);
void R80_Save_Param(uint8_t motor_id);
void R80_Save_Pos(uint8_t motor_id);
void R80_SingleMotor_Enable(uint8_t motor_id);
void R80_SingleMotor_Disable(uint8_t motor_id);
void R80_SingleMotor_Calib(uint8_t motor_id);
void R80_AllMotor_Enable(void);
void R80_AllMotor_Disable(void);
void R80_AllMotor_Calib(void);
uint16_t Change_ValueForm(float value, float Limit);
void R80_Current_cmd(float motor1_Current, float motor2_Current, float motor3_Current, float motor4_Current);
void R80_Velocity_cmd(float motor1_Velocity, float motor2_Velocity, float motor3_Velocity, float motor4_Velocity);
void R80_Position_cmd(float motor1_Position, float motor2_Position, float motor3_Position, float motor4_Position);

void R80_ParseFeedback(uint32_t std_id, const uint8_t data[8], r80_feedback_t* feedback);
const char* R80_ErrorToString(r80_error_code_t error_code);
void R80_InitFeedback(r80_feedback_t* feedback, uint8_t motor_id);
void R80_UpdatePosition(r80_feedback_t* feedback);
/* USER CODE END Prototypes */



#endif /* __MY_FDCAN_H__ */

