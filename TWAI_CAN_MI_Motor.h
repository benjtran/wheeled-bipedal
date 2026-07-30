#ifndef _TWAI_CAN_MI_MOTOR_H__
#define _TWAI_CAN_MI_MOTOR_H__

/*
    小米电机驱动器通信协议及使用说明

    电机通信为 CAN 2.0 通信接口，波特率 1Mbps，采用扩展帧格式，如下所示：
    数据域          29 位 ID        8Byte 数据区
    大小 Bit28~bit24 bit23~8 bit7~0 Byte0~Byte7
    描述 通信类型 数据区 2 目标地址 数据区 1

    电机支持的控制模式包括：
    运控模式：给定电机运控 5 个参数；
    电流模式：给定电机指定的 Iq 电流；
    速度模式：给定电机指定的运行速度；
    位置模式：给定电机指定的位置，电机将运行到该指定的位置；

    本台测试用的小米电机CAN ID=1，主机ID=0

    注意配置顺序：
        1. 电机CAN初始化->电机ID初始化->设定电机机械零点->设置电机控制模式->设置对应模式的控制参数->使能(启动)电机
        2. 必须给电机发数据才会有数据返回

    测试用例：
    MI_Motor_ M1_con;

    void setup() {
        Motor_CAN_Init();
        M1_con.Motor_Con_Init(MOTER_1_ID);
        M1_con.Motor_Set_Zero();
        M1_con.Change_Mode(SPEED_MODE);
        M1_con.Motor_Enable();
    }

    void loop() {
        M1_con.Motor_Data_Updata(20);
        Serial.printf("M1数据: %d,%d,%d,%d,%d,%d,%d,%d,%d,angle:%.2f,speed:%.2f,torque:%.2f,temp:%.2f,\r\n",
                                      M1_con.motor_rx_data.master_id,M1_con.motor_rx_data.motor_id,
                                      M1_con.motor_rx_data.err_sta,M1_con.motor_rx_data.HALL_err,
                                      M1_con.motor_rx_data.magnet_err,M1_con.motor_rx_data.temp_err,
                                      M1_con.motor_rx_data.current_err,M1_con.motor_rx_data.voltage_err,
                                      M1_con.motor_rx_data.mode_sta,
                                      M1_con.motor_rx_data.cur_angle,M1_con.motor_rx_data.cur_speed,
                                      M1_con.motor_rx_data.cur_torque,M1_con.motor_rx_data.cur_temp);
        M1_con.Set_SpeedMode(-1);
        vTaskDelay(20);
    }
*/

#include "Arduino.h"
#include <ESP32-TWAI-CAN.hpp>

/*CAN设置*/
#define CAN_TX          13
#define CAN_RX          4

#define MASTER_ID       0
#define MOTER_1_ID      127
#define MOTER_2_ID      1

/*基础配置*/
#define P_MIN           -12.5f
#define P_MAX           12.5f
#define V_MIN           -30.0f
#define V_MAX           30.0f
#define KP_MIN          0.0f
#define KP_MAX          500.0f
#define KD_MIN          0.0f
#define KD_MAX          5.0f
#define T_MIN           -12.0f
#define T_MAX           12.0f

/*数据配置*/
#define RUN_MODE        0x7005    // 运行模式, uint8, 1字节, 0运控模式,1位置模式，2速度模式，3电流模式
#define CTRL_MODE       0
#define POS_MODE        1
#define SPEED_MODE      2
#define CUR_MODE        3
#define IQ_REF          0x7006    // 电流模式 Iq 指令, float, 4字节, -23~23A
#define SPD_REF         0x700A    // 转速模式转速指令, float, 4字节, -30~30rad/s
#define LIMIT_TORQUE    0x700B    // 转矩限制, float, 4字节, 0~12Nm
#define CUR_KP          0x7010    // 电流的 Kp, float, 4字节, 默认值 0.125
#define CUR_KI          0x7011    // 电流的 Ki, float, 4字节, 默认值 0.0158
#define CUR_FILT_GAIN   0x7014    // 电流滤波系数 filt_gain, float, 4字节, 0~1.0，默认值 0.1
#define LOC_REF         0x7016    // 位置模式角度指令, float, 4字节, rad
#define LIMIT_SPD       0x7017    // 位置模式速度限制, float, 4字节, 0~30rad/s
#define LIMIT_CUR       0x7018    // 速度位置模式电流限制, float, 4字节, 0~23A
#define MECH_POS        0x7019    // 负载端计圈机械角度, float, 4字节, rad, 只读
#define IQF             0x701A    // iq 滤波值, float, 4字节, -23~23A, 只读
#define MECH_VEL        0x701B    // 负载端转速, float, 4字节, -30~30rad/s, 只读
#define VBUS            0x701C    // 母线电压, float, 4字节, V, 只读
#define ROTATION        0x701D    // 圈数, int16, 2字节, 圈数, 可读写
#define LOC_KP          0x701E    // 位置的 kp, float, 4字节, 默认值 30, 可读写
#define SPD_KP          0x701F    // 速度的 kp, float, 4字节, 默认值 1, 可读写
#define SPD_KI          0x7020    // 速度的 ki, float, 4字节, 默认值 0.002, 可读写

//拆分29位ID
#define RX_29ID_DISASSEMBLE_MASTER_ID(id)       (uint8_t)((id)&0xFF)
#define RX_29ID_DISASSEMBLE_MOTOR_ID(id)        (uint8_t)(((id)>>8)&0xFF)
//右移16位获取所有故障信息(bit21-16，共6位，111111->0x3F)，三元运算符判断是否有故障。0无1有
#define RX_29ID_DISASSEMBLE_ERR_STA(id)         (uint8_t)(((((id)>>16)&0x3F) > 0) ? 1 : 0)
#define RX_29ID_DISASSEMBLE_HALL_ERR(id)        (uint8_t)(((id)>>20)&0X01)
#define RX_29ID_DISASSEMBLE_MAGNET_ERR(id)      (uint8_t)(((id)>>19)&0x01)
#define RX_29ID_DISASSEMBLE_TEMP_ERR(id)        (uint8_t)(((id)>>18)&0x01)
#define RX_29ID_DISASSEMBLE_CURRENT_ERR(id)     (uint8_t)(((id)>>17)&0x01)
#define RX_29ID_DISASSEMBLE_VOLTAGE_ERR(id)     (uint8_t)(((id)>>16)&0x01)
//模式状态: 0:Reset模式[复位]; 1:Cali 模式[标定]; 2:Motor模式[运行]
#define RX_29ID_DISASSEMBLE_MODE_STA(id)        (uint8_t)(((id)>>22)&0x03)

//拆分数据
#define RX_DATA_DISASSEMBLE_CUR_ANGLE(data)     (uint16_t)((data[0]<<8)|data[1])
#define RX_DATA_DISASSEMBLE_CUR_SPEED(data)     (uint16_t)((data[2]<<8)|data[3])
#define RX_DATA_DISASSEMBLE_CUR_TORQUE(data)    (uint16_t)((data[4]<<8)|data[5])
#define RX_DATA_DISASSEMBLE_CUR_TEMP(data)      (uint16_t)((data[6]<<8)|data[7])

//转换系数
//转换过程：angle=data×8PI/65535-4PI,   范围：+-4PI
#define INT2ANGLE     0.000383501049
//转换过程：speed=data×60/65535-30,     范围：+-30rad/s
#define INT2SPEED     0.000915541314
//转换过程：torque=data×24/65535-12,    范围：+-12N·m
#define INT2TORQUE    0.000366216526

//发送数据包
typedef struct {
  uint32_t id:8;      //8位CAN ID
  uint32_t data:16;   //16位数据
  uint32_t mode:5;    //5位模式
  uint32_t res:3;     //3位保留
  uint8_t tx_data[8];
} can_frame_t;

//解析返回数据包
typedef struct {
  //29位ID解码状态
  uint8_t master_id;
  uint8_t motor_id;
  uint8_t err_sta;
  uint8_t HALL_err;
  uint8_t magnet_err;
  uint8_t temp_err;
  uint8_t current_err;
  uint8_t voltage_err;
  uint8_t mode_sta;

  //具体数据
  float cur_angle;      //(-4π~4π)
  float cur_speed;      //(-30rad/s~30rad/s)
  float cur_torque;     //（-12Nm~12Nm）
  float cur_temp;       //Temp(摄氏度）*10
} can_rx_frame_t;

class MI_Motor_
{
    private:
        uint8_t id;
        CanFrame rxFrame;               //接收原始数据

    public:
        can_rx_frame_t motor_rx_data;   //返回解析数据

        void Motor_Enable(void);
        void Motor_Reset(void);
        void Motor_Set_Zero(void);
        void Motor_ControlMode(float torque, float MechPosition, float speed, float kp, float kd);
        void Set_SingleParameter(uint16_t parameter, float index);
        void Decoding_Motor_data(CanFrame* can_Frame_point, can_rx_frame_t* motor_frame_point);
        void Set_SpeedMode(float speed);
        void Set_PosMode(float position, float max_speed);
        void Set_CurMode(float current);
        void Change_Mode(uint8_t mode);
        void Motor_Con_Init(uint8_t motor_id);
        uint8_t Motor_Data_Updata(uint32_t timeout);
};

extern void Motor_CAN_Init(void);

#endif