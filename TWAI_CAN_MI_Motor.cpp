#include "Arduino.h"
#include "TWAI_CAN_MI_Motor.h"

//把浮点数转换成uint_16 用在位置 扭矩 上面
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  if (x > x_max) x = x_max;
  else if (x < x_min) x = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

//底层的CAN发送指令，小米电机采用扩展帧，数据帧的格式
static void CAN_Send_Frame(can_frame_t* frame)
{
    CanFrame obdFrame = { 0 };
    uint32_t id_val, data_val, mode_val;
    uint32_t combined_val;

    obdFrame.extd = 1;              //0-标准帧; 1-扩展帧
    obdFrame.rtr = 0;               //0-数据帧; 1-远程帧
    obdFrame.ss = 0;                //0-错误重发; 1-单次发送(仲裁或丢失时消息不会被重发)，对接收消息无效
    obdFrame.self = 0;              //0-不接收自己发送的消息，1-接收自己发送的消息，对接收消息无效
    obdFrame.dlc_non_comp = 0;      //0-数据长度不大于8(ISO 11898-1); 1-数据长度大于8(非标);

    //拼接ID
    id_val = frame->id;
    data_val = frame->data;
    mode_val = frame->mode;
    combined_val |= (mode_val << 24);
    combined_val |= (data_val << 8);
    combined_val |= id_val;

    obdFrame.identifier = combined_val; //普通帧直接写id，扩展帧需要计算。11/29位ID
    obdFrame.data_length_code = 8;      //要发送的字节数

    for (int i = 0; i < 8; i++)
    {
        obdFrame.data[i] = frame->tx_data[i];
    }
    ESP32Can.writeFrame(obdFrame);
}

/**
      * @brief          电机使能
      */
void MI_Motor_::Motor_Enable(void)
{
    can_frame_t motor_con_frame;
    motor_con_frame.mode = 3;           //模式3，使能
    motor_con_frame.id = this->id;      //目标电机ID
    motor_con_frame.data = MASTER_ID;   //本机ID
    for (int i = 0; i < 8; i++)
    {
        motor_con_frame.tx_data[i] = 0;
    }
    CAN_Send_Frame(&motor_con_frame);
}

/**
      * @brief          电机停止
      */
void MI_Motor_::Motor_Reset(void)
{
    can_frame_t motor_con_frame;
    motor_con_frame.mode = 4;           //模式4，停止
    motor_con_frame.id = this->id;      //目标电机ID
    motor_con_frame.data = MASTER_ID;   //本机ID
    for (int i = 0; i < 8; i++)
    {
        motor_con_frame.tx_data[i] = 0;
    }
    CAN_Send_Frame(&motor_con_frame);
}

/**
      * @brief          设置电机机械零点
      */
void MI_Motor_::Motor_Set_Zero(void)
{
    can_frame_t motor_con_frame;
    motor_con_frame.mode = 6;           //模式6，设置零点
    motor_con_frame.id = this->id;      //目标电机ID
    motor_con_frame.data = MASTER_ID;   //本机ID
    for (int i = 0; i < 8; i++)
    {
        motor_con_frame.tx_data[i] = 0;
    }
    CAN_Send_Frame(&motor_con_frame);
}

/**
      * @brief          运动控制模式
      * @param[in]      torque 扭矩，N·m，-12到+12
      * @param[in]      MechPosition 角度，rad，-12.5到+12.5
      * @param[in]      speed 角速度，rad/s，-30到+30
      * @param[in]      kp 比例增益，0到500
      * @param[in]      kd 微分增益，0到5
      */
void MI_Motor_::Motor_ControlMode(float torque, float MechPosition, float speed, float kp, float kd)
{
    can_frame_t motor_con_frame;
    motor_con_frame.mode = 1;           //模式1,运控模式
    motor_con_frame.id = this->id;      //目标电机ID
    motor_con_frame.data = float_to_uint(torque, T_MIN, T_MAX, 16);

    motor_con_frame.tx_data[0] = float_to_uint(MechPosition, P_MIN, P_MAX, 16) >> 8;
    motor_con_frame.tx_data[1] = float_to_uint(MechPosition, P_MIN, P_MAX, 16);
    motor_con_frame.tx_data[2] = float_to_uint(speed, V_MIN, V_MAX, 16) >> 8;
    motor_con_frame.tx_data[3] = float_to_uint(speed, V_MIN, V_MAX, 16);
    motor_con_frame.tx_data[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8;
    motor_con_frame.tx_data[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16);
    motor_con_frame.tx_data[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8;
    motor_con_frame.tx_data[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16);
    CAN_Send_Frame(&motor_con_frame);
}

/**
      * @brief          设置单参数
      * @param[in]      parameter 参数数据
      * @param[in]      index 参数列表，详见电机说明书的4.1.11
      */
void MI_Motor_::Set_SingleParameter(uint16_t parameter, float index)
{
    can_frame_t motor_con_frame;
    motor_con_frame.mode = 18;          //模式18,单参数修改模式
    motor_con_frame.id = this->id;      //目标电机ID
    motor_con_frame.data = MASTER_ID;   //来标识主CAN_ID

    motor_con_frame.tx_data[2] = 0;
    motor_con_frame.tx_data[3] = 0;     //Byte2~3: 00

    memcpy(&motor_con_frame.tx_data[0], &parameter, 2);   //Byte0~1: index，参数列表详见4.1.11
    memcpy(&motor_con_frame.tx_data[4], &index, 4);       //Byte4~7: 参数数据，1字节数据在Byte4
    CAN_Send_Frame(&motor_con_frame);
}

/**
      * @brief          解析电机发送数据包
      * @param[in]      can_Frame_point CAN原始数据包的指针
      * @param[in]      motor_frame_point 电机解析数据包的指针
      */
void MI_Motor_::Decoding_Motor_data(CanFrame* can_Frame_point, can_rx_frame_t* motor_frame_point)
{
    motor_frame_point->master_id = RX_29ID_DISASSEMBLE_MASTER_ID(can_Frame_point->identifier);
    motor_frame_point->motor_id = RX_29ID_DISASSEMBLE_MOTOR_ID(can_Frame_point->identifier);
    motor_frame_point->err_sta = RX_29ID_DISASSEMBLE_ERR_STA(can_Frame_point->identifier);
    motor_frame_point->HALL_err = RX_29ID_DISASSEMBLE_HALL_ERR(can_Frame_point->identifier);
    motor_frame_point->magnet_err = RX_29ID_DISASSEMBLE_MAGNET_ERR(can_Frame_point->identifier);
    motor_frame_point->temp_err = RX_29ID_DISASSEMBLE_TEMP_ERR(can_Frame_point->identifier);
    motor_frame_point->current_err = RX_29ID_DISASSEMBLE_CURRENT_ERR(can_Frame_point->identifier);
    motor_frame_point->voltage_err = RX_29ID_DISASSEMBLE_VOLTAGE_ERR(can_Frame_point->identifier);
    motor_frame_point->mode_sta = RX_29ID_DISASSEMBLE_MODE_STA(can_Frame_point->identifier);

    //弧度单位
    motor_frame_point->cur_angle = RX_DATA_DISASSEMBLE_CUR_ANGLE(can_Frame_point->data) * INT2ANGLE - 4 * PI;
    motor_frame_point->cur_speed = RX_DATA_DISASSEMBLE_CUR_SPEED(can_Frame_point->data) * INT2SPEED - 30;
    motor_frame_point->cur_torque = RX_DATA_DISASSEMBLE_CUR_TORQUE(can_Frame_point->data) * INT2TORQUE - 12;
    motor_frame_point->cur_temp = RX_DATA_DISASSEMBLE_CUR_TEMP(can_Frame_point->data) / 10.0f;
}

/**
      * @brief          设置速度模式的参数
      * @param[in]      speed 角速度，rad/s，-30到+30
      */
void MI_Motor_::Set_SpeedMode(float speed)
{
    Set_SingleParameter(SPD_REF, speed);
}

/**
      * @brief          设置位置模式的参数
      * @param[in]      position 角度，rad，-12.5到+12.5
      * @param[in]      max_speed 角速度，rad/s，0到+30
      */
void MI_Motor_::Set_PosMode(float position, float max_speed)
{
    Set_SingleParameter(LIMIT_SPD, max_speed);
    Set_SingleParameter(LOC_REF, position);
}

/**
      * @brief          设置电流模式的参数
      * @param[in]      current 电流，A，-23到+23
      */
void MI_Motor_::Set_CurMode(float current)
{
    Set_SingleParameter(IQ_REF, current);
}

/**
      * @brief          设置运动模式
      * @param[in]      mode 运控模式：CTRL_MODE 电流模式：CUR_MODE 速度模式：SPEED_MODE 位置模式：POS_MODE
      */
void MI_Motor_::Change_Mode(uint8_t mode)
{
    uint16_t parameter = RUN_MODE;
    can_frame_t motor_con_frame;
    motor_con_frame.mode = 18;  //模式18，单参数写入
    motor_con_frame.data = MASTER_ID;
    motor_con_frame.id = this->id;    //电机ID
    for (int j = 0; j <= 7; j++) {
        motor_con_frame.tx_data[j] = 0;
    }
    motor_con_frame.tx_data[4] = mode;
    memcpy(&motor_con_frame.tx_data[0], &parameter, 2);
    CAN_Send_Frame(&motor_con_frame);
}

/**
      * @brief          电机初始化，在上位机查看电机ID和主机ID，主机ID默认为0
      * @param[in]      motor_id 电机ID
      */
void MI_Motor_::Motor_Con_Init(uint8_t motor_id)
{
    this->id = motor_id;
}

/**
      * @brief          CAN初始化
      */
void Motor_CAN_Init(void)
{
    ESP32Can.setPins(CAN_TX, CAN_RX);                //设置pin脚
    ESP32Can.setRxQueueSize(50);                     //设置缓存区大小
    ESP32Can.setTxQueueSize(50);                     //设置缓存区大小
    ESP32Can.setSpeed(ESP32Can.convertSpeed(1000));  //设置速度，一定要1M，其它库都没有1M，就这个库有
    ESP32Can.begin();
}

/**
      * @brief          更新电机数据
      * @param[in]      timeout 超时时间
      * @retval         err_sta 0：正常读取；1：读取到数据，但不是自己的；2：没有读取到数据
      */
uint8_t MI_Motor_::Motor_Data_Updata(uint32_t timeout)
{
    uint8_t err_sta = 0;
    if (ESP32Can.readFrame(rxFrame, timeout)) {
        if (RX_29ID_DISASSEMBLE_MOTOR_ID(rxFrame.identifier) == this->id) {
            Decoding_Motor_data(&rxFrame, &motor_rx_data);
        } else {
            err_sta = 1;
        }
    } else {
        err_sta = 2;
    }

    return err_sta;
}