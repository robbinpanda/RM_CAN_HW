//
// Created by RobbinPanda on 24-10-12.
//
#include "main.h"
#include "can.h"
#include "usart.h"
#include "tim.h"
extern uint8_t rx_message[8];
extern CAN_RxHeaderTypeDef rx_header;

extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t tx_message[8];
extern uint32_t txMailBox;





class M3508_Motor {
private:
    float ratio_;// 电机减速比
    float angle_;//存在误差的储存方式，输出端累计转动角度
    struct {
        int r;
        float angle;
    } angle_new;//用结构将输出端角度分开储存减少误差
    float delta_angle_;// deg 输出端新转动的角度
    float ecd_angle_;// deg 当前电机编码器角度
    float last_ecd_angle_;  // deg 上次电机编码器角度
    float delta_ecd_angle_; // deg 编码器端新转动的角度
    float rotate_speed_;
    // dps 反馈转子转速
    float current_;
    float temp;
public:
    M3508_Motor();
    void canRxMsgCallback(uint8_t rx_data[8]);
    uint8_t message1[];
};

M3508_Motor::M3508_Motor():
ratio_(3591.0f / 187.0f),
angle_(0.0f),
delta_angle_(0.0f),
ecd_angle_(0.0f),
last_ecd_angle_(0.0f),
delta_ecd_angle_(0.0f),
rotate_speed_(0.0f),
current_(0.0f),
temp(0.0f)
{}

float linearMapping(int in, int in_min, int in_max, float out_min, float out_max) {
    float slope; //设置斜率
    slope = (out_max - out_min) / static_cast<float>(in_max - in_min);
    return slope*static_cast<float>(in - in_min) + out_min;
}

void M3508_Motor::canRxMsgCallback(uint8_t rx_message[8]){
    uint16_t angle_raw =(static_cast<uint16_t>(rx_message[0]) << 8) | rx_message[1];
    // 通过位运算将两个数据合并得到转子机械角度
    ecd_angle_ = linearMapping(angle_raw , 0 , 8191 , 0 , 360);
    // 通过 C620 中的数据将传输数据计算成为角度的值
    rotate_speed_ = static_cast<float>((static_cast<uint16_t>(rx_message[2]) << 8) | rx_message[3]);
    // 通过位运算将两个数据合并得到转子转速
    current_ = static_cast<float>((static_cast<uint16_t>(rx_message[4]) << 8) | rx_message[5]);
    // 通过位运算将两个数据合并得到实际转矩电流
    temp = static_cast<float>(rx_message[6]);
    // 得到电机温度
    if(ecd_angle_ > last_ecd_angle_){
        delta_ecd_angle_ = ecd_angle_ - last_ecd_angle_;
    }
    else{
        delta_ecd_angle_ = ecd_angle_ - last_ecd_angle_ + 360.0f;
    }// 转过一圈需要补 360 deg
    last_ecd_angle_ = ecd_angle_; // 更新 last_ecd_angle
    delta_angle_ = delta_ecd_angle_ / ratio_; // 输出端转动角度 = 编码器转动角度 / 电机减速比
    angle_ = angle_ + delta_angle_; // 累加得到总共转过的角度
    message1[0] = rotate_speed_;
}

M3508_Motor motor;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    HAL_CAN_GetRxMessage(&hcan1,0,&rx_header,rx_message);
    motor.canRxMsgCallback(rx_message);
}

void SetMotor1(int current) {
    tx_header.StdId = 0x100;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_message[2] = 0;
    tx_message[3] = current;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        HAL_UART_Transmit_IT(&huart6, motor.message1, 1);
        SetMotor1(0x80);
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_message, &txMailBox);
    }
}
