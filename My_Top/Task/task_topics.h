#pragma once

typedef struct {
    // 实际位置/速度
    float theta_m;               //电机原始弧度
    float theta_m_speed;        //电机原始角速度(弧度)
    float theta_deg_final;       //最终的电机角度 (度)
    float reg_final;             //最终的电机角度 (弧度)

    float Angular_velocity_final; //最终的角速度

    // 目标位置/速度
    float _target_location2;     //目标位置 (度)
    float _target_speed;         //目标速度


    //热敏电阻
    float ADC_Temp_Val = 0.0f; //热敏电阻温度

    //调试信息
    float updown_duty = 0.0f;//电机全局占空比
    

} motor_telem_t;                  // 每路电机一个结构体

#define UORB_TOPIC_MOTOR "motor_telem"


