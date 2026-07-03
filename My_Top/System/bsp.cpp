#include "bsp.hpp"

//const 不可以被修改
// ---------- M1 电机 ----------
const MotorPWM_Config M1_PWM = {
    .htim      = &htim1,            
    .Tim_base  = TIM1,
    .ch_hin1   = TIM_CHANNEL_1,
    .ch_hin2 = TIM_CHANNEL_2,
    .hspi = &hspi3, //编码器SPI句柄
    .ENC_CS_PORT = ENC1_CS_GPIO_Port, //编码器片选引脚
    .ENC_CS_PIN = ENC1_CS_Pin, //编码器片选引脚
    .ENC_CAL_PORT = ENC1_CAL_GPIO_Port, //编码器校准引脚
    .ENC_CAL_PIN = ENC1_CAL_Pin, //编码器校准引脚
};

// ---------- M2 电机 ----------
const MotorPWM_Config M2_PWM = {
    .htim      = &htim1,            
    .Tim_base  = TIM1,
    .ch_hin1   = TIM_CHANNEL_3,
    .ch_hin2 = TIM_CHANNEL_4,
    .hspi = &hspi3, //编码器SPI句柄
    .ENC_CS_PORT = ENC2_CS_GPIO_Port, //编码器片选引脚
    .ENC_CS_PIN = ENC2_CS_Pin, //编码器片选引脚
    .ENC_CAL_PORT = ENC2_CAL_GPIO_Port, //编码器校准引脚
    .ENC_CAL_PIN = ENC2_CAL_Pin, //编码器校准引脚

};

// ---------- M3 电机 ----------
const MotorPWM_Config M3_PWM = {
    .htim      = &htim8,            
    .Tim_base  = TIM8,
    .ch_hin1   = TIM_CHANNEL_1,
    .ch_hin2 = TIM_CHANNEL_2,
    .hspi = &hspi3, //编码器SPI句柄
    .ENC_CS_PORT = ENC3_CS_GPIO_Port, //编码器片选引脚
    .ENC_CS_PIN = ENC3_CS_Pin, //编码器片选引脚
    .ENC_CAL_PORT = ENC3_CAL_GPIO_Port, //编码器校准引脚
    .ENC_CAL_PIN = ENC3_CAL_Pin, //编码器校准引脚
};
// ---------- M4 电机 ----------
const MotorPWM_Config M4_PWM = {
    .htim      = &htim8,            
    .Tim_base  = TIM8,
    .ch_hin1   = TIM_CHANNEL_3,
    .ch_hin2 = TIM_CHANNEL_4,
    .hspi = &hspi3, //编码器SPI句柄
    .ENC_CS_PORT = ENC4_CS_GPIO_Port, //编码器片选引脚
    .ENC_CS_PIN = ENC4_CS_Pin, //编码器片选引脚
    .ENC_CAL_PORT = ENC4_CAL_GPIO_Port, //编码器校准引脚
    .ENC_CAL_PIN = ENC4_CAL_Pin, //编码器校准引脚
};

