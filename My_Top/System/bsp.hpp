
#pragma once

#include "usart.h"
#include "tim.h"
#include "gpio.h"
#include "i2c.h"  
#include "spi.h"  
#include "adc.h"  


#define T0   275.15 // 0℃对应的开尔文温度
#define NTC_B  3950.0  //热敏电阻B值
#define NTC_R0 10.0  //热敏电阻初始阻值
#define T0_K  (25.0+T0) // 25℃对应的开尔文温度 25.0+273.15=298.15K


#define Fre_Accel_Fast  800.0f    // Hz/s，低速加速度（<1500Hz）
#define Fre_Accel_Slow  100.0f    // Hz/s，高速加速度（≥1500Hz）
#define Fre_Accel_Threshold   1500.0f  // 快慢分界线（Hz）

#define DC_Open_Loop_FREQ_DIV 4  //开环控制 
#define DC_VELOCITY_LOOP_FREQ_DIV 4  //速度环 
#define DC_POSITION_LOOP_FREQ_DIV 16 //位置环
#define DC_MAIN_LOOP_FREQ_HZ 16000.0
#define DC_VELOCITY_LOOP_FREQ_HZ (DC_MAIN_LOOP_FREQ_HZ / DC_VELOCITY_LOOP_FREQ_DIV)
#define DC_POSITION_LOOP_FREQ_HZ (DC_MAIN_LOOP_FREQ_HZ / DC_POSITION_LOOP_FREQ_DIV)
#define DC_Open_Loop_FREQ_HZ (DC_MAIN_LOOP_FREQ_HZ / DC_Open_Loop_FREQ_DIV)

#define therm_hadc (&hadc2)  //热敏电阻ADC句柄


#define closed_loop_htim (&htim3)   //闭环控制定时器
#define System_htim (&htim5)    //系统计时定时器
#define Vofa_huart (&huart2)    //Vofa串口
#define CLI_huart (&huart1)     //CLI串口和打印输出

#define EEPROM_hi2c       (&hi2c1)      // XBLW24C64 EEPROM I2C句柄

// ============================================================
// 电机 PWM 配置结构体
// ============================================================
struct MotorPWM_Config {
    TIM_HandleTypeDef*  htim;        // HAL 定时器句柄（用于 HAL 函数调用）
    TIM_TypeDef*        Tim_base;    // TIM 大写
    uint32_t            ch_hin1;     // PWM 输出通道
    uint32_t            ch_hin2;     // PWM 输出通道
    SPI_HandleTypeDef*  hspi;        // SPI 句柄
    GPIO_TypeDef *    ENC_CS_PORT;     // 编码器SPI片选脚
    uint16_t            ENC_CS_PIN;    // 编码器SPI片选脚
    GPIO_TypeDef *    ENC_CAL_PORT;     // 编码器校准脚
    uint16_t            ENC_CAL_PIN;    // 编码器校准脚
    uint8_t adc_channel;          // ADC通道
};

extern const MotorPWM_Config M1_PWM;
extern const MotorPWM_Config M2_PWM;
extern const MotorPWM_Config M3_PWM;
extern const MotorPWM_Config M4_PWM;






