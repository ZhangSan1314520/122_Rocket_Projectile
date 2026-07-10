#pragma once
#include "pid.hpp"
#include "pid_Increment.hpp"
#include "pll.hpp"
#include "kth_read.hpp"
#include "ex_math.hpp"
#include "bsp.hpp"
#include "XBLW24C64.hpp"
#include "my_dronecan.hpp"

#define max_chl_num 8 //最大通道数

enum Work_Mode
{
    open_loop = 0,//开环模式
    speed = 1,//速度模式
    position = 2,//位置模式

    EncoderCalibration = 99,//编码器校准模式
    null_mode = 100,        //无模式
};


class DC_Motor
{
public:
    DC_Motor(const MotorPWM_Config *motor_config, KTH7111 *encoder,PID_Increment *pid_inc_param ,PID *pid)
        : _motor_config(motor_config), _encoder(encoder), pid_speed_incparam(pid_inc_param) ,pid_location(pid){}//构造函数
    
    static uint8_t selected_motor; // 0=不选 M1~M4
    Work_Mode work_mode = open_loop; //电机工作模式
    bool motor_encoder_dir = true;//编码器方向，true为正，false为反
    bool control_init_flag = false; //电机控制初始化标志
    float motor_freq = 10000;//电机频率
    float motor_up_duty = 0.0f;//电机上桥臂占空比
    float motor_down_duty = 0.0f;//电机下桥臂占空比
    float updown_duty = 0.0f;//电机全局占空比
    float theta_m;//电机原始弧度
    float reg_final;//最终的电机角度 (弧度)
    float theta_deg_final ; //最终的电机角度 (角度)
    float Angular_velocity_final;  //最终的角速度
    float zero_offset;//电机零点偏移角度 (角度) 

    float _target_speed = 0.0f;//电机速度目标值
    float _target_location2 = 0.0f;//电机目标位置 (-180°~+180°)


    static uint16_t ADC_Value[max_chl_num]; //ADC采集值数组
    float ADC_RES_Val; //热敏电阻值
    float ADC_Temp_Val; //热敏电阻温度

    KTH7111   *_encoder; //编码器
    PID_Increment *pid_speed_incparam; //速度环 增量式PID
    PID       *pid_location; //位置环 位置式PID

    void test_laji();
    void My_DC_Motor_Init(void);
    void Motor_EN(bool en) ;//电机使能
    void Set_Motor_Frequency(void); //单独设置电机频率和占空比
    void Set_Motor_Glo_Duty();//全局设置电机占空比
    void Update_Speed_Angle_LPFAndPLL(); //更新速度
    void Speed_Loop(void); //速度环
    void Location_Loop(void); //位置环
    void Thermistor_Temp(); //计算热敏电阻温度

    int32_t laji; //用于记录中断间隔时间
private:
    uint16_t Fre_last = 0xff;   // 每个对象独立的上次频率 不能用 static
    float up_duty_last = -1.0f;   // 每个对象独立的上次占空比 不能用 static
    float down_duty_last = -1.0f;   // 每个对象独立的上次占空比 不能用 static
    bool _motor_is_on_last = false; //上次电机使能标志

    uint32_t timer_clock_freq_; // 定时器的实际计数时钟频率
    LowpassFilter speed_lpf;     // 速度低通滤波器
    LowpassFilter error_lpf;     // 误差低通滤波器
    AvgFilter speed_avg;     // 速度平均值滤波器
    AvgFilter adc_val_avg;     // ADC平均值滤波器
    const MotorPWM_Config *_motor_config; //电机PWM和编码器配置


    bool LPFAndPLL = true; //false 低通滤波 true 是PLL锁相环
    float theta_m_last; //电机上一次弧度
    float theta_m_offic;//角度差
    float theta_m_offic_filtered;// 角度差滤波
    float theta_m_speed;// 电机速度
    float theta_av_speed;// 电机平均速度
    float filtered_speed;// 速度滤波

    float _pll_reg_out = 0.0f; //PLL锁相环的输出角度
    float _pll_Angular_velocity = 0.0f; //PLL锁相环的输出角速度
    PLL_Parameter _pll_conf{FOC_PLL_KP, FOC_PLL_KI}; //PLL锁相环参数
};

extern DC_Motor M1, M2, M3, M4;



