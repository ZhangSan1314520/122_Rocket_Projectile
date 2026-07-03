#pragma once
#include "pid_Increment.hpp"
#include "pll.hpp"
#include "kth_read.hpp"
#include "ex_math.hpp"
#include "bsp.hpp"
#include "BL24C16F.hpp" //EEPROM
#include "my_dronecan.hpp"

enum Work_Mode
{
    speed = 0,//速度模式
    position = 1,//位置模式
    EncoderCalibration = 99,//编码器校准模式
    Normal_Mode = 3,// 正常模式
};


class DC_Motor
{
public:
    DC_Motor(const MotorPWM_Config *motor_config, KTH7111 *encoder)
        : _motor_config(motor_config), _encoder(encoder) {}//构造函数
    Work_Mode work_mode = Normal_Mode; //电机工作模式
    bool motor_encoder_dir = true;//编码器方向，true为正，false为反
    bool control_init_flag = false; //电机控制初始化标志
    uint16_t motor_freq = 10000;//电机频率
    float motor_up_duty = 0.0f;//电机上桥臂占空比
    float motor_down_duty = 0.0f;//电机下桥臂占空比

    KTH7111   *_encoder; //编码器
    

    void My_DC_Motor_Init(void);
    void Motor_EN(bool en) ;//电机使能
    void Set_Motor_Frequency(void); //设置电机频率和占空比
    int32_t laji; //用于记录中断间隔时间
private:
    uint16_t Fre_last = 0xff;   // 每个对象独立的上次频率 不能用 static
    float up_duty_last = -1.0f;   // 每个对象独立的上次占空比 不能用 static
    float down_duty_last = -1.0f;   // 每个对象独立的上次占空比 不能用 static

    uint32_t timer_clock_freq_; // 定时器的实际计数时钟频率
    LowpassFilter speed_lpf;     // 速度低通滤波器
    LowpassFilter error_lpf;     // 误差低通滤波器
    AvgFilter speed_avg;     // 速度平均值滤波器
    const MotorPWM_Config *_motor_config; //电机PWM和编码器配置
};

extern DC_Motor M1, M2, M3, M4;



