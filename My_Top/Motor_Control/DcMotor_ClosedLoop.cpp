#include "DcMotor_ClosedLoop.hpp"

KTH7111 kth7111_M1(&M1_PWM); //编码器对象，
KTH7111 kth7111_M2(&M2_PWM); //编码器对象
KTH7111 kth7111_M3(&M3_PWM); //编码器对象，
KTH7111 kth7111_M4(&M4_PWM); //编码器对象

DC_Motor M1(&M1_PWM, &kth7111_M1); //电机1
DC_Motor M2(&M2_PWM, &kth7111_M2); //电机2
DC_Motor M3(&M3_PWM, &kth7111_M3); //电机3
DC_Motor M4(&M4_PWM, &kth7111_M4); //电机4

void DC_Motor::My_DC_Motor_Init()
{
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    timer_clock_freq_ = sysclk / (_motor_config->htim->Instance->PSC + 1);

    speed_lpf.init(0.7);
    error_lpf.init(0.7);
    speed_avg.init(5);

    // ===== ① 设频率和占空比（0% 用于充电） =====
    Set_Motor_Frequency();

    // ===== ② 自举电容预充电：CCR=0 → 下管导通 → 电容充电 =====
    Motor_EN(true);  // 打开PWM输出
    vTaskDelay(pdMS_TO_TICKS(1));  // 等 1ms 充满
    Motor_EN(false); // 关闭PWM输出

    _encoder->KTH7111_Init(motor_encoder_dir);
    control_init_flag = true;
}



void DC_Motor::Set_Motor_Frequency() //设置电机频率和占空比
{
    bool new_dir = false;
    uint32_t arr = 0;
    uint16_t Fre = (uint16_t)motor_freq;//转整数计算
    float up_duty = constraint_value(motor_up_duty, 0.0f, 1.0f); //限制占空比在0~1之间
    float down_duty = constraint_value(motor_down_duty, 0.0f, 1.0f); //限制占空比在0~1之间

    if (Fre == Fre_last && up_duty == up_duty_last && down_duty == down_duty_last) return;// 如果频率和占空比两个都没有变化，则直接返回
    Fre_last = Fre;
    up_duty_last = up_duty;
    down_duty_last = down_duty;

    if (Fre == 0) //频率0直接关电机
    {
        Motor_EN(false);
        return;
    }

    arr = timer_clock_freq_ / Fre;
    if (arr >= 65535) arr = 65535;
    else if (arr <= 1) arr = 1;

    _motor_config->htim->Instance->CNT = 0; // 清零计数器
    __HAL_TIM_SET_AUTORELOAD(_motor_config->htim, (uint16_t)(arr - 1));//设置周期 
    __HAL_TIM_SET_COMPARE(_motor_config->htim, _motor_config->ch_hin1, (uint32_t)(arr*up_duty));//设置上桥臂占空比 
    __HAL_TIM_SET_COMPARE(_motor_config->htim, _motor_config->ch_hin2, (uint32_t)(arr*down_duty));//设置下桥臂占空比
}




void DC_Motor::Motor_EN(bool en) 
{
    if (en == true)
    {
        HAL_TIM_PWM_Start(_motor_config->htim, _motor_config->ch_hin1);
        HAL_TIMEx_PWMN_Start(_motor_config->htim, _motor_config->ch_hin1);
        HAL_TIM_PWM_Start(_motor_config->htim, _motor_config->ch_hin2);
        HAL_TIMEx_PWMN_Start(_motor_config->htim, _motor_config->ch_hin2);
    }else
    {
        HAL_TIM_PWM_Stop(_motor_config->htim, _motor_config->ch_hin1);
        HAL_TIMEx_PWMN_Stop(_motor_config->htim, _motor_config->ch_hin1);
        HAL_TIM_PWM_Stop(_motor_config->htim, _motor_config->ch_hin2);
        HAL_TIMEx_PWMN_Stop(_motor_config->htim, _motor_config->ch_hin2);
    }
}

