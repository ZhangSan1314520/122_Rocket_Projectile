#include "DcMotor_ClosedLoop.hpp"
#include "Motor_Param.hpp"


// 7.17周五更新LQR最佳参数3S响应稳定值
uint8_t DC_Motor::selected_motor = 0;// 静态成员定义
uint16_t DC_Motor::ADC_Value[max_chl_num] = {0};// 静态成员定义


KTH7111 kth7111_M1(&M1_PWM); //编码器对象，
KTH7111 kth7111_M2(&M2_PWM); //编码器对象
KTH7111 kth7111_M3(&M3_PWM); //编码器对象，
KTH7111 kth7111_M4(&M4_PWM); //编码器对象

PID_Increment pid_spd_M1
(
    MOTOR_DEFAULT_PARAM.speed_pid.kp,
    MOTOR_DEFAULT_PARAM.speed_pid.ki,
    MOTOR_DEFAULT_PARAM.speed_pid.kd,

    1.0/DC_VELOCITY_LOOP_FREQ_HZ,

    MOTOR_DEFAULT_PARAM.limit.max_duty,
    MOTOR_DEFAULT_PARAM.limit.min_duty
);

PID pid_loc_M1
(
    MOTOR_DEFAULT_PARAM.location_pid.kp,
    MOTOR_DEFAULT_PARAM.location_pid.ki,
    MOTOR_DEFAULT_PARAM.location_pid.kd,

    1.0/DC_POSITION_LOOP_FREQ_HZ,

    MOTOR_DEFAULT_PARAM.limit.i_ratio *
    MOTOR_DEFAULT_PARAM.limit.max_duty,

    MOTOR_DEFAULT_PARAM.limit.max_speed,
    MOTOR_DEFAULT_PARAM.limit.min_speed
);
LQR lqr_M1
(
    MOTOR_DEFAULT_PARAM.lqr.k1,
    MOTOR_DEFAULT_PARAM.lqr.k2,
    MOTOR_DEFAULT_PARAM.lqr.max_speed,
    MOTOR_DEFAULT_PARAM.lqr.min_speed
);




PID_Increment pid_spd_M2
(
    MOTOR_DEFAULT_PARAM.speed_pid.kp,
    MOTOR_DEFAULT_PARAM.speed_pid.ki,
    MOTOR_DEFAULT_PARAM.speed_pid.kd,

    1.0/DC_VELOCITY_LOOP_FREQ_HZ,

    MOTOR_DEFAULT_PARAM.limit.max_duty,
    MOTOR_DEFAULT_PARAM.limit.min_duty
);

PID pid_loc_M2
(
    MOTOR_DEFAULT_PARAM.location_pid.kp,
    MOTOR_DEFAULT_PARAM.location_pid.ki,
    MOTOR_DEFAULT_PARAM.location_pid.kd,

    1.0/DC_POSITION_LOOP_FREQ_HZ,

    MOTOR_DEFAULT_PARAM.limit.i_ratio *
    MOTOR_DEFAULT_PARAM.limit.max_duty,

    MOTOR_DEFAULT_PARAM.limit.max_speed,
    MOTOR_DEFAULT_PARAM.limit.min_speed
);

LQR lqr_M2
(
    MOTOR_DEFAULT_PARAM.lqr.k1,
    MOTOR_DEFAULT_PARAM.lqr.k2,
    MOTOR_DEFAULT_PARAM.lqr.max_speed,
    MOTOR_DEFAULT_PARAM.lqr.min_speed
);






PID_Increment pid_spd_M3
(
    MOTOR_DEFAULT_PARAM.speed_pid.kp,
    MOTOR_DEFAULT_PARAM.speed_pid.ki,
    MOTOR_DEFAULT_PARAM.speed_pid.kd,

    1.0/DC_VELOCITY_LOOP_FREQ_HZ,

    MOTOR_DEFAULT_PARAM.limit.max_duty,
    MOTOR_DEFAULT_PARAM.limit.min_duty
);

PID pid_loc_M3
(
    MOTOR_DEFAULT_PARAM.location_pid.kp,
    MOTOR_DEFAULT_PARAM.location_pid.ki,
    MOTOR_DEFAULT_PARAM.location_pid.kd,

    1.0/DC_POSITION_LOOP_FREQ_HZ,

    MOTOR_DEFAULT_PARAM.limit.i_ratio *
    MOTOR_DEFAULT_PARAM.limit.max_duty,

    MOTOR_DEFAULT_PARAM.limit.max_speed,
    MOTOR_DEFAULT_PARAM.limit.min_speed
);

LQR lqr_M3
(
    MOTOR_DEFAULT_PARAM.lqr.k1,
    MOTOR_DEFAULT_PARAM.lqr.k2,
    MOTOR_DEFAULT_PARAM.lqr.max_speed,
    MOTOR_DEFAULT_PARAM.lqr.min_speed
);





PID_Increment pid_spd_M4
(
    MOTOR_DEFAULT_PARAM.speed_pid.kp,
    MOTOR_DEFAULT_PARAM.speed_pid.ki,
    MOTOR_DEFAULT_PARAM.speed_pid.kd,

    1.0/DC_VELOCITY_LOOP_FREQ_HZ,

    MOTOR_DEFAULT_PARAM.limit.max_duty,
    MOTOR_DEFAULT_PARAM.limit.min_duty
);

PID pid_loc_M4
(
    MOTOR_DEFAULT_PARAM.location_pid.kp,
    MOTOR_DEFAULT_PARAM.location_pid.ki,
    MOTOR_DEFAULT_PARAM.location_pid.kd,

    1.0/DC_POSITION_LOOP_FREQ_HZ,

    MOTOR_DEFAULT_PARAM.limit.i_ratio *
    MOTOR_DEFAULT_PARAM.limit.max_duty,

    MOTOR_DEFAULT_PARAM.limit.max_speed,
    MOTOR_DEFAULT_PARAM.limit.min_speed
);

LQR lqr_M4
(
    MOTOR_DEFAULT_PARAM.lqr.k1,
    MOTOR_DEFAULT_PARAM.lqr.k2,
    MOTOR_DEFAULT_PARAM.lqr.max_speed,
    MOTOR_DEFAULT_PARAM.lqr.min_speed
);



DC_Motor M1(&M1_PWM, &kth7111_M1, &pid_spd_M1, &pid_loc_M1,&lqr_M1); //电机1
DC_Motor M2(&M2_PWM, &kth7111_M2, &pid_spd_M2, &pid_loc_M2,&lqr_M2); //电机2
DC_Motor M3(&M3_PWM, &kth7111_M3, &pid_spd_M3, &pid_loc_M3,&lqr_M3); //电机3
DC_Motor M4(&M4_PWM, &kth7111_M4, &pid_spd_M4, &pid_loc_M4,&lqr_M4); //电机4



void DC_Motor::test_laji()
{
   
    __HAL_TIM_SET_AUTORELOAD(&htim1, 100);//设置周期 
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);//设置上桥臂占空比 
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);//设置下桥臂占空比
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}

void DC_Motor::My_DC_Motor_Reset()
{
    _target_location2 = rad2deg(reg_final); //复位位置为当前位置
    _target_speed = 0.0;
    updown_duty = 0.0;
    Motor_EN(false);
    pid_speed_incparam->reset(); //复位pid
    pid_location->reset(); //复位pid
}




void DC_Motor::My_DC_Motor_Init()
{
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    timer_clock_freq_ = sysclk / (_motor_config->htim->Instance->PSC + 1);

    speed_lpf.init(0.7);
    error_lpf.init(0.7);
    speed_avg.init(5);
    adc_val_avg.init(100);//ADC采样值滤波 //67极限

    // ===== ① 设频率和占空比（0% 用于充电） =====
    Set_Motor_Glo_Duty();//设置电机全局占空比

    // ===== ② 自举电容预充电：CCR=0 → 下管导通 → 电容充电 =====
    Motor_EN(true);  // 打开PWM输出
    vTaskDelay(pdMS_TO_TICKS(1));  // 等 1ms 充满
    Motor_EN(false); // 关闭PWM输出
    _encoder->KTH7111_Init(motor_encoder_dir);
    control_init_flag = true;
}



void DC_Motor::Set_Motor_Frequency() //单独设置电机频率和占空比
{
    bool new_dir = false;
    uint32_t arr = 0;
    uint16_t Fre = (uint16_t)motor_freq;//转整数计算
    float up_duty = constraint_value(motor_up_duty, 0.0f, MOTOR_DEFAULT_PARAM.limit.max_duty); //限制占空比在0~1之间
    float down_duty = constraint_value(motor_down_duty, 0.0f, MOTOR_DEFAULT_PARAM.limit.max_duty); //限制占空比在0~1之间

    if (Fre == Fre_last && up_duty == up_duty_last && down_duty == down_duty_last) return;// 如果频率和占空比两个都没有变化，则直接返回
    Fre_last = Fre;
    up_duty_last = up_duty;
    down_duty_last = down_duty;

    if (Fre == 0 ){Motor_EN(false); return;}
    else {Motor_EN(true);}

    arr = timer_clock_freq_ / Fre;
    if (arr >= 65535) arr = 65535;
    else if (arr <= 1) arr = 1;

    _motor_config->htim->Instance->CNT = 0; // 清零计数器
    __HAL_TIM_SET_AUTORELOAD(_motor_config->htim, (uint16_t)(arr - 1));//设置周期 
    __HAL_TIM_SET_COMPARE(_motor_config->htim, _motor_config->ch_hin1, (uint32_t)(arr*up_duty));//设置上桥臂占空比 
    __HAL_TIM_SET_COMPARE(_motor_config->htim, _motor_config->ch_hin2, (uint32_t)(arr*down_duty));//设置下桥臂占空比
}



void DC_Motor::Set_Motor_Glo_Duty() //设置电机全局占空比
{
    float duty = constraint_value(updown_duty, MOTOR_DEFAULT_PARAM.limit.min_duty, MOTOR_DEFAULT_PARAM.limit.max_duty);

    if (duty == 0.0f || fabsf(duty) < 1e-6f)
    {
        motor_up_duty = motor_down_duty = 0.0f; //电流极大 ?????
        Motor_EN(false);
    }else if (duty > 0.0f) 
    {
        motor_up_duty = duty;
        motor_down_duty = 0.0f;
        Motor_EN(true);
    }else if (duty < 0.0f) 
    {
        motor_up_duty = 0.0f;
        motor_down_duty = -duty;
        Motor_EN(true);
    }

    Set_Motor_Frequency();
}






void DC_Motor::Motor_EN(bool en)
{
    if (en == _motor_is_on_last) return;  // 状态没变，跳过
    _motor_is_on_last = en;

    if (en)
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


void DC_Motor::Update_Speed_Angle_LPFAndPLL() //更新速度和角度
{
    // 静态变量，用于临时存储角度值和上一次的角度值
    static float fudu_test = 0.0f;
    static float theta_m_offic_temp = 0.0;

    _encoder->KTH7111_Send_JIAO_CMD();//发送角度指令
    fudu_test = _encoder->Get_KTH7111_Radian();//获取编码器角度

    if (LPFAndPLL == false) //低通滤波角度
    {
      theta_m = wrap_to_PI(fudu_test);
      //方式1 低通滤波 更新机械角度速度
      theta_m_offic_temp = theta_m - theta_m_last; 
      theta_m_last = theta_m; // 更新上一次的机械角度
      theta_m_offic = wrap_to_PI(theta_m_offic_temp);// 将差值规整到 [-PI, PI) 区间

      theta_m_offic_filtered = error_lpf.filter(theta_m_offic);//对误差角度低通滤波
      theta_m_speed = theta_m_offic_filtered*DC_VELOCITY_LOOP_FREQ_HZ;//speed=路程/t 
      theta_av_speed = speed_avg.filter(theta_m_speed);//对速度进行平均值滤波
      filtered_speed = speed_lpf.filter(theta_av_speed);//对速度进行低通滤波
      reg_final = theta_m; //将机械角度赋值给最终角度
      theta_deg_final = rad2deg(reg_final); //将弧度转换为360度
      Angular_velocity_final = filtered_speed;  //将最终速度赋值给最终角速度
      
    }else if (LPFAndPLL == true)
    {
      // 方式2PLL锁相环 更新PLL锁相环的输出角度和角速度
      theta_m = fudu_test; // 获取原始机械角度 弧度
      foc_pll_run(theta_m,PLL_FREQ_Dt,&_pll_reg_out,&_pll_Angular_velocity,&_pll_conf); //更新PLL锁相环的输出角度和角速度
      reg_final = _pll_reg_out; //将PLL锁相环的输出角度赋值给最终角度
      theta_deg_final = rad2deg(reg_final); //将弧度转换为360度
      Angular_velocity_final = _pll_Angular_velocity; //将最终速度赋值给最终角速度
    }

}


void DC_Motor::Speed_Loop(void) //速度环
{
    float _delta_pwm = 0.0f; //计算PID增量
    float error_speed = _target_speed - Angular_velocity_final; //计算速度误差

    _delta_pwm = pid_speed_incparam->update(error_speed); //计算PID增量
    updown_duty += _delta_pwm; //更新电机占空比
    Set_Motor_Glo_Duty(); //设置电机全局占空比
}


// void DC_Motor::Location_Loop(void) //位置环
// {
//     float error = wrap_to_PI(deg2rad(_target_location2) - reg_final); //计算位置误差
//     _target_speed = pid_location->update(error); //传入实际误差值
// }



// void DC_Motor::Location_Loop(void)
// {
//     float diff = _target_location2_cmd - _target_location2;

//     while (diff > 180.0f) diff -= 360.0f;    // 改成 while，确保 diff 在 [-180, 180]
//     while (diff < -180.0f) diff += 360.0f;

//     if (diff > 60.0f) diff = 60.0f;
//     if (diff < -60.0f) diff = -60.0f;
//     _target_location2 += diff;

//     while (_target_location2 > 180.0f) _target_location2 -= 360.0f;
//     while (_target_location2 < -180.0f) _target_location2 += 360.0f;

//     float error = wrap_to_PI(deg2rad(_target_location2) - reg_final);
//     float raw_speed = pid_location->update(error);
//     _target_speed = raw_speed;

//     if (_target_speed > PID_Max_Speed) _target_speed = PID_Max_Speed;
//     if (_target_speed < PID_Min_Speed) _target_speed = PID_Min_Speed;
// }





void DC_Motor::Location_Loop(void)
{
    float pos_error = wrap_to_PI(deg2rad(_target_location2) - reg_final);  // 计算当前位置误差

    float pos_abs = fabs(pos_error); // 误差位置绝对值

    float vel_abs = fabs(Angular_velocity_final); //真实速度绝对值


    if(position_done) // 位置环到位
    {
        if(pos_abs > deg2rad(0.6f))
        {
            position_done = false;
        }
    }else // 位置环未到位
    {
        if(pos_abs < deg2rad(0.3f) &&vel_abs < deg2rad(4.0f))
        {
            position_ok_cnt++;//连续10次满足条件，认为位置到位
            if(position_ok_cnt >= 10)
            {
                position_done = true;
            }
        }else
        {
            position_ok_cnt = 0;
        }
    }


    if(position_done)  // 位置环到位，停止输出速度
    {
        _target_speed = 0.0f;
        target_speed_last = 0.0f;
        return;
    }

    float vel_error = -Angular_velocity_final;  //计算速度误差
    _target_speed = _lqr->update(pos_error, vel_error);  // LQR输出目标速度
    //速度变化斜坡
    float lqr_speed = _lqr->update(pos_error, vel_error);  // LQR输出目标速度
    float speed_delta = lqr_speed - target_speed_last;
    if(speed_delta > 0.002f)
    {
        speed_delta = 0.002f;
    }
    else if(speed_delta < -0.002f)
    {
        speed_delta = -0.002f;
    }

    target_speed_last += speed_delta;

    _target_speed = target_speed_last;

}





void DC_Motor::Thermistor_Temp() //计算热敏电阻温度
{
    float adc_av_val = 0.0f;
    
    adc_av_val = adc_val_avg.filter((float)ADC_Value[_motor_config->adc_channel]); //对ADC值进行平均值滤波
    // ADC 值异常（悬空/未接热敏电阻 跳过计算
    if (adc_av_val < 10.0f || adc_av_val > 4086.0f) return;  // 保持上次有效温度，不更新

    ADC_RES_Val = 10.0*(4096-adc_av_val) / adc_av_val; //计算ADC值对应的电阻值大小

    float inv_T = 1.0f / T0_K + log(ADC_RES_Val / NTC_R0) / NTC_B; //计算温度的倒数 单位开尔文
    
    ADC_Temp_Val = (1.0f / inv_T) - T0;  //转换为摄氏度温度
}


