#include "my_tim.hpp"




void My_Tim_OC_Callback(TIM_HandleTypeDef *htim)
{
    static uint16_t count = 0;
    static uint32_t start_time, end_time = 0;//求计算时间
    if(M1.control_init_flag == false) return;// 初始化未完成时，不执行
    if(M2.control_init_flag == false) return;// 初始化未完成时，不执行
    if(M3.control_init_flag == false) return;// 初始化未完成时，不执行
    if(M4.control_init_flag == false) return;// 初始化未完成时，不执行

    if (count % 16 == 1)
    {
        HAL_ADC_Start_DMA(therm_hadc,(uint32_t *)DC_Motor::ADC_Value,4); //电池电压采集

        M1.Thermistor_Temp();
        M2.Thermistor_Temp();
        M3.Thermistor_Temp();
        M4.Thermistor_Temp();
    }

    count = (count % 16000) + 1;
}


void My_Tim_Callback(TIM_HandleTypeDef *htim)
{
    static uint16_t count = 0;
    static uint32_t start_time, end_time = 0;//求计算时间
    if ((htim->Instance->CR1 & 0x10) != 0x00U) return; //不是向上计数时，不执行

    DC_Motor* motors[] = { &M1, &M2, &M3, &M4 };

    for (int i = 0; i < 4; i++)
    {
        DC_Motor* m = motors[i];
        if (!m->control_init_flag) continue;

        if (count % 40 == 1)
            m->Update_Speed_Angle_LPFAndPLL();

        switch (m->work_mode)
        {
        case open_loop: // 开环控制
            if (count % DC_Open_Loop_FREQ_DIV == 1) 
            {
                // start_time = HAL_System::get_tick_us(); //记录开始时间
                m->Set_Motor_Glo_Duty(); //耗时10us*4 = 40us
                // end_time = HAL_System::get_tick_us(); // 记录结束时间
                // m->laji = (float)(end_time - start_time);  // 计算时间差
            }
            break;

        case speed: // 速度控制
            if (count % DC_VELOCITY_LOOP_FREQ_DIV == 1)
            {
                // start_time = HAL_System::get_tick_us(); //记录开始时间
                m->Speed_Loop();//耗时2us*4 = 8us
                // end_time = HAL_System::get_tick_us(); // 记录结束时间
                // m->laji = (float)(end_time - start_time);  // 计算时间差
            }
            break;

        case position: // 位置控制 
            if (count % DC_POSITION_LOOP_FREQ_DIV == 1)
            {
                // start_time = HAL_System::get_tick_us(); //记录开始时间
                m->Location_Loop();//耗时4us*4 = 16us
                // end_time = HAL_System::get_tick_us(); // 记录结束时间
                // m->laji = (float)(end_time - start_time);  // 计算时间差
            }
            break;

        case EncoderCalibration: //编码器校准模式
            break;
        }
    }

    count = (count % 16000) + 1;
}








/*
void laji()
{
    static uint32_t last_time = 0;
    uint32_t now_time = 0;
    now_time =  HAL_System::get_tick_us();
    if (last_time != 0) {
         M1.laji = (int32_t)(now_time - last_time);  // 单次中断间隔，单位us
    }
    last_time = now_time;
}
*/

/**
 * @brief 初始化定时器并设置回调函数
 * @param htim 定时器句柄指针，指向定时器配置结构体
 */

void My_Tim_Init(TIM_HandleTypeDef* htim,ADC_HandleTypeDef *hadc)
{
    uint16_t duty = __HAL_TIM_GET_AUTORELOAD(htim)*0.5f;// 获取定时器ARR寄存器的值
    
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, duty); // 设置占空比为50% 控制My_Tim_OC_Callback回调函数的触发时机
    HAL_TIM_Base_Start_IT(htim); //启动定时器周期中断
    HAL_TIM_OC_Start_IT(htim, TIM_CHANNEL_1); //启动定时器比较中断
    htim->PeriodElapsedCallback = My_Tim_Callback;  // 设置定时器周期结束回调函数
    htim->PWM_PulseFinishedCallback = My_Tim_OC_Callback;// PWM的模式的比较匹配回调函数
}




