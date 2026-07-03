#include "my_tim.hpp"

uint16_t ADC_Value[16] = {0}; //ADC采集值数组
/**
 * @brief 定时器回调函数，用于执行FOC（磁场定向控制）相关任务
 * @param htim 定时器句柄指针，包含定时器相关信息
 */
void My_Tim_Callback(TIM_HandleTypeDef *htim)
{
    static uint32_t start_time, end_time = 0;//求计算时间

    static uint32_t last_time = 0;//求中断间隔时间
    uint32_t now_time = 0;

    static uint16_t count = 0;

    if (M1.control_init_flag == false) return;// 初始化未完成时，不执行

    if ((htim->Instance->CR1 & 0x10) == 0x00U) // 向上计数时，请求编码器位置信息
    {
        if (count % DC_VELOCITY_LOOP_FREQ_DIV == 1)// 每4个周期执行一次（4k频率）
        {
            // start_time = HAL_System::get_tick_us(); //记录开始时间
            // M1.Update_Speed_Angle_LPFAndPLL();//更新速度角度 耗时
            // end_time = HAL_System::get_tick_us(); // 记录结束时间
            // M1.laji = (float)(end_time - start_time); 

        }
        if (count % 160 == 1)// 每160个周期执行一次（0.1k频率）
        {
            HAL_ADC_Start_DMA(therm_hadc,(uint32_t *)ADC_Value,4); //电池电压采集
        }
        count = (count%10000)+1;

    }
}


void My_Tim_OC_Callback(TIM_HandleTypeDef *htim) // TIM3_CH1 的输出比较中断
{
    static uint16_t count = 0;
    static uint32_t start_time = 0, end_time = 0; // 计时变量

    if (M1.control_init_flag == false) return;// 初始化未完成时，不执行
    M1.Set_Motor_Frequency(); //设置电机频率
    count = (count%10000)+1;

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




