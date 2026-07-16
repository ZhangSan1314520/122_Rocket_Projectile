#include "FreeRTOS.h"
#include "task.h"
#include "uorb.hpp"
#include "task_topics.h"
#include "DcMotor_ClosedLoop.hpp"

static void fill_telem(motor_telem_t *dst, DC_Motor *m) // 电机数据填充转存
{

    // 实际位置/速度
    dst->theta_m = m->theta_m;                //电机原始弧度
    dst->theta_m_speed = m->theta_m_speed;        //电机原始弧度速度
    dst->theta_deg_final = m->theta_deg_final;        //最终的电机角度 (度)
    dst->reg_final = m->reg_final;              //最终的电机角度 (弧度)
    dst->Angular_velocity_final = m->Angular_velocity_final; //最终的角速度

    // 目标位置/速度
    dst->_target_location2 = m->_target_location2;      //目标位置 (度)
    dst->_target_location2_cmd = m->_target_location2_cmd;
    dst->_target_speed = m->_target_speed;          //目标速度
 
    //热敏电阻
    dst->ADC_Temp_Val = m->ADC_Temp_Val;           //热敏电阻温度
 
    //调试信息
    dst->updown_duty = m->updown_duty;     //电机全局占空比

}

void Task_MotorPublish(void *argument)
{
    uorb_handle_t topic = uorb_advertise(UORB_TOPIC_MOTOR, sizeof(motor_telem_t) * 4); //注册发布者
    motor_telem_t buf[4];

    while (1)
    {
        fill_telem(&buf[0], &M1);
        fill_telem(&buf[1], &M2);
        fill_telem(&buf[2], &M3);
        fill_telem(&buf[3], &M4);

        uorb_publish(topic, buf);//发布数据 (没有指定给谁发，放在公共区域里)
        vTaskDelay(pdMS_TO_TICKS(10));  //
    }
}


