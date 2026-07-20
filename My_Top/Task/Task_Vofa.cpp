
#include "FreeRTOS.h"
#include "task.h"
#include "My_Vofa.hpp"
#include "MC_Serial.hpp"
#include "DcMotor_ClosedLoop.hpp"
#include "uorb.hpp"
#include "task_topics.h"


struct Vafa_data
{ // 解析后的参数值
    uint8_t len;
    float val[VOFA_PARAM_MAX];
    float last_val[VOFA_PARAM_MAX]; // 记录上次的值
    bool changed[VOFA_PARAM_MAX];   // 标记哪个变了
};

/* 同步 last_val = val，用于初始化或写入后 */
void vofa_sync_last(Vafa_data *d)
{
    for (uint8_t i = 0; i < d->len; i++)
    {
        d->last_val[i] = d->val[i];
        d->changed[i] = false;
    }
}

const char *IA_Names[] = {"iq_p", "iq_i", "id_p", "id_i"};
const char *Speed_Names[] = {"speed_p", "speed_i", "speed_d"};
const char *Target_Names[] = {"Target_Iq", "Target_Id", "Target_speed", "Target_location1", "Target_location2"};
const char *Location_Names[] = {"location_p", "location_i", "location_d"};
const char *Button_Names[] = {"motor_selection", "motor_fre", "motor_duty"}; // 电机选择12 / 电机频率 / 电机占空比

Vafa_data Vafa_IA;
Vafa_data Vafa_Speed;
Vafa_data Vafa_Target;
Vafa_data Vafa_Location;
Vafa_data Vafa_Button;

void vofa_data_init()
{
    Vafa_IA.len = sizeof(IA_Names) / sizeof(IA_Names[0]);
    Vafa_Speed.len = sizeof(Speed_Names) / sizeof(IA_Names[0]);
    Vafa_Target.len = sizeof(Target_Names) / sizeof(Target_Names[0]);
    Vafa_Location.len = sizeof(Location_Names) / sizeof(Location_Names[0]);
    Vafa_Button.len = sizeof(Button_Names) / sizeof(Button_Names[0]);
    // 第一次读取后同步 last_val，避免误判 */
    vofa1.vofa_get_batch(&vofa1, IA_Names, Vafa_IA.val, Vafa_IA.len); // 读到的都是0
    vofa1.vofa_get_batch(&vofa1, Speed_Names, Vafa_Speed.val, Vafa_Speed.len);
    vofa1.vofa_get_batch(&vofa1, Target_Names, Vafa_Target.val, Vafa_Target.len);
    vofa1.vofa_get_batch(&vofa1, Location_Names, Vafa_Location.val, Vafa_Location.len);
    vofa1.vofa_get_batch(&vofa1, Button_Names, Vafa_Button.val, Vafa_Button.len);

    vofa_sync_last(&Vafa_IA);
    vofa_sync_last(&Vafa_Speed);
    vofa_sync_last(&Vafa_Target);
    vofa_sync_last(&Vafa_Location);
    vofa_sync_last(&Vafa_Button);
}

/* 模式切换专用函数 */
void vofa_update_mode(Work_Mode *target, Work_Mode default_val , DC_Motor* motor) // 只在值变了的时候，写入目标到target
{
    static Work_Mode last_mode = (Work_Mode)0; 
    float val = vofa1.get("work_mode", (float)default_val);
    Work_Mode new_mode = (Work_Mode)((int)val);

    if (new_mode != last_mode)
    {
        *target = new_mode;
        last_mode = new_mode;
        if (motor != NULL)
        {
            motor->My_DC_Motor_Reset();
        }
    }
}



void vofa_update_if_changed(Vafa_data *d, uint8_t idx, float *target,  DC_Motor* motor)
{
    if (!d->changed[idx]) return;// 只在值变了的时候

    *target = d->val[idx];      // 写入目标

    d->last_val[idx] = d->val[idx]; // 更新last
    d->changed[idx] = false; // 标记无变化

    if (motor == NULL) return; // 如果没有指定电机，不复位
    motor->My_DC_Motor_Reset();

}




void vofa_detect_changes(Vafa_data *d) // 检测数据的变化 写入标志位到changed
{
    for (uint8_t i = 0; i < d->len; i++)
    {
        d->changed[i] = (d->val[i] != d->last_val[i]);
    }
}

void Task_VofaRx(void *argument)
{ 
    static float Motor_Select = 0.0; // 电机选择1-4
    static uint8_t last_idx = 0; // 上一次的电机选择
    VofaRxFrame_t frame;   // 队列接收缓冲区

    vofa_data_init();
    
    while (1)
    {
        xQueueReceive(vofaRxQueue, &frame, portMAX_DELAY); // 从队列中获取数据
        /* 把队列数据喂给 vofa1（复用原有解析逻辑） */
        memcpy(vofa1.rx_buf, frame.buf, frame.len);
        vofa1.rx_len = frame.len;
        vofa1.parse();// 解析收到的帧

        vofa1.vofa_get_batch(&vofa1, IA_Names, Vafa_IA.val, Vafa_IA.len); // 读取IA_Names参数值
        vofa1.vofa_get_batch(&vofa1, Speed_Names, Vafa_Speed.val, Vafa_Speed.len); // 读取Speed_Names参数值
        vofa1.vofa_get_batch(&vofa1, Target_Names, Vafa_Target.val, Vafa_Target.len); // 读取Target_Names参数值
        vofa1.vofa_get_batch(&vofa1, Location_Names, Vafa_Location.val, Vafa_Location.len); // 读取Location_Names参数值
        vofa1.vofa_get_batch(&vofa1, Button_Names, Vafa_Button.val, Vafa_Button.len);       // 读取Button_Names参数值

        vofa_detect_changes(&Vafa_IA); // 检测数据的变化 写入标志位到changed
        vofa_detect_changes(&Vafa_Speed);
        vofa_detect_changes(&Vafa_Target);
        vofa_detect_changes(&Vafa_Location);
        vofa_detect_changes(&Vafa_Button);

        
        vofa_update_if_changed(&Vafa_Button, 0, &Motor_Select, 0);
        
        uint8_t idx = (uint8_t)Motor_Select;  // 1=M1, 2=M2, 3=M3, 4=M4
        DC_Motor::selected_motor = idx;//记录当前选择的电机
        if (idx != last_idx) //电机切换后           
        {                                   
            vofa_sync_last(&Vafa_IA);  //同步last_val = val且标记无变化 
            vofa_sync_last(&Vafa_Speed); 
            vofa_sync_last(&Vafa_Target);
            vofa_sync_last(&Vafa_Location); 
            vofa_sync_last(&Vafa_Button);
            last_idx = idx;
        }
        if (idx < 1 || idx > 4)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;                        // 跳过本轮，不退出任务
        }

        DC_Motor* m = NULL;
        switch (idx)
        {
            case 1: m = &M1; break;
            case 2: m = &M2; break;
            case 3: m = &M3; break;
            case 4: m = &M4; break;
        }
        switch (m->work_mode) 
        {
            case open_loop: // 开环模式
                vofa_update_if_changed(&Vafa_Button, 1, &m->motor_freq, NULL); // 电机频率
                vofa_update_if_changed(&Vafa_Button, 2, &m->updown_duty, NULL); // 电机全局占空比
                break;
            case speed: // 速度闭环模式
                vofa_update_if_changed(&Vafa_Speed,  0, &m->pid_speed_incparam->_kp, m);//速度PID
                vofa_update_if_changed(&Vafa_Speed,  1, &m->pid_speed_incparam->_ki, m);
                vofa_update_if_changed(&Vafa_Speed,  2, &m->pid_speed_incparam->_kd, m);
                vofa_update_if_changed(&Vafa_Target,  2, &m->_target_speed, NULL); //目标速度
                break;
            case position: // 位置闭环模式
                vofa_update_if_changed(&Vafa_Location, 0, &m->pid_location->_kp, m);//位置PID
                vofa_update_if_changed(&Vafa_Location, 1, &m->pid_location->_ki, m);
                vofa_update_if_changed(&Vafa_Location, 2, &m->pid_location->_kd, m);
                vofa_update_if_changed(&Vafa_Target,  4, &m->_target_location2, NULL); //目标位置-180°~+180°
                vofa_update_if_changed(&Vafa_Speed,  0, &m->pid_speed_incparam->_kp, m);//速度PID
                vofa_update_if_changed(&Vafa_Speed,  1, &m->pid_speed_incparam->_ki, m);
                vofa_update_if_changed(&Vafa_Speed,  2, &m->pid_speed_incparam->_kd, m);   
                
                vofa_update_if_changed(&Vafa_Target,  0, &m->_lqr->_k1, NULL); //
                vofa_update_if_changed(&Vafa_Target,  1, &m->_lqr->_k2, NULL); //
            
                break;
        }
        vofa_update_mode(&m->work_mode, open_loop, m);        

    }
}





// ===== Vofa 发送任务 =====
void Task_VofaTx(void *argument)
{
    uorb_handle_t topic = uorb_subscribe(UORB_TOPIC_MOTOR); // 订阅motor_state话题
    motor_telem_t buf[4];

    while (1)
    {
        uorb_copy(topic, buf);  // 阻塞等新数据

        // Vofa_SendFireWater_VA(21,
        //     // M1
        //     buf[0].theta_m, buf[0].theta_deg_final,buf[0].Angular_velocity_final, 
        //     buf[0]._target_location2, buf[0]._target_speed,999.0f,
        //     // M2
        //     buf[1].theta_m, buf[1].theta_deg_final,buf[1].Angular_velocity_final,
        //     buf[1]._target_location2, buf[1]._target_speed,999.0f,
        //     // M3
        //     buf[2].theta_m, buf[2].theta_deg_final,buf[2].Angular_velocity_final,
        //     buf[2]._target_location2, buf[2]._target_speed,999.0f,
        //     // M4
        //     buf[3].theta_m, buf[3].theta_deg_final,buf[3].Angular_velocity_final,
        //     buf[3]._target_location2, buf[3]._target_speed,999.0f

        // );

        Vofa_SendFireWater_VA(5,
            // M1
            buf[1]._target_speed, buf[1].Angular_velocity_final,
            buf[1]._target_location2, rad2deg(buf[1].reg_final),
            buf[1].updown_duty 
                                        
        );  

    }
}





// Vofa_SendFireWater_VA(4,
//     // M1
//     buf[0].ADC_Temp_Val, buf[1].ADC_Temp_Val, 
//     buf[2].ADC_Temp_Val, buf[3].ADC_Temp_Val

// );    

