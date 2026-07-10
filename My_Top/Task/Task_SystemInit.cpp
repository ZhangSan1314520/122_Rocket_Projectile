
#include "FreeRTOS.h"
#include "task.h"
#include "ex_math.hpp"
#include "HAL_System.hpp"
#include "MC_Serial.hpp"
#include "DcMotor_ClosedLoop.hpp"
#include "My_Vofa.hpp"
#include "uorb.hpp"
#include "my_tim.hpp"
#include "Motor_Storage.hpp"
#include "usb_device.h"


void Task_SystemInit(void *argument)
{

    MX_USB_Device_Init(); //初始化USB设备
    uorb_init(); //初始化uORB消息中间件
    My_Canard_Init(); //初始化Canard协议栈
    HAL_System::init(); //初始化计时器 
    MC_Serial::init(); //初始化串口
    Vofa_Init(); //初始化Vofa 
    My_Tim_Init(closed_loop_htim,therm_hadc); //初始化定时器绑定中断
    M1.My_DC_Motor_Init(); //初始化电机1
    M2.My_DC_Motor_Init(); //初始化电机2
    M3.My_DC_Motor_Init(); //初始化电机3
    M4.My_DC_Motor_Init(); //初始化电机4

    Motor_Storage::load(1);//读eeprom的数据传回给电机




    printf("Sys_init栈剩余:%d\r\n", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelete(NULL); //删除任务
}





