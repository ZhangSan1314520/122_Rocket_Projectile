
#include "FreeRTOS.h"
#include "task.h"
#include "MC_Serial.hpp"
#include "My_Vofa.hpp"
#include "uorb.hpp"
#include "bsp.hpp"
#include "DcMotor_ClosedLoop.hpp"
#include "My_HID_Keyboard.hpp"


extern TaskHandle_t SystemInitHandle;
extern TaskHandle_t TestHandle;
extern TaskHandle_t Task_VofaRxHandle;
extern TaskHandle_t DroneCANHandle;
extern TaskHandle_t MotorPubHandle;
extern TaskHandle_t CLIHandle;
extern TaskHandle_t VofaTxHandle;

void look_mem(void)
{
    printf("历史最低栈剩余 Sys:%d VofaRx:%d CLI:%d Can:%d Pub:%d VofaTx:%d 自己:%d 堆剩余:%d / %d\r\n",
           uxTaskGetStackHighWaterMark(SystemInitHandle),
           uxTaskGetStackHighWaterMark(Task_VofaRxHandle),
           uxTaskGetStackHighWaterMark(CLIHandle),
           uxTaskGetStackHighWaterMark(DroneCANHandle),
           uxTaskGetStackHighWaterMark(MotorPubHandle),
           uxTaskGetStackHighWaterMark(VofaTxHandle),
           uxTaskGetStackHighWaterMark(NULL), // 自己的任务栈
           xPortGetFreeHeapSize(), configTOTAL_HEAP_SIZE);
}

XBLW24C64 EEPROM1(EEPROM_hi2c);
uint8_t add[]= {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0d,0x10,0x11,0x12,0x16,0x72};
uint8_t data_tx[]= {0x00,0x00,0x00,0xff,0x03,0x00,0x07,0x00,0x00,0xa5,0xa5,0x88,0x00,0xaC,0x02,0x08,0x00};

uint8_t len = sizeof(data_tx)/sizeof(data_tx[0]);
uint8_t data_rx1[20], data_rx2[20], data_rx3[20], data_rx4[20],data_rx5[20];

void Task_Test(void *argument)
{
    printf("1-任务启动中\r\n");
    // M1._encoder->KTH7111_Read_MoreRegs(add, data_rx, len); //读取多个寄存器
    vTaskDelay(pdMS_TO_TICKS(1000));
    


    // M1._encoder->KTH7111_Write_MoreRegs(add, data_tx, len,true); //写入多个寄存器
    // M2._encoder->KTH7111_Write_MoreRegs(add, data_tx, len,true); //写入多个寄存器
    // M3._encoder->KTH7111_Write_MoreRegs(add, data_tx, len,true); //写入多个寄存器
    // M4._encoder->KTH7111_Write_MoreRegs(add, data_tx, len,true); //写入多个寄存器
    // M1._encoder->KTH7111_Write_MoreRegs(add, data_tx, len,true); //写入多个寄存器

    // M1._encoder->KTH7111_Read_MoreRegs(add, data_rx1, len); //读取多个寄存器
    // M2._encoder->KTH7111_Read_MoreRegs(add, data_rx2, len); //读取多个寄存器
    // M3._encoder->KTH7111_Read_MoreRegs(add, data_rx3, len); //读取多个寄存器
    // M4._encoder->KTH7111_Read_MoreRegs(add, data_rx4, len); //读取多个寄存器


    while (1)
    {

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


