#include "My_Vofa.hpp"


// 1.发送部分
// 定义双缓冲区，防止连续发送时覆盖 
#define MAX_VOFA_FLOATS 30 // 临时缓冲区，最多发送 30 个浮点数 
#define VOFA_BUFFER_SIZE (MAX_VOFA_FLOATS * sizeof(float) + 4)  // 最多20个浮点数 + 4字节帧尾
static uint8_t vofa_buf1[VOFA_BUFFER_SIZE];
static uint8_t vofa_buf2[VOFA_BUFFER_SIZE];
static uint8_t vofa_buf_sel = 0;  // 简单轮换选择

static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

void Vofa_SendFireWater(float *data, uint16_t count, uint32_t timeout)//发送指定参数
{
    if (count == 0) return;
    uint16_t data_len = count * sizeof(float);
    uint16_t total_len = data_len + 4;

    // 选择当前可用的缓冲区（简单轮换，确保不覆盖正在DMA发送中的数据）
    uint8_t *buf = (vofa_buf_sel == 0) ? vofa_buf1 : vofa_buf2;
    vofa_buf_sel = !vofa_buf_sel;  // 下次用另一个

    // 拼接数据
    memcpy(buf, data, data_len);
    memcpy(buf + data_len, tail, 4);
 
    // CDC_Transmit_FS(buf, total_len); // 发送数据
    CDC_Transmit(0, (uint8_t*)buf, total_len); // 发送数据

}



void Vofa_SendFireWater_VA(uint16_t count, ...)//发送不定长浮点型参数
{
    float buffer[MAX_VOFA_FLOATS];
    
    if (count > MAX_VOFA_FLOATS) count = MAX_VOFA_FLOATS;  // 防溢出
    
    va_list args;
    va_start(args, count); //让 args 指向第一个可变参数
    for (uint16_t i = 0; i < count; i++) {//取出下一个参数
        buffer[i] = (float)va_arg(args, double);  // 变参 float 会被提升为 double
    }
    va_end(args);
    
    Vofa_SendFireWater(buffer, count,0);
}



//2.接受部分
VoFA_Rx vofa1 = {};

void My_USB_Handler(uint8_t* data, uint32_t len)
{
    vofa1.on_recv(data, len);
}

void Vofa_Init()
{
    USB_Register_RxCallback(My_USB_Handler); // 注册接收回调函数
}



