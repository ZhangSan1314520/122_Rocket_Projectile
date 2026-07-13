#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*USB_RxCallback)(uint8_t* data, uint32_t len); //定义函数指针类型


void USB_Register_RxCallback(USB_RxCallback cb);//用户调用绑定回调函数

void My_USB_CDC_ProcessRx(uint8_t* data, uint32_t len);

#ifdef __cplusplus
}
#endif

