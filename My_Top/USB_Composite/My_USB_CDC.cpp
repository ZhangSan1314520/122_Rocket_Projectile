#include "My_USB_CDC.hpp"

static USB_RxCallback _usb_rx_cb = NULL;

void USB_Register_RxCallback(USB_RxCallback cb)
{
    _usb_rx_cb = cb;
}

void My_USB_CDC_ProcessRx(uint8_t* data, uint32_t len)
{
    if (_usb_rx_cb != NULL) {
        _usb_rx_cb(data, len);
    }
}




/* 复合设备添加补丁修改
    #define _STM32F1_DEVICE      改成true cubemx配置
    MX_USB_DEVICE_Init();     //初始化USB设备 换大写


  // CDC_Transmit(cdc_ch, Buf, *Len); // echo back on same channel
  extern void My_USB_CDC_ProcessRx(uint8_t* data, uint32_t len); //中断调用
  My_USB_CDC_ProcessRx(Buf, *Len);







*/

