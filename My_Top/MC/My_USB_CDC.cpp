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



