#include "vcp.h"

#define USB_TIMEOUT  50

static VCP* vcpPtr = nullptr;

void vcp_rx_callback(uint8_t byte)
{
  if (vcpPtr->cb_)
    vcpPtr->cb_(byte);
}

void vcp_ls_callback(void *context, uint16_t ctrlLineState)
{
  (void)context;
  (void)ctrlLineState;
  if (ctrlLineState != 3)
  {
    volatile int debug = 1;
    (void)debug;
  }  
}

void VCP::init()
{
  // Initialize the GPIOs for the pins
  rx_pin_.init(GPIOA, GPIO_Pin_11, GPIO::PERIPH_IN_OUT);
  tx_pin_.init(GPIOA, GPIO_Pin_12, GPIO::PERIPH_IN_OUT);
  vbus_sens_.init(GPIOC, GPIO_Pin_5, GPIO::INPUT);

  send_disconnect_signal();
  connected_ = false;
  USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
  vcpPtr = this;
  
  CDC_SetCtrlLineStateCb(&vcp_ls_callback, NULL);
}

void VCP::write(const uint8_t*ch, uint8_t len)
{
  uint32_t start = millis();
  
  // Don't send bytes to a disconnected USB
  if (connected() && tx_bytes_free() > 0)
  {  
    while (len > 0)
    {
      uint32_t num_bytes_sent = CDC_Send_DATA(ch, len);
      len -= num_bytes_sent;
      ch += num_bytes_sent;
  
      if (millis() > start + USB_TIMEOUT)
        break;
    }
  }
  perform_maintenance();
}

void VCP::perform_maintenance()
{
  // detect if we are connected to a computer
  if (rx_bytes_waiting())
    connected_ = true;
  
  // If we were receiving bytes, but now we're not connected
  // reset the VCP
  if (!connected() && connected_)
  {
    USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
    connected_ = false;    
  }
}


uint32_t VCP::rx_bytes_waiting()
{
  return CDC_Receive_BytesAvailable();
}


uint32_t VCP::tx_bytes_free()
{
  return CDC_Send_FreeBytes();
}


uint8_t VCP::read_byte()
{
  uint8_t data;

  if (CDC_Receive_DATA(&data, 1))
  {
    connected_ = true;
    return data;
  }
  else
    return 0;
}

bool VCP::tx_buffer_empty()
{
  return CDC_Send_FreeBytes() > 0;
}


void VCP::put_byte(uint8_t ch)
{
  if (connected())
    CDC_Send_DATA(&ch, 1);
}

bool VCP::connected()
{
  return (vbus_sens_.read() == GPIO::HIGH);
}

bool VCP::flush()
{
//  CDC_flush();
  return true;
}
void VCP::begin_write(){}
void VCP::end_write(){}


void VCP::register_rx_callback(void (*rx_callback_ptr)(uint8_t data) )
{
  cb_ = rx_callback_ptr;
//  Register_CDC_RxCallback(&vcp_rx_callback);
}
void VCP::unregister_rx_callback()
{
    receive_CB_ = NULL;
//    Register_CDC_RxCallback(NULL);
}


bool VCP::in_bulk_mode()
{
  return false;
}


void VCP::send_disconnect_signal()
{
  tx_pin_.set_mode(GPIO::OUTPUT);
  tx_pin_.write(GPIO::LOW);
  delay(200);
  tx_pin_.write(GPIO::HIGH);
  tx_pin_.set_mode(GPIO::PERIPH_IN_OUT);
}
