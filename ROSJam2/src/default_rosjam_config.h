#include "rosjam_config.h"

#ifndef USB_CDC_ITF
#define USB_CDC_ITF 0
#endif

#ifndef USB_CDC_FIFO_SIZE
#define USB_CDC_FIFO_SIZE 2048
#endif

#ifndef USB_CDC_EP_BUF_SIZE
#define USB_CDC_EP_BUF_SIZE 2048
#endif

#ifndef ENDPOINT_COUNT 
#define ENDPOINT_COUNT 7
#endif

#ifndef ENDPOINT_BUF_LEN
#define ENDPOINT_BUF_LEN 2048
#endif

#ifndef USB_MANUFACTURER_STR
#define USB_MANUFACTURER_STR "McGill Robotics Rover"
#endif

#ifndef USB_PRODUCT_STR
#define USB_PRODUCT_STR "ROSJam device"
#endif

#ifndef USB_BASE_PID
#define USB_BASE_PID 0x4000
#endif