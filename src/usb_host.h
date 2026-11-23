#ifndef  USB_HOST_H
#define USB_HOST_H

#include <sdkconfig.h>
#include "esp_idf_version.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#if !defined ESP_IDF_VERSION_MAJOR || (defined ESP_IDF_VERSION_MAJOR && (ESP_IDF_VERSION_MAJOR < 5 || (ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR < 5)))
  #error "This espressif core is too old, minimum 5.5 required, use an earlier version of this library for older core versions"
#endif

#include "driver/gptimer.h" // general purpose timer

#if __has_include("soc/gpio_periph.h")
  #include "soc/gpio_periph.h" // for GPIO_PIN_MUX_REG
#endif

#include "driver/gpio.h"
#include "esp32-hal-log.h"

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
  #define DEBUG_ALL
#endif

#define TIMER_INTERVAL0_SEC   (0.001) // sample test interval for the first timer
// non configured device -  must be zero
#define  ZERO_USB_ADDRESS     0
// any number less 127, but no zero
#define  ASSIGNED_USB_ADDRESS 3
// usb ports in this hub
#define  NUM_USB              4

#define STDCLASS     0x00
#define HIDCLASS     0x03
#define HUBCLASS     0x09      /* bDeviceClass, bInterfaceClass */

void IRAM_ATTR printState();
void IRAM_ATTR usb_process();
typedef void (*onusbmesscb_t)( uint8_t src,uint8_t len,uint8_t *data );
void set_usb_mess_cb( onusbmesscb_t onUSBMessCb );
typedef void (*printcb_t)( uint8_t usbNum, uint8_t byte_depth, uint8_t* data, uint8_t data_len );
void set_print_cb( printcb_t onDataCB );
typedef void (*ondetectcb_t)( uint8_t usbNum, void *device );
void set_ondetect_cb( ondetectcb_t onDetectCB );
typedef void (*ondisconnectcb_t)( uint8_t usbNum );
void set_ondisconnect_cb( ondisconnectcb_t onDisconnectCB );
typedef void(*onledblinkcb_t)( int on_off );
void set_onled_blink_cb( onledblinkcb_t cb );
typedef void(*onconfigdesccb_t)( uint8_t ref, int cfgCount, void *lcfg, size_t len );
void set_onconfigdesc_cb( onconfigdesccb_t cb );
typedef void(*onifacedesccb_t)(uint8_t ref, int cfgCount, int sIntfCount, void* sIntf, size_t len);
void set_onifacedesc_cb( onifacedesccb_t cb );
typedef void(*onhiddevdesccb_t)(uint8_t ref, int cfgCount, int sIntfCount, int hidCount, void*hid, size_t len);
void set_onhiddevdesc_cb( onhiddevdesccb_t cb );
typedef void(*onepdesccb_t)(uint8_t ref, int cfgCount, int epdCount, void*epd, size_t len);
void set_onepdesc_cb( onepdesccb_t cb );


void initStates( int DP0,int DM0,int DP1,int DM1,int DP2,int DM2,int DP3,int DM3 );
void setCPUDelay( uint8_t ticks );
// keyboard led controls
uint8_t usbGetFlags(int _usb_num);
void usbSetFlags( int _usb_num,uint8_t flags );

int64_t get_system_time_us();

// Device Descriptor
typedef struct
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t bcdUSB;
  uint8_t bDeviceClass;
  uint8_t bDeviceSubClass;
  uint8_t bDeviceProtocol;
  uint8_t bMaxPacketSize0;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t iManufacturer;
  uint8_t iProduct;
  uint8_t iSerialNumber;
  uint8_t bNumConfigurations;
} sDevDesc;


// Configuration Descriptor
typedef struct
{
  uint8_t bLength;
  uint8_t bType;
  uint16_t wLength;
  uint8_t bNumIntf;
  uint8_t bCV;
  uint8_t bIndex;
  uint8_t bAttr;
  uint8_t bMaxPower;
} sCfgDesc;


// Interface Descriptor
typedef struct
{
  uint8_t bLength;
  uint8_t bType;
  uint8_t iNum;
  uint8_t iAltString;
  uint8_t bEndPoints;
  uint8_t iClass;
  uint8_t iSub;
  uint8_t iProto;
  uint8_t iIndex;
} sIntfDesc;


// HID Device Descriptor
typedef struct
{
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint16_t  bcdHID;
  uint8_t   bCountryCode;
  uint8_t   bNumDescriptors;
  uint8_t   bReportDescriptorType;
  uint8_t   wItemLengthL;
  uint8_t   wItemLengthH;
} HIDDescriptor;


// EndPoint Descriptor
typedef struct
{
  uint8_t bLength;
  uint8_t bType;
  uint8_t bEPAdd;
  uint8_t bAttr;
  uint16_t wPayLoad;               /* low-speed this must be 0x08 */
  uint8_t bInterval;
} sEPDesc;




typedef struct
{
  uint8_t bLength;
  uint8_t bType;
  uint16_t wLang;
} sStrDesc;


#endif
