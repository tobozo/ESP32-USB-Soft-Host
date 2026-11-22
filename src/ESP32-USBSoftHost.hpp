#pragma once

#include <Arduino.h>
#include <sdkconfig.h>
#include "esp_idf_version.h"

#define __USB_SOFT_HOST_HPP_

#if !defined ESP_IDF_VERSION_MAJOR || (defined ESP_IDF_VERSION_MAJOR && (ESP_IDF_VERSION_MAJOR < 5 || (ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR < 5)))
  #error "This espressif core is too old, minimum 5.5 required, use an earlier version of this library for older core versions"
#endif

#if defined CONFIG_ESP_SYSTEM_MEMPROT_FEATURE || defined CONFIG_IDF_TARGET_ESP32S3 || defined CONFIG_IDF_TARGET_ESP32P4
  #define USE_TEMPLATED_NOPS
#endif

// include the now heavily modified usb_host version from Dmitry Samsonov
extern "C"
{
  #include "usb_host.h"
}


struct USBMessage;

// pins configuration
typedef struct
{
  int dp0;
  int dm0;
  int dp1;
  int dm1;
  int dp2;
  int dm2;
  int dp3;
  int dm3;
} usb_pins_config_t;

// task ticker
typedef void (*ontick_t)();


void Default_USB_DetectCB( uint8_t usbNum, void * dev );
void Default_USB_DisconnectCB( uint8_t usbNum );
void Default_USB_DataCB(uint8_t usbNum, uint8_t byte_depth, uint8_t* data, uint8_t data_len);
void Default_USB_ConfigDescCB( uint8_t ref, int cfgCount, void *lcfg, size_t len );
void Default_USB_IfaceDescCb(uint8_t ref, int cfgCount, int sIntfCount, void* sIntf, size_t len);
void Default_USB_HIDDevDescCb(uint8_t ref, int cfgCount, int sIntfCount, int hidCount, void*hid, size_t len);
void Default_USB_EPDescCb(uint8_t ref, int cfgCount, int epdCount, void*epd, size_t len);

class USB_SOFT_HOST
{
  public:
    bool init( usb_pins_config_t pconf, ondetectcb_t detectCB = Default_USB_DetectCB, printcb_t onDataCB = Default_USB_DataCB, ontick_t onTickCB = nullptr, ondisconnectcb_t onDisconnectCB = nullptr );
    void setPrintCb( printcb_t onDataCB );
    void setOndetectCb( ondetectcb_t onDetectCB );
    void setOndisconnectCb( ondisconnectcb_t onDisconnectCB );
    void setOnConfigDescCB( onconfigdesccb_t cb );
    void setOnIfaceDescCb( onifacedesccb_t cb );
    void setOnHIDDevDescCb( onhiddevdesccb_t cb );
    void setOnEPDescCb( onepdesccb_t cb );

    void setTaskTicker( ontick_t onTickCB );
    void setActivityBlinker( onledblinkcb_t onActivityCB );
    void setTaskPriority( uint8_t p ) { priority = p; };
    void setTaskCore( uint8_t c ) { core = c; }
    void setBlinkPin( gpio_num_t pin_number );
    void setISRAllocFlag( int alloc_flags ); // ESP_INTR_FLAG_IRAM, ESP_INTR_FLAG_LEVEL2 or ESP_INTR_FLAG_LEVEL3``
    // use those to avoid the pesky "Guru Meditation Error: Core 1 panic'ed (Cache disabled but cached memory region accessed)" error
    // may happen when using SPIFFS, SD or other IRAM driven libraries
    void TimerPause();
    void TimerResume();

  private:
    bool inited = false;
    bool paused = false;
    uint8_t priority = 15;
    uint8_t core = 0;
    int intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    gpio_num_t blink_gpio = GPIO_NUM_NC;
    bool _init( usb_pins_config_t pconf );
    void setUSBMessageCb( onusbmesscb_t onMessageCB );
    static void onUSBMessageDecode(uint8_t src, uint8_t len, uint8_t *data);
    static void (*ticker)();
    static void TimerTask(void *arg);
    gptimer_handle_t gptimer = NULL;
};


extern USB_SOFT_HOST USH;
