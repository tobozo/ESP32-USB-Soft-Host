#pragma once

#include <Arduino.h>

#define __USB_SOFT_HOST_HPP_

#ifndef BLINK_GPIO
  #if CONFIG_IDF_TARGET_ESP32C3 || defined ESP32C3
    #define BLINK_GPIO 18
  #elif CONFIG_IDF_TARGET_ESP32S2 || defined ESP32S2
    #define BLINK_GPIO 2 //19 // 9 // 2
  #else
    #define BLINK_GPIO 22
  #endif
#endif

#if defined CONFIG_ESP_SYSTEM_MEMPROT_FEATURE || defined FORCE_TEMPLATED_NOPS
  #pragma message "memory protection features disabled, templated asm nop() will be used"
  #include "nops.hpp"
#endif


// include the modified version from Dmitry Samsonov
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
void Default_USB_DataCB(uint8_t usbNum, uint8_t byte_depth, uint8_t* data, uint8_t data_len);

#if !defined USE_NATIVE_GROUP_TIMERS

  typedef struct
  {
      int type;  // the type of timer's event
      int timer_group;
      int timer_idx;
      uint64_t timer_counter_value;
  } timer_event_t;
  static xQueueHandle timer_queue = NULL;

#endif


void IRAM_ATTR timer_group0_isr(void *para);



class USB_SOFT_HOST
{
  public:
    bool init( usb_pins_config_t pconf, ondetectcb_t detectCB = Default_USB_DetectCB, printcb_t onDataCB = Default_USB_DataCB, ontick_t onTickCB = nullptr );
    void setPrintCb( printcb_t onDataCB );
    void setOndetectCb( ondetectcb_t onDetectCB );

    void setTaskTicker( ontick_t onTickCB );
    void setActivityBlinker( onledblinkcb_t onActivityCB );
    void setTaskPriority( uint8_t p ) { priority = p; };
    void setTaskCore( uint8_t c ) { core = c; }
    void setBlinkPin( gpio_num_t pin_number );
    void setISRAllocFlag( int alloc_flags );
    // use those to avoid the pesky "Guru Meditation Error: Core 1 panic'ed (Cache disabled but cached memory region accessed)" error
    // may happen when using SPIFFS, SD or other IRAM driven libraries
    void TimerPause();
    void TimerResume();

  private:
    bool inited = false;
    bool paused = false;
    uint8_t priority = 5;
    uint8_t core = 1;
    int intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    gpio_num_t blink_gpio = (gpio_num_t)BLINK_GPIO;
    bool _init( usb_pins_config_t pconf );
    void setUSBMessageCb( onusbmesscb_t onMessageCB );
    static void onUSBMessageDecode(uint8_t src, uint8_t len, uint8_t *data);
    static void (*ticker)();
    static void TimerTask(void *arg);
};


extern USB_SOFT_HOST USH;
