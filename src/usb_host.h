#ifndef  USB_HOST_H
#define USB_HOST_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "esp32-hal-log.h"

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
  #define DEBUG_ALL
#endif


#if defined ESP_IDF_VERSION_MAJOR && ESP_IDF_VERSION_MAJOR >= 4
  #define USE_NATIVE_GROUP_TIMERS
#else
  #warning "Using software group timers"
#endif


#define TIMER_DIVIDER         2  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (0.001) // sample test interval for the first timer
// non configured device -  must be zero
#define  ZERO_USB_ADDRESS   0
// any number less 127, but no zero
#define  ASSIGNED_USB_ADDRESS    3

void IRAM_ATTR printState();
void IRAM_ATTR usb_process();
typedef void (*onusbmesscb_t)(uint8_t src,uint8_t len,uint8_t *data);
void set_usb_mess_cb( onusbmesscb_t onUSBMessCb );
typedef void (*printcb_t)(uint8_t usbNum, uint8_t byte_depth, uint8_t* data, uint8_t data_len);
void set_print_cb( printcb_t onDataCB );
typedef void (*ondetectcb_t)(uint8_t usbNum, void *device);
void set_ondetect_cb( ondetectcb_t onDetectCB );
typedef void(*onledblinkcb_t)(int on_off);
void set_onled_blink_cb( onledblinkcb_t cb );

#define  NUM_USB 4

void initStates( int DP0,int DM0,int DP1,int DM1,int DP2,int DM2,int DP3,int DM3);
void setDelay(uint8_t ticks);
uint8_t usbGetFlags(int _usb_num);
void usbSetFlags(int _usb_num,uint8_t flags);


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
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint16_t  bcdHID;
  uint8_t   bCountryCode;
  uint8_t   bNumDescriptors;
  uint8_t   bReportDescriptorType;
  uint8_t   wItemLengthL;
  uint8_t   wItemLengthH;
} HIDDescriptor;



typedef struct
{
  uint8_t bLength;
  uint8_t bType;
  uint16_t wLang;
} sStrDesc;




#if !defined USE_NATIVE_GROUP_TIMERS
  typedef struct
  {
      int type;  // the type of timer's event
      int timer_group;
      int timer_idx;
      uint64_t timer_counter_value;
  } timer_event_t;
#endif

static xQueueHandle timer_queue = NULL;

static void IRAM_ATTR timer_group0_isr(void *para)
{
  #if defined USE_NATIVE_GROUP_TIMERS
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    //taskENTER_CRITICAL();
    usb_process();
    //taskEXIT_CRITICAL();
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
  #else
    // this is mainly a group-timer layer for esp-idf 3.x
    // most of this is handled by the SDK since esp-idf 4.x
    int timer_idx = (int) para;
    uint32_t intr_status = TIMERG0.int_st_timers.val;// Retrieve the interrupt status and the counter value from the timer that reported the interrupt
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value = ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32 | TIMERG0.hw_timer[timer_idx].cnt_low;
    timer_event_t evt; // Prepare basic event data that will be then sent back to the main program task
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;
    usb_process();// process USB signal
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) { // Clear the interrupt and update the alarm time for the timer with without reload
      evt.type = 1; // no reload
      TIMERG0.int_clr_timers.t0 = 1;
      timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
      TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
      TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
    }
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN; // After the alarm has been triggered we need enable it again, so it is triggered the next time
    xQueueSendFromISR(timer_queue, &evt, NULL); // Now just send the event data back to the main program task
  #endif
}



#endif
