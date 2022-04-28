#ifndef USB_HOST_H
#define USB_HOST_H

#define USBHOST_SINGLE_CORE

#ifndef USBHOST_SINGLE_CORE
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
  #define DEBUG_ALL
#endif
#endif

#if defined(ESP32) || defined(__IMXRT1062__)
#define TIMER_INTERVAL0_SEC   (0.001) // sample test interval for the first timer
#endif

#define USE_TUSB_FIFO
#ifdef USE_TUSB_FIFO
#include <common/tusb_fifo.h> //use TinyUSB queues
#endif

#ifdef ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "esp32-hal-log.h"

#ifdef TIMER_INTERVAL0_SEC
#include "soc/timer_group_struct.h"
#include "driver/timer.h"
#endif

#if defined ESP_IDF_VERSION_MAJOR && ESP_IDF_VERSION_MAJOR >= 4
  #define USE_NATIVE_GROUP_TIMERS
#else
  #warning "Using software group timers"
#endif


#define      hal_gpio_pad_select_gpio(pin) gpio_pad_select_gpio(pin)
//#define      hal_gpio_set_direction(pin, output) gpio_set_direction(pin, (output) ? GPIO_MODE_OUTPUT: GPIO_MODE_INPUT);
#define      hal_gpio_set_direction(pin, output) if(output) { GPIO.enable_w1ts = 1 << (pin); PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[pin]); } else { PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[pin]); GPIO.enable_w1tc = (1 << (pin)); }
#define      hal_gpio_set_level(pin, level) gpio_set_level(pin, level)
#define      hal_gpio_pulldown_en(pin) gpio_pulldown_en(pin)
#define      hal_gpio_read(pin) ((GPIO.in>>pin)&1)
#define      hal_enable_irq() //seems not required since the dual processor
#define      hal_disable_irq()


//#ifdef TIMER_INTERVAL0_SEC
//#define hal_timer_start(tim) timer_start(TIMER_GROUP_0, tim)
//#define hal_timer_pause(tim) timer_pause(TIMER_GROUP_0, tim)
//#endif

#ifndef USE_TUSB_FIFO
typedef xQueueHandle hal_queue_handle_t;
#define hal_queue_create(n, sz, b) xQueueCreate(n, sz)
#define hal_queue_send(q, m)  xQueueSend(q, ( void * ) (m), (TickType_t)0)
#define hal_queue_receive(q, m) xQueueReceive(q, m, 0)
#endif

#define hal_gpio_num_t gpio_num_t

#define hal_delay(x) vTaskDelay((x)/portTICK_PERIOD_MS)


#define TIMER_DIVIDER         2  //  Hardware timer clock divider
#define TIMER_CB_HAS_PARAM
#else //not ESP32
#include <math.h>
#undef IRAM_ATTR
#define IRAM_ATTR
#define      hal_gpio_pad_select_gpio(pin)

#ifdef __IMXRT1062__
#define GPIO_MODE_OUTPUT OUTPUT
#define GPIO_MODE_INPUT INPUT
#define      hal_gpio_set_direction(pin, output) pinMode(pin, output ? OUTPUT : INPUT_PULLDOWN)
#define      hal_gpio_set_level(pin, level) digital_pin_to_info_PGM[pin].reg[level?0x21:0x22] = digital_pin_to_info_PGM[pin].mask //digitalWrite(pin, level ? HIGH : LOW)
#define      hal_gpio_read(pin) (digital_pin_to_info_PGM[pin].reg[2] & digital_pin_to_info_PGM[pin].mask ? 1 : 0)//digitalRead(pin)
#define      hal_gpio_pulldown_en(pin)
#define      hal_enable_irq() interrupts()
#define      hal_disable_irq() noInterrupts()
#define cpu_hal_get_cycle_count64() ARM_DWT_CYCCNT
#define cpu_hal_get_cycle_count (uint32_t) cpu_hal_get_cycle_count64
#define hal_delay(x) delay(x)
#define hal_get_cpu_mhz() (F_CPU/1000000)

#else //not __IMXRT1062__

#include <liblitesdk/litesdk_gpio.h>
#include <liblitesdk/litesdk_timer.h>
#include <generated/soc.h>
#define GPIO_MODE_OUTPUT true
#define GPIO_MODE_INPUT false
#define USBHOST_GPIO litegpio0
#define hal_gpio_set_direction(pin, output) if(output) litegpio_mode_output(USBHOST_GPIO, pin); else litegpio_mode_input(USBHOST_GPIO, pin)
#define hal_gpio_set_level(pin, level) litegpio_write(USBHOST_GPIO, pin, level)
#define hal_gpio_read(pin) litegpio_read(USBHOST_GPIO, pin)
#define hal_gpio_pulldown_en(pin)

#define      hal_enable_irq() irq_setie(1)
#define      hal_disable_irq() irq_setie(0)

#define hal_set_differential_gpio_value(dp, dm,v) USBHOST_GPIO->OUT = ((v & 1) << dm) | ((v == 0) << dp) //| (1 << BLINK_GPIO)
/*
#define SET_I(dp, dm)  USBHOST_GPIO->OE &= ~((1 << dp) | (1 << dm))
#define SET_O(dp, dm)  USBHOST_GPIO->OE |= (1 << dp) | (1 << dm)
#define SE_J USBHOST_GPIO->OUT = 1 << DP_PIN //clear / set
#define SE_0 USBHOST_GPIO->OUT = 0 //clear / clear
*/

inline uint32_t cpu_hal_get_cycle_count() { timer0_uptime_latch_write(1); return csr_read_simple(CSR_TIMER0_UPTIME_CYCLES_ADDR+4); }
inline uint64_t cpu_hal_get_cycle_count64() { timer0_uptime_latch_write(1); return timer0_uptime_cycles_read(); }
#define F_CPU LITETIMER_BASE_FREQUENCY
#define hal_delay(x) {long t1=cpu_hal_get_cycle_count64()+x*(F_CPU/1000); while(long(cpu_hal_get_cycle_count() - t1) < 0); }
#define hal_get_cpu_mhz() (F_CPU/1000000)
#define micros() ((1000000ull*cpu_hal_get_cycle_count64())/F_CPU)
#endif

typedef int timer_idx_t;
#define log_d(m) printf(m)
#define log_e(m) printf(m)
#define hal_gpio_num_t int
#ifdef TIMER_INTERVAL0_SEC
#define TIMER_0 0
#define TIMER_DIVIDER  1
#define TIMER_BASE_CLK 1000000
//#define hal_timer_start(tim)
//#define hal_timer_pause(tim)
#define portTICK_PERIOD_MS 1
#define USE_NATIVE_GROUP_TIMERS
#define usbhost_timer_cb usb_process
#endif

#define fabsf(x) (float)fabs(x)

#ifndef hal_gpio_set_level
#define      hal_gpio_set_level(pin, level) digitalWrite(pin, level ? HIGH : LOW)
#endif
#ifndef hal_gpio_read
#define      hal_gpio_read(pin) digitalRead(pin)
#endif

#endif //ESP32

#ifdef USE_TUSB_FIFO
typedef tu_fifo_t hal_queue_handle_t;
hal_queue_handle_t hal_queue_create(size_t n, size_t sz, void *buffer);
#define hal_queue_send(q, m) tu_fifo_write(&q, m)
#define hal_queue_receive(q, m) tu_fifo_read(&q, m)
#endif

#ifdef TIMER_INTERVAL0_SEC
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#ifdef TIMER_CB_HAS_PARAM
typedef void (*timer_isr_t)(void *para);
#else
typedef void (*timer_isr_t)(void);
#endif
void hal_timer_setup(timer_idx_t timer_num, uint32_t alarm_value, timer_isr_t timer_isr);
#endif

// non configured device -  must be zero
#define  ZERO_USB_ADDRESS   0
// any number less 127, but no zero
#define  ASSIGNED_USB_ADDRESS    3

void IRAM_ATTR printState(void);
void IRAM_ATTR usb_process(void);
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
void setDelay(uint16_t ticks);
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



#ifdef ESP32
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

#ifdef TIMER_INTERVAL0_SEC
static void IRAM_ATTR usbhost_timer_cb(void *para)
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

#endif


#endif
