#ifndef __USB_SOFT_HOST_HPP_
#define __USB_SOFT_HOST_HPP_

#ifndef BLINK_GPIO
  #if CONFIG_IDF_TARGET_ESP32C3 || defined ESP32C3
    #define BLINK_GPIO 18
  #else
    #define BLINK_GPIO 22
  #endif
#endif


// include the modified version from Dmitry Samsonov
extern "C" {
  #include "usb_host.h"
}

static xQueueHandle usb_msg_queue = NULL;

struct USBMessage
{
  uint8_t src;
  uint8_t len;
  uint8_t data[0x8];
};



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


void (*printDataCB)(uint8_t usbNum, uint8_t byte_depth, uint8_t* data, uint8_t data_len) = NULL;

void set_print_cb( printcb_t cb )
{
  printDataCB = cb;
}


static void Default_USB_DetectCB( uint8_t usbNum, void * dev )
{
  sDevDesc *device = (sDevDesc*)dev;
  printf("New device detected on USB#%d\n", usbNum);
  printf("desc.bcdUSB             = 0x%04x\n", device->bcdUSB);
  printf("desc.bDeviceClass       = 0x%02x\n", device->bDeviceClass);
  printf("desc.bDeviceSubClass    = 0x%02x\n", device->bDeviceSubClass);
  printf("desc.bDeviceProtocol    = 0x%02x\n", device->bDeviceProtocol);
  printf("desc.bMaxPacketSize0    = 0x%02x\n", device->bMaxPacketSize0);
  printf("desc.idVendor           = 0x%04x\n", device->idVendor);
  printf("desc.idProduct          = 0x%04x\n", device->idProduct);
  printf("desc.bcdDevice          = 0x%04x\n", device->bcdDevice);
  printf("desc.iManufacturer      = 0x%02x\n", device->iManufacturer);
  printf("desc.iSerialNumber      = 0x%02x\n", device->iSerialNumber);
  printf("desc.bNumConfigurations = 0x%02x\n", device->bNumConfigurations);
  // if( device->idProduct == mySupportedIdProduct && device->idVendor == mySupportedVendor ) {
  //   myListenUSBPort = usbNum;
  // }
}

static void Default_USB_DataCB(uint8_t usbNum, uint8_t byte_depth, uint8_t* data, uint8_t data_len)
{
  // if( myListenUSBPort != usbNum ) return;
  printf("in: ");
  for(int k=0;k<data_len;k++) {
    printf("0x%02x ", data[k] );
  }
  printf("\n");
}




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
    // use those to avoid the pesky "Guru Meditation Error: Core 1 panic'ed (Cache disabled but cached memory region accessed)" error
    // may happen when using SPIFFS, SD or other IRAM driven libraries
    void TimerPause();
    void TimerResume();


  private:
    bool inited = false;
    bool paused = false;
    uint8_t priority = 5;
    uint8_t core = 1;
    bool _init( usb_pins_config_t pconf );
    void setUSBMessageCb( onusbmesscb_t onMessageCB );
    static void onUSBMessageDecode(uint8_t src, uint8_t len, uint8_t *data);
    static void (*ticker)();
    static void TimerTask(void *arg);
};



bool USB_SOFT_HOST::init( usb_pins_config_t pconf, ondetectcb_t onDetectCB, printcb_t onDataCB, ontick_t onTickCB )
{
  setOndetectCb( onDetectCB );
  setPrintCb( onDataCB );
  setUSBMessageCb( onUSBMessageDecode );
  //setMessageReceiver(
  USB_SOFT_HOST::setTaskTicker( onTickCB );
  if( _init( pconf ) ) {
    xTaskCreatePinnedToCore(USB_SOFT_HOST::TimerTask, "USB Soft Host Timer Task", 8192, NULL, priority, NULL, core);
    log_w("USB Soft Host Group timer task is now running on core #%d with priority %d", core, priority);
    return true;
  }
  return false;
}



bool USB_SOFT_HOST::_init( usb_pins_config_t pconf )
{
  if( inited ) return false;

  timer_config_t config;
  config.divider     = TIMER_DIVIDER;
  config.counter_dir = TIMER_COUNT_UP;
  config.counter_en  = TIMER_PAUSE;
  config.alarm_en    = TIMER_ALARM_EN;
  config.auto_reload = (timer_autoreload_t) 1; // fix for ¬invalid conversion from 'int' to 'timer_autoreload_t'¬ thanks rudi ;-)

  #if !defined USE_NATIVE_GROUP_TIMERS
    timer_queue = xQueueCreate( 10, sizeof(timer_event_t) );
  #endif

  setDelay(4);

  usb_msg_queue = xQueueCreate( 10, sizeof(struct USBMessage) );

  initStates(
    (gpio_num_t)pconf.dp0, (gpio_num_t)pconf.dm0,
    (gpio_num_t)pconf.dp1, (gpio_num_t)pconf.dm1,
    (gpio_num_t)pconf.dp2, (gpio_num_t)pconf.dm2,
    (gpio_num_t)pconf.dp3, (gpio_num_t)pconf.dm3
  );

  gpio_pad_select_gpio((gpio_num_t)BLINK_GPIO);
  //gpio_set_direction((gpio_num_t)BLINK_GPIO, GPIO_MODE_OUTPUT);

  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, (double)TIMER_INTERVAL0_SEC * TIMER_SCALE);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr, (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);
  timer_start(TIMER_GROUP_0, TIMER_0);

  inited = true;

  return true;
}


void USB_SOFT_HOST::setUSBMessageCb( onusbmesscb_t onMessageCB )
{
  set_usb_mess_cb( onMessageCB );
}


void USB_SOFT_HOST::setPrintCb( printcb_t onDataCB )
{
  set_print_cb( onDataCB );
}



void USB_SOFT_HOST::setOndetectCb( ondetectcb_t onDetectCB )
{
  set_ondetect_cb( onDetectCB );
}



void USB_SOFT_HOST::setTaskTicker( ontick_t onTickCB )
{
  USB_SOFT_HOST::ticker = onTickCB;
}



void USB_SOFT_HOST::setActivityBlinker( onledblinkcb_t onActivityCB )
{
  set_onled_blink_cb( onActivityCB );
}



// called from underlaying C
void USB_SOFT_HOST::onUSBMessageDecode(uint8_t src, uint8_t len, uint8_t *data)
{
  struct  USBMessage msg;
  msg.src = src;
  msg.len = len<0x8?len:0x8;
  for(int k=0;k<msg.len;k++) {
    msg.data[k] = data[k];
  }
  xQueueSend( usb_msg_queue, ( void * ) &msg,(TickType_t)0 );
}



void (*USB_SOFT_HOST::ticker)() = nullptr;


void USB_SOFT_HOST::TimerPause()
{
  if( !paused ) {
    log_d("Pausing timer");
    timer_pause(TIMER_GROUP_0, TIMER_0);
    paused = true;
    vTaskDelay(1);
  } else {
    log_e("Timer already paused!");
  }
}

void USB_SOFT_HOST::TimerResume()
{
  if( paused ) {
    log_d("Resuming timer");
    timer_start(TIMER_GROUP_0, TIMER_0);
    paused = false;
    vTaskDelay(1);
  } else {
    log_e("Timer already running!");
  }
}



void USB_SOFT_HOST::TimerTask(void *arg)
{
  while (1) {
    struct USBMessage msg;

    #if !defined USE_NATIVE_GROUP_TIMERS
      timer_event_t evt;
      xQueueReceive(timer_queue, &evt, portMAX_DELAY);
    #endif

    if( xQueueReceive(usb_msg_queue, &msg, 0) ) {
      if( printDataCB ) {
        printDataCB( msg.src/4, 32, msg.data, msg.len );
      }
    }

    printState();
    if( ticker ) ticker();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


USB_SOFT_HOST USH;


#endif
