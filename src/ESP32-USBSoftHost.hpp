#ifndef __USB_SOFT_HOST_HPP_
#define __USB_SOFT_HOST_HPP_

#ifndef BLINK_GPIO
#ifdef ESP32
  #if CONFIG_IDF_TARGET_ESP32C3 || defined ESP32C3
    #define BLINK_GPIO 18
  #else
    #define BLINK_GPIO 22
  #endif
#else
#define BLINK_GPIO LED_BUILTIN
#endif
#endif

// include the modified version from Dmitry Samsonov
extern "C" {
  #include "usb_host.h"
}

static hal_queue_handle_t usb_msg_queue;

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
#ifndef USBHOST_SINGLE_CORE
    static void TimerTask(void *arg);
#endif
};



bool USB_SOFT_HOST::init( usb_pins_config_t pconf, ondetectcb_t onDetectCB, printcb_t onDataCB, ontick_t onTickCB )
{
  setOndetectCb( onDetectCB );
  setPrintCb( onDataCB );
  setUSBMessageCb( onUSBMessageDecode );
  //setMessageReceiver(
  USB_SOFT_HOST::setTaskTicker( onTickCB );
  if( _init( pconf ) ) {
#ifndef USBHOST_SINGLE_CORE
#ifdef ESP32
    xTaskCreatePinnedToCore(USB_SOFT_HOST::TimerTask, "USB Soft Host Timer Task", 8192, NULL, priority, NULL, core);
    log_w("USB Soft Host Group timer task is now running on core #%d with priority %d", core, priority);
#else
#warning implement timer task
#endif
#endif
    return true;
  }
  return false;
}



bool USB_SOFT_HOST::_init( usb_pins_config_t pconf )
{
  if( inited ) return false;


  setDelay(4);
  
  
  usb_msg_queue = hal_queue_create( 10, sizeof(struct USBMessage) );

  initStates(
    (hal_gpio_num_t)pconf.dp0, (hal_gpio_num_t)pconf.dm0,
    (hal_gpio_num_t)pconf.dp1, (hal_gpio_num_t)pconf.dm1,
    (hal_gpio_num_t)pconf.dp2, (hal_gpio_num_t)pconf.dm2,
    (hal_gpio_num_t)pconf.dp3, (hal_gpio_num_t)pconf.dm3
  );

  hal_gpio_pad_select_gpio((hal_gpio_num_t)BLINK_GPIO);
  //hal_gpio_set_direction((hal_gpio_num_t)BLINK_GPIO, GPIO_MODE_OUTPUT);
#ifdef TIMER_INTERVAL0_SEC
  #if !defined USE_NATIVE_GROUP_TIMERS && defined(ESP32)
  timer_queue = xQueueCreate( 10, sizeof(timer_event_t) );
  #endif
  hal_timer_setup(TIMER_0, (uint64_t) ((double)TIMER_INTERVAL0_SEC * TIMER_SCALE), usbhost_timer_cb);
#endif

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
  hal_queue_send( usb_msg_queue, &msg);
}



void (*USB_SOFT_HOST::ticker)() = nullptr;

/*
void USB_SOFT_HOST::TimerPause()
{
  if( !paused ) {
    log_d("Pausing timer");
    hal_timer_pause(TIMER_0);
    paused = true;
    hal_delay(1);
  } else {
    log_e("Timer already paused!");
  }
}

void USB_SOFT_HOST::TimerResume()
{
  if( paused ) {
    log_d("Resuming timer");
    hal_timer_start(TIMER_0);
    paused = false;
    hal_delay(1);
  } else {
    log_e("Timer already running!");
  }
}
*/

#ifndef USBHOST_SINGLE_CORE
void USB_SOFT_HOST::TimerTask(void *arg)
{
  while (1) {
    struct USBMessage msg;

    #if !defined(USE_NATIVE_GROUP_TIMERS) && defined(TIMER_INTERVAL0_SEC)
      timer_event_t evt;
      xQueueReceive(timer_queue, &evt, portMAX_DELAY);
    #endif
    if( hal_queue_receive(usb_msg_queue, &msg) ) {
      if( printDataCB ) {
        printDataCB( msg.src/4, 32, msg.data, msg.len );
      }
    }

    printState();
    if( ticker ) ticker();
    hal_delay(10);
  }
}
#endif

USB_SOFT_HOST USH;


#endif
