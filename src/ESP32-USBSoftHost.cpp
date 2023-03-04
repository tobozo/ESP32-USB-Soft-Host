
#include "./ESP32-USBSoftHost.hpp"

static xQueueHandle usb_msg_queue = NULL;

struct USBMessage
{
  uint8_t src;
  uint8_t len;
  uint8_t data[0x8];
};


void (*printDataCB)(uint8_t usbNum, uint8_t byte_depth, uint8_t* data, uint8_t data_len) = NULL;

void set_print_cb( printcb_t cb )
{
  printDataCB = cb;
}


void Default_USB_DetectCB( uint8_t usbNum, void * dev )
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

void Default_USB_DataCB(uint8_t usbNum, uint8_t byte_depth, uint8_t* data, uint8_t data_len)
{
  // if( myListenUSBPort != usbNum ) return;
  printf("in: ");
  for(int k=0;k<data_len;k++) {
    printf("0x%02x ", data[k] );
  }
  printf("\n");
}


#if !defined USE_NATIVE_GROUP_TIMERS

  typedef struct
  {
      int type;  // the type of timer's event
      int timer_group;
      int timer_idx;
      uint64_t timer_counter_value;
  } timer_event_t;
  static xQueueHandle timer_queue = NULL;

  void IRAM_ATTR timer_group0_isr(void *para)
  {
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

  }

#else

  void IRAM_ATTR timer_group0_isr(void *para)
  {
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    //taskENTER_CRITICAL();
    usb_process();
    //taskEXIT_CRITICAL();
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
  }

#endif



bool USB_SOFT_HOST::init( usb_pins_config_t pconf, ondetectcb_t onDetectCB, printcb_t onDataCB, ontick_t onTickCB )
{
  Serial.println("Attaching onDetectCB");
  setOndetectCb( onDetectCB );
  Serial.println("Attaching onDataCB");
  setPrintCb( onDataCB );
  Serial.println("Attaching onUSBMessageDecode");
  setUSBMessageCb( onUSBMessageDecode );
  Serial.println("Attaching onTickCB");
  USB_SOFT_HOST::setTaskTicker( onTickCB );
  if( _init( pconf ) ) {
    Serial.println("Attaching Timer Task");
    xTaskCreatePinnedToCore(USB_SOFT_HOST::TimerTask, "USB Soft Host Timer Task", 8192, NULL, priority, NULL, core);
    //xTaskCreate(USB_SOFT_HOST::TimerTask, "USB Soft Host Timer Task", 8192, NULL, priority, NULL);
    Serial.printf("USB Soft Host Group timer task is now running on core #%d with priority %d\n", core, priority);
    return true;
  }
  return false;
}


void USB_SOFT_HOST::setBlinkPin( gpio_num_t pin_number )
{
  blink_gpio = pin_number;
}


void USB_SOFT_HOST::setISRAllocFlag( int alloc_flags )
{
  // ESP_INTR_FLAG_LEVEL1        < Accept a Level 1 interrupt vector (lowest priority)
  // ESP_INTR_FLAG_LEVEL2        < Accept a Level 2 interrupt vector
  // ESP_INTR_FLAG_LEVEL3        < Accept a Level 3 interrupt vector
  // ESP_INTR_FLAG_LEVEL4        < Accept a Level 4 interrupt vector
  // ESP_INTR_FLAG_LEVEL5        < Accept a Level 5 interrupt vector
  // ESP_INTR_FLAG_LEVEL6        < Accept a Level 6 interrupt vector
  // ESP_INTR_FLAG_NMI           < Accept a Level 7 interrupt vector (highest priority)
  // ESP_INTR_FLAG_SHARED        < Interrupt can be shared between ISRs
  // ESP_INTR_FLAG_EDGE          < Edge-triggered interrupt
  // ESP_INTR_FLAG_IRAM          < ISR can be called if cache is disabled
  // ESP_INTR_FLAG_INTRDISABLED  < Return with this interrupt disabled
  intr_alloc_flags = alloc_flags;
}


bool USB_SOFT_HOST::_init( usb_pins_config_t pconf )
{
  if( inited ) return false;

  #if !defined USE_NATIVE_GROUP_TIMERS
    Serial.println("Using custom group timer");
    timer_queue = xQueueCreate( 10, sizeof(timer_event_t) );
  #endif

  Serial.println("Creating message queue");

  usb_msg_queue = xQueueCreate( 10, sizeof(struct USBMessage) );

  Serial.println("Setting USB Delay");

  setCPUDelay(4);

  Serial.println("Setting up pins");

  initStates(
    (gpio_num_t)pconf.dp0, (gpio_num_t)pconf.dm0,
    (gpio_num_t)pconf.dp1, (gpio_num_t)pconf.dm1,
    (gpio_num_t)pconf.dp2, (gpio_num_t)pconf.dm2,
    (gpio_num_t)pconf.dp3, (gpio_num_t)pconf.dm3
  );

  Serial.printf("Seleting SCL (blink) Pin #%d\n", blink_gpio );

  gpio_pad_select_gpio( blink_gpio );
  gpio_set_direction( blink_gpio, GPIO_MODE_OUTPUT);

  Serial.println("Creating timer config");

  #if defined ESP32 || defined ESP32S2
    timer_config_t config;
    config.divider     = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en  = TIMER_PAUSE;
    config.alarm_en    = TIMER_ALARM_EN;
    config.intr_type   = TIMER_INTR_MAX;
    config.auto_reload = (timer_autoreload_t) 1; // fix for ¬invalid conversion from 'int' to 'timer_autoreload_t'¬ thanks rudi ;-)
  #elif defined ESP32C3
    timer_config_t config =
    {
      .clk_src     = TIMER_SRC_CLK_XTAL, // TIMER_SRC_CLK_DEFAULT ?
      .alarm_en    = TIMER_ALARM_EN,//enable timer alarm
      .counter_en  = TIMER_PAUSE,//starts counting counter once timer_init called
      .intr_type   = TIMER_INTR_MAX,
      .counter_dir = TIMER_COUNT_UP,//counts from 0 to counter value
      .auto_reload = TIMER_AUTORELOAD_EN,// reloads counter automatically
      .divider     = TIMER_DIVIDER
    };
  //#elif defined ESP32S2
  #else
    #error "Invalid board"
  #endif

  Serial.println("Init timer");
  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, (double)TIMER_INTERVAL0_SEC * TIMER_SCALE);
  Serial.println("Enable interrupt");
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  Serial.println("Register ISR");
  timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr, (void *) TIMER_0, intr_alloc_flags, NULL);
  Serial.println("Start timer");
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
