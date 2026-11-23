
#include "./ESP32-USBSoftHost.hpp"

#if defined USE_TEMPLATED_NOPS
  // #pragma message "memory protection features disabled, templated asm nop() will be used"
  #include "nops.hpp"
#endif

#include "esp_clk_tree.h"
#include "soc/clk_tree_defs.h"

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


void Default_USB_DisconnectCB( uint8_t usbNum )
{
  printf("USB#%d disconnected\n", usbNum);
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



void Default_USB_ConfigDescCB( uint8_t ref, int cfgCount, void *cfg, size_t len )
{
  sCfgDesc *lcfg = (sCfgDesc*)cfg;
  printf("Config Descriptor #%d\n", cfgCount);
  printf("  cfg.wLength         = 0x%02x\n", lcfg->wLength);
  printf("  cfg.bNumIntf        = 0x%02x\n", lcfg->bNumIntf);
  printf("  cfg.bCV             = 0x%02x\n", lcfg->bCV);
  printf("  cfg.bMaxPower       = %d\n",     lcfg->bMaxPower);
}


void Default_USB_IfaceDescCb(uint8_t ref, int cfgCount, int sIntfCount, void* Intf, size_t len)
{
  sIntfDesc *sIntf = (sIntfDesc*)Intf;
  printf("    Interface Descriptor #%d\n", sIntfCount);
  printf("      sIntf.bLength     = 0x%02x\n", sIntf->bLength);
  printf("      sIntf.bType       = 0x%02x\n", sIntf->bType);
  printf("      sIntf.iNum        = 0x%02x\n", sIntf->iNum);
  printf("      sIntf.iAltString  = 0x%02x\n", sIntf->iAltString);
  printf("      sIntf.bEndPoints  = 0x%02x\n", sIntf->bEndPoints);
  printf("      sIntf.iClass      = 0x%02x\n", sIntf->iClass);
  printf("      sIntf.iSub        = 0x%02x\n", sIntf->iSub);
  printf("      sIntf.iProto      = 0x%02x\n", sIntf->iProto);
  printf("      sIntf.iIndex      = 0x%02x\n", sIntf->iIndex);
}


void Default_USB_HIDDevDescCb(uint8_t ref, int cfgCount, int sIntfCount, int hidCount, void*vhid, size_t len)
{
  HIDDescriptor *hid = (HIDDescriptor*)vhid;
  printf("        HID Device Descriptor #%d\n", hidCount);
  printf("          hid.bLength               = 0x%02x\n", hid->bLength               );
  printf("          hid.bDescriptorType       = 0x%02x\n", hid->bDescriptorType       );
  printf("          hid.bcdHID                = 0x%02x\n", hid->bcdHID                );
  printf("          hid.bCountryCode          = 0x%02x\n", hid->bCountryCode          );
  printf("          hid.bNumDescriptors       = 0x%02x\n", hid->bNumDescriptors       );
  printf("          hid.bReportDescriptorType = 0x%02x\n", hid->bReportDescriptorType );
  printf("          hid.wItemLengthL          = 0x%02x\n", hid->wItemLengthL          );
  printf("          hid.wItemLengthH          = 0x%02x\n", hid->wItemLengthH          );
}


void Default_USB_EPDescCb(uint8_t ref, int cfgCount, int epdCount, void*vepd, size_t len)
{
  sEPDesc *epd = (sEPDesc*)vepd;
  printf("      EndPoint Descriptor #%d\n", epdCount);
  printf("        epd.bLength       = 0x%02x\n", epd->bLength);
  printf("        epd.bType         = 0x%02x\n", epd->bType);
  printf("        epd.bEPAdd        = 0x%02x\n", epd->bEPAdd);
  printf("        epd.bAttr         = 0x%02x\n", epd->bAttr);
  printf("        epd.wPayLoad      = 0x%02x\n", epd->wPayLoad);
  printf("        epd.bInterval     = 0x%02x\n", epd->bInterval);
}




static bool ush_timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *ushptr)
{
  //// stop the timer when the alarm occurs
  // gptimer_stop(timer);

  usb_process();

  //// set counter & re-arm alarm
  // gptimer_set_raw_count(timer, edata->alarm_value);
  // gptimer_alarm_config_t alarm_config = {
  //     .alarm_count = edata->alarm_value + ALARM_INTERVAL, // Next alarm in 1000000us (1s) from the current alarm
  //     //.reload_count = 0,
  //     .flags = { .auto_reload_on_alarm = false },
  // };
  //// Update the alarm value
  // gptimer_set_alarm_action(timer, &alarm_config);
  //// resume the timer
  // gptimer_start(timer);

  // Return whether a high-priority task was awakened during the above operations to notify the scheduler to switch tasks
  return false;
}





bool USB_SOFT_HOST::init( usb_pins_config_t pconf, ondetectcb_t onDetectCB, printcb_t onDataCB, ontick_t onTickCB, ondisconnectcb_t onDisconnectCB )
{
  Serial.println("Attaching onDetectCB");
  setOndetectCb( onDetectCB );

  if( onDisconnectCB ) {
    Serial.println("Attaching onDisconnectCB");
    setOndisconnectCb( onDisconnectCB );
  }

  Serial.println("Attaching onDataCB");
  setPrintCb( onDataCB );

  Serial.println("Attaching onUSBMessageDecode");
  setUSBMessageCb( onUSBMessageDecode );

  if( onTickCB ) {
    Serial.println("Attaching onTickCB");
    USB_SOFT_HOST::setTaskTicker( onTickCB );
  }

  if( _init( pconf ) ) {
    Serial.println("Attaching Timer Task");
    xTaskCreatePinnedToCore(USB_SOFT_HOST::TimerTask, "USB Soft Host Timer Task", 8192, NULL, priority, NULL, core);
    //xTaskCreate(USB_SOFT_HOST::TimerTask, "USB Soft Host Timer Task", 8192, NULL, priority, NULL);
    Serial.printf("USB Soft Host Timer task is now running on core #%d with priority %d\n", core, priority);
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

  Serial.println("Creating message queue");

  usb_msg_queue = xQueueCreate( 10, sizeof(struct USBMessage) );

  Serial.println("Setting USB Delay");

  static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

  #if defined USE_TEMPLATED_NOPS
    for(uint8_t i=0;i<0xff;i++)
    {
      auto cpuDelay = nops[i];
      // using asm "memory" trick +lock to prevent the compiler from optimizing the benchmark
      portENTER_CRITICAL(&my_spinlock);
      asm volatile("" ::: "memory");
      auto start = esp_cpu_get_cycle_count();
      cpuDelay();
      auto end = esp_cpu_get_cycle_count()/*-cycle_count_cycles*/;
      asm volatile("" ::: "memory");
      portEXIT_CRITICAL(&my_spinlock);
      uint64_t elapsed = (end-start);
      //float avg = elapsed/1000.0f;
      printf("Nop[%02x] ~= %llu cycles\n", i, elapsed);
    }
  #endif


  Serial.println("Setting up pins");

  initStates(
    (gpio_num_t)pconf.dp0, (gpio_num_t)pconf.dm0,
    (gpio_num_t)pconf.dp1, (gpio_num_t)pconf.dm1,
    (gpio_num_t)pconf.dp2, (gpio_num_t)pconf.dm2,
    (gpio_num_t)pconf.dp3, (gpio_num_t)pconf.dm3
  );

  if( blink_gpio != GPIO_NUM_NC ) {
    Serial.printf("Seleting SCL (blink) Pin #%d\n", blink_gpio );
    esp_rom_gpio_pad_select_gpio( blink_gpio );
    gpio_set_direction( blink_gpio, GPIO_MODE_OUTPUT);
  }

  Serial.printf("Note: this SOC has %d PINS\n", SOC_GPIO_PIN_COUNT);

  Serial.println("Creating timer config");

  uint32_t apb_freq_hz = 0;
  esp_clk_tree_src_get_freq_hz( SOC_MOD_CLK_APB, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &apb_freq_hz );

  #if defined CONFIG_IDF_TARGET_ESP32P4
    #define GPTIMER_CLK_SRC GPTIMER_CLK_SRC_XTAL // GPTIMER_CLK_SRC_DEFAULT // GPTIMER_CLK_SRC_PLL_F80M
  #else
    #define GPTIMER_CLK_SRC GPTIMER_CLK_SRC_DEFAULT
  #endif


  uint32_t timer_resolution_hz = 1000000; // 1MHz
  uint64_t alarm_interval = 1000; // 1ms

  Serial.printf("Timer resolution = %lu Hz\n", timer_resolution_hz);
  Serial.printf("Timer alarm count = %llu ticks = 1ms\n", alarm_interval);


  gptimer_config_t timer_config = gptimer_config_t();
  {
      // TODO: figure out the right value for resolution_hz
      timer_config.clk_src       = GPTIMER_CLK_SRC; // GPTIMER_CLK_SRC_DEFAULT or GPTIMER_CLK_SRC_PLL_F80M ; // clock source
      timer_config.direction     = GPTIMER_COUNT_UP ;              // Counting direction is up
      timer_config.resolution_hz = timer_resolution_hz ;  // Note: Preferred 40MHz, but usual resolution for oversampling USB-LS is 48 MHz, which gptimer can't do
      // timer_config.intr_priority = 16 ; // GPTimer interrupt priority, if set to 0, the driver will try to allocate an interrupt with a relative low priority (1,2,3)
      // timer_config.intr_shared = 1, // Set true, the timer interrupt number can be shared with other peripherals
      // timer_config.flags = {.allow_pd }
  };


  // Create a timer instance
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

  gptimer_alarm_config_t alarm_config = gptimer_alarm_config_t();
  {
      // TODO: figure out the right value for alarm_count
      alarm_config.reload_count = 0; // When the alarm event occurs, the timer will automatically reload to 0
      alarm_config.alarm_count = alarm_interval ; // Set the actual alarm period, since the resolution is 1us, 1000000 represents 1s
      alarm_config.flags.auto_reload_on_alarm = true ; // enable auto-reload function
  };


  // Set the timer's alarm action
  ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

  gptimer_event_callbacks_t cbs = {
      .on_alarm = ush_timer_on_alarm_cb, // Call the user callback function when the alarm event occurs
  };
  // Register timer event callback functions, allowing user context to be carried
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, this));

  // Enable the timer
  ESP_ERROR_CHECK(gptimer_enable(gptimer));
  // Start the timer
  ESP_ERROR_CHECK(gptimer_start(gptimer));

  // // Check the timer's resolution
  // uint32_t resolution_hz;
  // ESP_ERROR_CHECK(gptimer_get_resolution(gptimer, &resolution_hz));
  // // Read the current count value
  // uint64_t count;
  // ESP_ERROR_CHECK(gptimer_get_raw_count(gptimer, &count));
  // // (Optional) Convert the count value to time units (seconds)
  // double time = (double)count / resolution_hz;
  // Serial.printf("Actual timer settings: %lu Hz\n", resolution_hz);

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


void USB_SOFT_HOST::setOndisconnectCb( ondisconnectcb_t onDisconnectCB )
{
  set_ondisconnect_cb( onDisconnectCB );
}


void USB_SOFT_HOST::setOnConfigDescCB( onconfigdesccb_t cb )
{
  set_onconfigdesc_cb(cb);
}


void USB_SOFT_HOST::setOnIfaceDescCb( onifacedesccb_t cb )
{
  set_onifacedesc_cb(cb);
}


void USB_SOFT_HOST::setOnHIDDevDescCb( onhiddevdesccb_t cb )
{
  set_onhiddevdesc_cb(cb);
}


void USB_SOFT_HOST::setOnEPDescCb( onepdesccb_t cb )
{
  set_onepdesc_cb(cb);
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
    gptimer_stop(gptimer);
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
    gptimer_start(gptimer);
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
