#define DEBUG_ALL
//#define FORCE_TEMPLATED_NOPS
#include <ESP32-USB-Soft-Host.h>

#if defined ARDUINO_LOLIN_S3 || defined ARDUINO_LOLIN_S3_PRO

  #define PROFILE_NAME "LoLin S3"
  #define DP_P0  15  // always enabled
  #define DM_P0  16  // always enabled
  #define DP_P1  -1
  #define DM_P1  -1
  #define DP_P2  -1
  #define DM_P2  -1
  #define DP_P3  -1
  #define DM_P3  -1

#elif defined ARDUINO_LOLIN_D32_PRO
  // [KO] 15/13 : unassigned pins seem unresponsive
  // [OK] 22/23 : pins for MOSI/SCK work just fine
  #define PROFILE_NAME "LoLin D32 Pro"
  #define DP_P0  22  // always enabled
  #define DM_P0  21  // always enabled
  #define DP_P1  -1
  #define DM_P1  -1
  #define DP_P2  -1
  #define DM_P2  -1
  #define DP_P3  -1
  #define DM_P3  -1
#elif defined ARDUINO_M5STACK_Core2
  // [OK] 33/32 are the GROVE port pins for I2C, but there are other devices interferring on the bus
  #define PROFILE_NAME "M5Stack Core2"
  #define DP_P0  33  // always enabled
  #define DM_P0  32  // always enabled
  #define DP_P1  -1
  #define DM_P1  -1
  #define DP_P2  -1
  #define DM_P2  -1
  #define DP_P3  -1
  #define DM_P3  -1
#elif defined ARDUINO_M5STACK_FIRE
  // [KO] 16/17 : GROVE port pins for RX2/TX2 but seem unresponsive
  // [KO] 21/22 : GROVE port pins for SDA/SCL, but there are other devices interferring on the bus
  // [KO] 26/36 : GROVE port pins for I/O but seem unresponsive
  #define PROFILE_NAME "M5Stack Fire"
  #define DP_P0  16  // always enabled
  #define DM_P0  17  // always enabled
  #define DP_P1  -1
  #define DM_P1  -1
  #define DP_P2  -1
  #define DM_P2  -1
  #define DP_P3  -1
  #define DM_P3  -1
#elif defined ARDUINO_M5Stack_Core_ESP32
  // [OK] 16/17 : M5Bottom pins for RX2/TX2 work just fine
  #define PROFILE_NAME "M5Stack Gray"
  #define DP_P0  16  // always enabled
  #define DM_P0  17  // always enabled
  #define DP_P1  -1
  #define DM_P1  -1
  #define DP_P2  -1
  #define DM_P2  -1
  #define DP_P3  -1
  #define DM_P3  -1
#elif defined CONFIG_IDF_TARGET_ESP32C3
  #define PROFILE_NAME "ESP32 C3 Dev module"
  #define DP_P0   6
  #define DM_P0   8
  #define DP_P1  -1
  #define DM_P1  -1
  #define DP_P2  -1
  #define DM_P2  -1
  #define DP_P3  -1
  #define DM_P3  -1
#elif defined CONFIG_IDF_TARGET_ESP32S2
  #define LOAD_TINYUSB
  #define PROFILE_NAME "ESP32 S2 Dev module"
  // [/!\] 20/19 = (USB RS/TS), might be unresponsive
  #define DP_P0  20 // ok ESP32-S2-Kaluga
  #define DM_P0  19 // ok ESP32-S2-Kaluga
  #define DP_P1  9  // ok ESP32-S2-Kaluga
  #define DM_P1  8  // ok ESP32-S2-Kaluga
  #define DP_P2  11 // ok ESP32-S2-Kaluga
  #define DM_P2  10 // ok ESP32-S2-Kaluga
  #define DP_P3  13 // ok ESP32-S2-Kaluga
  #define DM_P3  12 // ok ESP32-S2-Kaluga
#elif defined ARDUINO_M5STACK_TAB5
  #include <M5Unified.h> // Tab5 need some registers poked to power the PortA
  #define PROFILE_NAME "M5Tab5 / ESP32 P4"
  #define DP_P0   53 // porta
  #define DM_P0   54 // porta
  #define DP_P1   -1//47 // M-Bus (free)
  #define DM_P1   -1//48 // M-Bus (free)
  #define DP_P2   -1//17 // M-Bus (free)
  #define DM_P2   -1//16 // M-Bus (PC_IN)
  #define DP_P3   -1//19 // M-Bus (MOSI)
  #define DM_P3   -1//18 // M-Bus (MISO)
#elif defined ARDUINO_M5STACK_CORES3
  #define PROFILE_NAME "M5Stack CoreS3 (PORTB)"
  #define DP_P0   9 // portb
  #define DM_P0   8 // portb
  #define DP_P1  -1 // 1 // porta
  #define DM_P1  -1 // 2 // porta
  #define DP_P2  -1 // 18 // portc
  #define DM_P2  -1 // 17 // portc
  #define DP_P3  -1
  #define DM_P3  -1
#else // defined CONFIG_IDF_TARGET_ESP32
  // default pins tested on ESP32-Wroom
  #define PROFILE_NAME "Default Wroom"
  #define DP_P0  16  // always enabled
  #define DM_P0  17  // always enabled
  #define DP_P1  22 // -1 to disable
  #define DM_P1  23 // -1 to disable
  #define DP_P2  18 // -1 to disable
  #define DM_P2  19 // -1 to disable
  #define DP_P3  13 // -1 to disable
  #define DM_P3  15 // -1 to disable
#endif


static void my_USB_DetectCB( uint8_t usbNum, void * dev )
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
  printf("desc.iProduct           = 0x%02x\n", device->iProduct);
  printf("desc.iSerialNumber      = 0x%02x\n", device->iSerialNumber);
  printf("desc.bNumConfigurations = 0x%02x\n", device->bNumConfigurations);
  // if( device->iProduct == mySupportedIdProduct && device->iManufacturer == mySupportedManufacturer ) {
  //   myListenUSBPort = usbNum;
  // }
}


static void my_USB_PrintCB(uint8_t usbNum, uint8_t byte_depth, uint8_t* data, uint8_t data_len)
{
  // if( myListenUSBPort != usbNum ) return;
  printf("in: ");
  for(int k=0;k<data_len;k++) {
    printf("0x%02x ", data[k] );
  }
  printf("\n");
}


static void onDisconnectCb( uint8_t portnum )
{
  printf("USB#%d disconnected\n", portnum);
}


usb_pins_config_t USB_Pins_Config =
{
  DP_P0, DM_P0,
  DP_P1, DM_P1,
  DP_P2, DM_P2,
  DP_P3, DM_P3
};


void setup()
{
  //Serial.begin(115200);

  #if __has_include(<M5Unified.h>)
    M5.begin();
    // Explicitly enable 5V on M5-Bus (along with HY2.0-4P & 2.54-10P)
    M5.Power.setExtOutput(true, m5::ext_port_mask_t::ext_PA);
  #endif

  delay(1000);

  printf("USB Soft Host Test for %s\n", PROFILE_NAME );
  // USH.setTaskCore( 0 );
  // USH.setBlinkPin( (gpio_num_t) 2 );
  // USH.setTaskPriority( 16 );
  USH.setOnConfigDescCB( Default_USB_ConfigDescCB );
  USH.setOnIfaceDescCb( Default_USB_IfaceDescCb );
  USH.setOnHIDDevDescCb( Default_USB_HIDDevDescCb );
  USH.setOnEPDescCb( Default_USB_EPDescCb );
  USH.setOndisconnectCb( onDisconnectCb );

  USH.init( USB_Pins_Config, my_USB_DetectCB, my_USB_PrintCB );
}

void loop()
{
  vTaskDelete(NULL);
}


