#if !defined ARDUINO_M5Stack_Core_ESP32
#error "This sketch only works with M5Stack Core classic!"
#endif

#include <ESP32-Chimera-Core.h> // http://librarymanager#esp32-chimera-core + http://librarymanager#lovyangfx
#include <ESP32-USBSoftHost.hpp> // https://github.com/tobozo/ESP32-USB-Soft-Host
#include "usbkbd.h" // KeyboardReportParser
#include "C64_UI.h" // fancy keyboard demo
#include "kbdparser.h"

// Only one USB port is open in this demo!
// Pins configuration for this USB Soft Host:
// [OK] 16/17 : M5Bottom pins for RX2/TX2
#define DP_P0  16  // always enabled
#define DM_P0  17  // always enabled
#define DP_P1  -1
#define DM_P1  -1
#define DP_P2  -1
#define DM_P2  -1
#define DP_P3  -1
#define DM_P3  -1

KbdRptParser Prs;

// keyboard data parser to pass to the USB driver as a callback
static void m5_USB_PrintCB(uint8_t usbNum, uint8_t byte_depth, uint8_t* data, uint8_t data_len)
{
  Prs.Parse( data_len, data );
}


// USB detection callback
static void my_USB_DetectCB( uint8_t usbNum, void * dev )
{
  sDevDesc *device = (sDevDesc*)dev;
  printf("New device detected on USB#%d\n", usbNum);
  // printf("desc.bcdUSB             = 0x%04x\n", device->bcdUSB);
  // printf("desc.bDeviceClass       = 0x%02x\n", device->bDeviceClass);
  // printf("desc.bDeviceSubClass    = 0x%02x\n", device->bDeviceSubClass);
  // printf("desc.bDeviceProtocol    = 0x%02x\n", device->bDeviceProtocol);
  // printf("desc.bMaxPacketSize0    = 0x%02x\n", device->bMaxPacketSize0);
  printf("desc.idVendor           = 0x%04x\n", device->idVendor);
  printf("desc.idProduct          = 0x%04x\n", device->idProduct);
  // printf("desc.bcdDevice          = 0x%04x\n", device->bcdDevice);
  // printf("desc.iManufacturer      = 0x%02x\n", device->iManufacturer);
  // printf("desc.iSerialNumber      = 0x%02x\n", device->iSerialNumber);
  // printf("desc.bNumConfigurations = 0x%02x\n", device->bNumConfigurations);
  // if( device->iProduct == mySupportedIdProduct && device->iManufacturer == mySupportedManufacturer ) {
  //   myListenUSBPort = usbNum;
  // }
}



void setup()
{
  M5.begin();
  delay(200);
  Serial.println("USB Test");
  delay(1000);

  tft.fillScreen( bgcolor );
  tft.setTextSize(2);
  tft.setFont( &Font8x8C64 );
  tft.setTextColor( fgcolor, bgcolor );
  tft.setTextDatum( TL_DATUM );

  // fortunately this is a monotype font :-)
  fontWidth  = tft.fontHeight(&Font8x8C64);
  fontHeight = tft.fontHeight(&Font8x8C64);
  // guess screenBuffer width/height from display mode
  screenColumns = tft.width() / fontWidth;
  screenRows    = tft.height() / fontHeight;
  screenBuffer = (char*)calloc( (screenColumns*screenRows)+1, sizeof(char) );
  if( screenBuffer == NULL ) {
    Serial.println("Can't allocate screen buffer, aborting");
    while(1);
  }

  const char* l1 = " **M5Stack USBHID**";
  const char* l2 = "READY.";

  // store in screen buffer the text we're going to print
  memcpy( &screenBuffer[screenColumns*1], l1, strlen(l1) );
  memcpy( &screenBuffer[screenColumns*3], l2, strlen(l2) );
  // draw the decoration text
  tft.drawString(l1, 0, fontHeight );
  tft.drawString(l2, 0, fontHeight*3 );
  // set the cursor to the next line
  cursorX = 0;
  cursorY = fontHeight*4;
  tft.setCursor(cursorX, cursorY );
  // init blinky
  blinkCursor();
  // init the USB Host
  USH.init(
    (usb_pins_config_t){
      DP_P0, DM_P0,
      DP_P1, DM_P1,
      DP_P2, DM_P2,
      DP_P3, DM_P3
    },
    my_USB_DetectCB,
    m5_USB_PrintCB,
    blinkCursor
  );
}



void loop()
{
  vTaskDelete(NULL);
}
