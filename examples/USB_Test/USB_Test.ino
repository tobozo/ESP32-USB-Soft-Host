
#include <ESP32-USBSoftHost.hpp>

// set pin numbers here
#define DP_P0  16  // always enabled
#define DM_P0  17  // always enabled
#define DP_P1  22 // -1 to disable
#define DM_P1  23 // -1 to disable
#define DP_P2  18 // -1 to disable
#define DM_P2  19 // -1 to disable
#define DP_P3  13 // -1 to disable
#define DM_P3  15 // -1 to disable

usb_pins_config_t USB_Pins_Config =
{
  DP_P0, DM_P0,
  DP_P1, DM_P1,
  DP_P2, DM_P2,
  DP_P3, DM_P3
};

static void USB_SoftHostTimerTask(void *arg)
{
  while (1) {
    timer_event_t evt;
    xQueueReceive(timer_queue, &evt, portMAX_DELAY);
    printState();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

static void my_USB_PrintCB( uint8_t* data, uint8_t data_len )
{
  printf("in :");
  for(int k=0;k<data_len;k++) {
    printf("%02x ",data[k]);
  }
  printf("\n");
}

void setup()
{

  setPrintCb( &my_USB_PrintCB );
  usb_init( USB_Pins_Config );

  xTaskCreatePinnedToCore(&USB_SoftHostTimerTask, "USB Soft Host Timer Task", 4096, NULL, 5, NULL, 1);

}

void loop()
{
  vTaskDelete(NULL);
}


