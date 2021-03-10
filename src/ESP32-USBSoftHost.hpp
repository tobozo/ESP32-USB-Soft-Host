#ifndef __USB_SOFT_HOST_HPP_
#define __USB_SOFT_HOST_HPP_

#ifndef BLINK_GPIO
#define BLINK_GPIO 2
#endif


extern "C" {
  #include <usb_host.h>
}


typedef struct
{
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;

static void IRAM_ATTR timer_group0_isr(void *para)
{
  int timer_idx = (int) para;
  // Retrieve the interrupt status and the counter value from the timer that reported the interrupt
  uint32_t intr_status = TIMERG0.int_st_timers.val;
  TIMERG0.hw_timer[timer_idx].update = 1;
  uint64_t timer_counter_value = ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32 | TIMERG0.hw_timer[timer_idx].cnt_low;
  /* Prepare basic event data that will be then sent back to the main program task */
  timer_event_t evt;
  evt.timer_group = 0;
  evt.timer_idx = timer_idx;
  evt.timer_counter_value = timer_counter_value;
  // process USB signal
  usb_process();
  // Clear the interrupt and update the alarm time for the timer with without reload
  if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
    evt.type = 1; // no reload
    TIMERG0.int_clr_timers.t0 = 1;
    timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
    TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
    TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
  }/* else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
    evt.type = 1; // reload
    TIMERG0.int_clr_timers.t1 = 1;
  } else {
    evt.type = -1; // not supported even type
  }*/
  // After the alarm has been triggered we need enable it again, so it is triggered the next time
  TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
  // Now just send the event data back to the main program task
  xQueueSendFromISR(timer_queue, &evt, NULL);
}

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


static void usb_init( usb_pins_config_t pconf )
{
  timer_config_t config;
  config.divider     = TIMER_DIVIDER;
  config.counter_dir = TIMER_COUNT_UP;
  config.counter_en  = TIMER_PAUSE;
  config.alarm_en    = TIMER_ALARM_EN;
  config.auto_reload = 1;

  timer_queue = xQueueCreate(10, sizeof(timer_event_t));

  initStates(
    (gpio_num_t)pconf.dp0, (gpio_num_t)pconf.dm0,
    (gpio_num_t)pconf.dp1, (gpio_num_t)pconf.dm1,
    (gpio_num_t)pconf.dp2, (gpio_num_t)pconf.dm2,
    (gpio_num_t)pconf.dp3, (gpio_num_t)pconf.dm3
  );

  gpio_pad_select_gpio((gpio_num_t)BLINK_GPIO);
  gpio_set_direction((gpio_num_t)BLINK_GPIO, GPIO_MODE_OUTPUT);

  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, (double)TIMER_INTERVAL0_SEC * TIMER_SCALE);
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr, (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);
  timer_start(TIMER_GROUP_0, TIMER_0);
}


#endif
