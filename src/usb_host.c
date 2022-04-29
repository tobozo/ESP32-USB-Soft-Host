#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>


#include "usb_host.h"
#ifdef ESP32
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/timer.h"
#include "soc/soc.h"
#include "soc/rtc.h"
#include "math.h"
#include "esp_heap_caps.h"
#include "esp32-hal.h"

inline uint32_t hal_get_cpu_mhz(void)
{
  rtc_cpu_freq_config_t  out_config;
  rtc_clk_cpu_freq_get_config(&out_config);
  return out_config.freq_mhz;
}

#ifdef TIMER_INTERVAL0_SEC
typedef void (*timer_isr_t)(void *para);

void hal_timer_setup(timer_idx_t timer_num, uint32_t alarm_value, timer_isr_t timer_isr)
{
  timer_config_t config;
  config.divider     = TIMER_DIVIDER;
  config.counter_dir = TIMER_COUNT_UP;
  config.counter_en  = TIMER_PAUSE;
  config.alarm_en    = TIMER_ALARM_EN;
  config.auto_reload = (timer_autoreload_t) 1; // fix for ¬invalid conversion from 'int' to 'timer_autoreload_t'¬ thanks rudi ;-)

  timer_init(TIMER_GROUP_0, timer_num, &config);
  timer_set_counter_value(TIMER_GROUP_0, timer_num, 0x00000000ULL);
  timer_set_alarm_value(TIMER_GROUP_0, timer_num, alarm_value);
  timer_enable_intr(TIMER_GROUP_0, timer_num);
  timer_isr_register(TIMER_GROUP_0, timer_num, timer_isr, (void *) timer_num, ESP_INTR_FLAG_IRAM, NULL);
  timer_start(TIMER_GROUP_0, timer_num);
}
#endif

#define cpu_hal_get_cycle_count xthal_get_ccount
#else
#ifdef __IMXRT1062__
#include <Arduino.h>
#else
#include <generated/csr.h>
#endif
#endif //ESP32

#ifdef USE_TUSB_FIFO
hal_queue_handle_t hal_queue_create(size_t n, size_t sz, void *buffer)
{
  tu_fifo_t f;
  memset(&f, 0, sizeof(f));
  tu_fifo_config(&f, buffer, n, sz, true);
  return f;
}
#endif


/*******************************
*    warning!!!: any copy of this code or his part must include this:
*  "The original was written by Dima Samsonov @ Israel sdima1357@gmail.com on 3/2021" *
*  Copyright (C) 2021  Dmitry Samsonov *
********************************/

/*\
   Changes by tobozo (sept 2021)
     - Backported commits from https://github.com/sdima1357/esp32_usb_soft_host/commit/7c51138ec1ea37d4dd5363d3f25a8e7ab8716871
     - ESP32-C3 can't work with Arduino IDE ( CONFIG_ESP_SYSTEM_MEMPROT_FEATURE=y ), C++ wrapper still valid through platformio/esp-idf though
   Changes by tobozo (may 2021):
     - Backported calibration at init (last changes from from Samsonov's repo)
   Changes by tobozo (march 2021):
     - Arduino IDE compliance (mostly code regression to esp-idf 3.x)
     - Added callbacks (data and device detection)
\*/

// Arduino IDE complains about volatile at init, but we don't care
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"


#define T_START   0b00000001
#define T_ACK     0b01001011
#define T_NACK    0b01011010
#define T_SOF    0b10100101
#define T_SETUP  0b10110100
#define T_DATA0  0b11000011
#define T_DATA1  0b11010010
#define T_DATA2  0b11100001
#define T_OUT    0b10000111
#define T_IN    0b10010110

#define T_ERR    0b00111100
#define T_PRE    0b00111100
#define T_NYET  0b01101001
#define T_STALL  0b01111000

// local non std
#define T_NEED_ACK   0b01111011
#define T_CHK_ERR    0b01111111

#define USB_LS_K  0
#define USB_LS_J  1
#define USB_LS_S  2

//most counters- uint_8t :  so  prevents overflow...

#define DEF_BUFF_SIZE 0x100

// somethins short  like ACK
#define SMALL_NO_DATA 36




int TRANSMIT_TIME_DELAY = 110;  //delay each bit transmit
int TIME_MULT           = 25;    //received time factor delta clocks* TIME_MULT/TIME_SCALE
int TM_OUT              = 64;    //receive time out no activity on bus
#define  TIME_SCALE       1024

//#define TEST
#ifdef TEST
  #define TOUT  1000
#else
  #define TOUT  (TM_OUT)
#endif


#if defined(ESP32) || defined(__IMXRT1062__)
//aproximate scale depending on CPU clock frequency
#if   F_CPU <   50*1000000
#define TIME_FACTOR_BITS 0
#elif F_CPU <  100*1000000
#define TIME_FACTOR_BITS 1
#elif F_CPU <  200*1000000
#define TIME_FACTOR_BITS 2
#elif F_CPU <  400*1000000
#define TIME_FACTOR_BITS 3
#elif F_CPU <  800*1000000
#define TIME_FACTOR_BITS 4
#elif F_CPU < 1600*1000000
#define TIME_FACTOR_BITS 5
#else
#define TIME_FACTOR_BITS 6
#endif
#else
#define TIME_FACTOR_BITS 1 //vexriscv 100Mhz
#endif

static uint32_t _getCycleCount32(void)
{
  uint32_t ccount = cpu_hal_get_cycle_count();
  return  ccount;
}
static uint8_t _getCycleCount8d8(void)
{
  uint32_t ccount = _getCycleCount32();
  return ccount>>TIME_FACTOR_BITS;
}


#if 0//def ESP32

//#define SE_J  { *snd[1][0] = (1 << DM_PIN);*snd[1][1] = (1 << DP_PIN); } //clear / set
//#define SE_0  { *snd[2][0] = (1 << DM_PIN);*snd[2][1] = (1 << DP_PIN); } //clear / clear

#if CONFIG_IDF_TARGET_ESP32
  #define SET_I(dp, dm) { PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[dp]); PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[dm]); GPIO.enable_w1tc = (1 << (dp)) | (1 << (dm));  }
  #define SET_O(dp, dm) { GPIO.enable_w1ts = (1 << (dp)) | (1 << (dm));  PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[dp]); PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[dm]);  }
  #define READ_BOTH_PINS (((GPIO.in&RD_MASK)<<8)>>RD_SHIFT)
  uint32_t * snd[4][2]  =
  {
    {&GPIO.out_w1tc,&GPIO.out_w1ts}, //clear / set
    {&GPIO.out_w1ts,&GPIO.out_w1tc}, //set   / clear
    {&GPIO.out_w1tc,&GPIO.out_w1tc}, //clear / clear
    {&GPIO.out_w1tc,&GPIO.out_w1tc}  //clear / clear
  } ;
#else
  //gpio_ll_output_disable: hw->enable_w1tc = (0x1 << gpio_num);
  //GPIO_PIN_MUX_REG[0]: IO_MUX_GPIO0_REG
  //PIN_INPUT_ENABLE(PIN_NAME): SET_PERI_REG_MASK(PIN_NAME,FUN_IE)
  #define SET_I(dp, dm) { PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[dp]); PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[dm]);  gpio_ll_output_disable(&GPIO,dm); gpio_ll_output_disable(&GPIO,dp);}
  #define SET_O(dp, dm) { GPIO.enable_w1ts.val = (1 << (dp)) | (1 << (dm));  PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[dp]); PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[dm]);  }
  #define READ_BOTH_PINS (((GPIO.in.val&RD_MASK)<<8)>>RD_SHIFT)
  uint32_t * snd[4][2]  =
  {
    {&GPIO.out_w1tc.val,&GPIO.out_w1ts.val},
    {&GPIO.out_w1ts.val,&GPIO.out_w1tc.val},
    {&GPIO.out_w1tc.val,&GPIO.out_w1tc.val},
    {&GPIO.out_w1tc.val,&GPIO.out_w1tc.val}
  } ;

#endif
#endif
#if defined(__IMXRT1062__) || defined(ESP32)
#warning assumes DM_PIN > DP_PIN
#endif
#ifndef READ_BOTH_PINS
#define READ_BOTH_PINS ((hal_gpio_read(DM_PIN) ? 1<<(8+DM_PIN - DP_PIN) : 0) | (hal_gpio_read(DP_PIN) ? 0x100 : 0))
#endif
#ifndef SET_I
#define SET_I(dp, dm)  { hal_gpio_set_direction(dp, 0); hal_gpio_set_direction(dm, 0); }
#endif
#ifndef SET_O
#define SET_O(dp, dm)  { hal_gpio_set_direction(dp, 1); hal_gpio_set_direction(dm, 1); }
#endif
#ifndef SE_J 
#define SE_J { hal_gpio_set_level(DM_PIN, 0); hal_gpio_set_level(DP_PIN, 1); } //clear / set
#endif
#ifndef SE_0 
#define SE_0 { hal_gpio_set_level(DM_PIN, 0); hal_gpio_set_level(DP_PIN, 0); } //clear / clear
#endif


#ifndef hal_set_differential_gpio_value
#define hal_set_differential_gpio_value(dp, dm,v) \
{ hal_gpio_set_level(dm, v & 1); /*v==0 => clear, v==1 => set,   v == 2 => clear*/ \
    hal_gpio_set_level(dp, v == 0); } /*v==0 => set  , v==1 => clear, v == 2 => clear */
#endif

//must be setup ech time with setPins
uint32_t DP_PIN;
uint32_t DM_PIN;

uint32_t DM_PIN_M;
uint32_t DP_PIN_M;
uint16_t M_ONE;
uint16_t P_ONE;
uint32_t RD_MASK;
uint32_t RD_SHIFT;
//end must be setup ech time with setPins

// temporary used insize lowlevel
volatile uint8_t received_NRZI_buffer_bytesCnt;
uint16_t received_NRZI_buffer[DEF_BUFF_SIZE];

volatile uint8_t transmit_bits_buffer_store_cnt;
//uint8_t transmit_bits_buffer_store[DEF_BUFF_SIZE];
uint8_t* transmit_bits_buffer_store = (uint8_t*)&received_NRZI_buffer[0];

volatile uint8_t transmit_NRZI_buffer_cnt;
uint8_t transmit_NRZI_buffer[DEF_BUFF_SIZE];

volatile uint8_t decoded_receive_buffer_head;
volatile uint8_t decoded_receive_buffer_tail;
uint8_t decoded_receive_buffer[DEF_BUFF_SIZE];
// end temporary used insize lowlevel

#ifdef ESP32
void (*delay_pntA)() = NULL;
#define cpuDelay(x) {(*delay_pntA)();}


#if CONFIG_IDF_TARGET_ESP32
  #define MAX_DELAY_CODE_SIZE 0x280
  #define SDELAYMALLOC(X) malloc(X)
  #define SDELAYREALLOC(X,Y,Z) heap_caps_realloc(X,Y,Z)
  #define SDELAYRMASK MALLOC_CAP_EXEC
  void makeOpcodes( uint8_t* ptr, uint8_t ticks )
  {
    //put head of delay procedure
    *ptr++ = 0x36;
    *ptr++ = 0x41;
    *ptr++ = 0;
    for(int k=0;k<ticks;k++) {
      //put NOPs
      *ptr++ = 0x3d;
      *ptr++ = 0xf0;
    }
    //put tail of delay procedure
    *ptr++ = 0x1d;
    *ptr++ = 0xf0;
    *ptr++ = 0x00;
    *ptr++ = 0x00;
  }
#else // ESP32-C3
  #define MAX_DELAY_CODE_SIZE 0x210
  #define SDELAYMALLOC(X) heap_caps_aligned_alloc(X,MAX_DELAY_CODE_SIZE, MALLOC_CAP_8BIT);
  #define SDELAYREALLOC(X,Y,Z) heap_caps_realloc(X,Y,Z)
  #define SDELAYRMASK MALLOC_CAP_32BIT | MALLOC_CAP_EXEC
  void makeOpcodes( uint8_t* ptr, uint8_t ticks )
  {
    //put head of delay procedure
    for(int k=0;k<ticks;k++) {
      //put NOPs
      *ptr++ = 0x1;
      *ptr++ = 0x0;
    }
    //put tail of delay procedure
    *ptr++ = 0x82;
    *ptr++ = 0x80;
  }
#endif


void setDelay(uint16_t ticks)
{
  uint8_t* pntS;
  if(ticks > MAX_DELAY_CODE_SIZE)
    ticks = MAX_DELAY_CODE_SIZE;
  // it can't execute but can read & write
  if(!delay_pntA) {
    pntS = SDELAYMALLOC( MAX_DELAY_CODE_SIZE );
  } else {
    pntS = SDELAYREALLOC( delay_pntA, MAX_DELAY_CODE_SIZE, MALLOC_CAP_8BIT );
  }
  uint8_t* pnt = (uint8_t*)pntS;
  makeOpcodes( pnt, ticks );
  // move it to executable memory segment
  // it can't  write  but can read & execute
  delay_pntA = SDELAYREALLOC( pntS, MAX_DELAY_CODE_SIZE, SDELAYRMASK );
  if(!delay_pntA) {
    printf("idf.py menuconfig\n Component config-> ESP System Setting -> Memory protectiom-> Disable.\n memory prot must be disabled!!!\n delay_pntA = %p,\nHalting...\n",delay_pntA);
    while(1) { vTaskDelay(1); }
  }
}
#else //not ESP32

void setDelay(uint16_t ticks) {}
void cpuDelay(uint16_t x)
{
 uint32_t t0 = cpu_hal_get_cycle_count();
 for(;;)
 {
   uint32_t t1 =  cpu_hal_get_cycle_count();
   t1 -= t0;
   if(t1 > x)
     break;
  }
}
#endif



typedef struct
{
  uint8_t cmd;
  uint8_t addr;
  uint8_t eop;

  uint8_t  dataCmd;
  uint8_t  bmRequestType;
  uint8_t  bmRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLen;
} Req;



enum DeviceState
{
  NOT_ATTACHED,
  ATTACHED,
  POWERED,
  DEFAULT,
  ADDRESS,
  PARSE_CONFIG,
  PARSE_CONFIG1,
  PARSE_CONFIG2,
  PARSE_CONFIG3,
  POST_ATTACHED,
  RESET_COMPLETE,
  POWERED_COMPLETE,
  DEFAULT_COMPL
};



enum CallbackCmd
{
  CB_CHECK,
  CB_RESET,
  CB_WAIT0,
  CB_POWER,
  CB_TICK,
  CB_2,
  CB_2Ack,
  CB_3,
  CB_4,
  CB_5,
  CB_6,
  CB_7,
  CB_8,
  CB_9,
  CB_WAIT1
};



//Req rq;
typedef struct
{
  int isValid;
  int selfNum;
  int epCount;
  int cnt;

  uint8_t flags_new;
  uint8_t flags;

  uint32_t DP;
  uint32_t DM;

  volatile enum CallbackCmd cb_Cmd;
  volatile enum DeviceState fsm_state;
  volatile uint16_t wires_last_state;

  sDevDesc desc;
  sCfgDesc cfg;
  Req rq;

  int counterNAck;
  int counterAck;

  uint8_t descrBuffer[DEF_BUFF_SIZE];
  uint8_t descrBufferLen;

  volatile int bComplete;
  volatile int in_data_flip_flop;

  int cmdTimeOut;

  uint32_t ufPrintDesc;
  int numb_reps_errors_allowed;

  uint8_t acc_decoded_resp[DEF_BUFF_SIZE];
  uint8_t acc_decoded_resp_counter;

  int asckedReceiveBytes;
  int transmitL1Bytes;
  uint8_t transmitL1[DEF_BUFF_SIZE];

} sUsbContStruct;

sUsbContStruct * current;
void usb_disable_current(void) { current->isValid = 0; }

void parseImmed(sUsbContStruct * pcurrent)
{
  static sCfgDesc      cfg;
  static sIntfDesc     sIntf;
  static HIDDescriptor hid[4];
  static sEPDesc       epd;
  /*static int           cfgCount   = 0;
  static int           sIntfCount   = 0;*/
  static int           hidCount   = 0;
  int                  pos = 0;
  #define STDCLASS     0x00
  #define HIDCLASS     0x03
  #define HUBCLASS     0x09      /* bDeviceClass, bInterfaceClass */

  pcurrent->epCount     = 0;

  while(pos<pcurrent->descrBufferLen-2) {
    uint8_t len  =  pcurrent->descrBuffer[pos];
    uint8_t type =  pcurrent->descrBuffer[pos+1];
    if(len==0) {
      //printf("pos = %02x type = %02x cfg.wLength = %02x pcurrent->acc_decoded_resp_counter = %02x\n ",pos,type,cfg.wLength,pcurrent->acc_decoded_resp_counter);
      pos = pcurrent->descrBufferLen;
    }
    if(pos+len<=pcurrent->descrBufferLen) {
      if(type == 0x2) {
        memcpy(&cfg,&pcurrent->descrBuffer[pos],len);

      } else if (type == 0x4) {
        memcpy(&sIntf,&pcurrent->descrBuffer[pos],len);
      } else if (type == 0x21) {
        hidCount++;
        int i = hidCount-1;
        memcpy(&hid[i],&pcurrent->descrBuffer[pos],len);
      } else if (type == 0x5) {
        pcurrent->epCount++;
        memcpy(&epd,&pcurrent->descrBuffer[pos],len);
      }
    }
    pos+=len;
  }
}

#ifdef WR_SIMULTA
uint32_t sndA[4]  = {0,0,0,0};
#endif



void restart(void)
{
  transmit_NRZI_buffer_cnt = 0;
}



void decoded_receive_buffer_clear(void)
{
  decoded_receive_buffer_tail = decoded_receive_buffer_head;
}



void decoded_receive_buffer_put(uint8_t val)
{
  decoded_receive_buffer[decoded_receive_buffer_head] = val;
  decoded_receive_buffer_head++;
}



uint8_t decoded_receive_buffer_get(void)
{
  return decoded_receive_buffer[decoded_receive_buffer_tail++];
}



uint8_t decoded_receive_buffer_size(void)
{
  return (uint8_t )(decoded_receive_buffer_head-decoded_receive_buffer_tail);
}



uint8_t cal5(void)
{
  uint8_t   crcb;
  uint8_t   rem;

  crcb = 0b00101;
  rem =  0b11111;

  for(int k=16;k<transmit_bits_buffer_store_cnt;k++) {
    int rb = (rem>>4)&1;
    rem   =  (rem<<1)&0b11111;

    if(rb^(transmit_bits_buffer_store[k]&1)) {
      rem ^= crcb;
    }
  }
  return (~rem)&0b11111;
}



uint32_t cal16(void)
{
  uint32_t   crcb;
  uint32_t   rem;

  crcb = 0b1000000000000101;
  rem =  0b1111111111111111;

  for(int k=16;k<transmit_bits_buffer_store_cnt;k++) {
    int rb = (rem>>15)&1;
    rem   =  (rem<<1)&0b1111111111111111;

    if(rb^(transmit_bits_buffer_store[k]&1)) {
      rem ^= crcb;
    }
  }
  return (~rem)&0b1111111111111111;
}



void seB(int bit)
{
  transmit_bits_buffer_store[transmit_bits_buffer_store_cnt++] = bit;
}



void pu_MSB(uint16_t msg,int N)
{
  for(int k=0;k<N;k++) {
    seB(msg&(1<<(N-1-k))?1:0);
  }
}



void pu_LSB(uint16_t msg,int N)
{
  for(int k=0;k<N;k++) {
    seB(msg&(1<<(k))?1:0);
  }
}



void repack(void)
{
  int last = USB_LS_J;
  int cntOnes = 0;
  transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_J;
  for(int k=0;k<transmit_bits_buffer_store_cnt;k++) {
    if(transmit_bits_buffer_store[k]==0) {
      if(last==USB_LS_J||last==USB_LS_S) {
        last = USB_LS_K;
      } else {
        last = USB_LS_J;
      }
      cntOnes = 0;
    } else if(transmit_bits_buffer_store[k]==1) {
      cntOnes++;
      if(cntOnes==6) {
        transmit_NRZI_buffer[transmit_NRZI_buffer_cnt] = last;
        transmit_NRZI_buffer_cnt++;
        if(last==USB_LS_J) {
          last = USB_LS_K;
        } else {
          last = USB_LS_J;
        }
        cntOnes = 0;
      }
      if(last==USB_LS_S) {
        last = USB_LS_J;
      }
    }
    transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = last;
  }

  transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_S;
  transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_S;

  transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_J;
  transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_J;
  transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_J;
  transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_J;

  transmit_bits_buffer_store_cnt = 0;
}



uint8_t rev8(uint8_t j)
{
  uint8_t res = 0;
  for(int i=0;i<8;i++) {
    res<<=1;
    res|=(j>>i)&1;
  }
  return res;
}



uint16_t rev16(uint16_t j)
{
  uint16_t res = 0;
  for(int i=0;i<16;i++) {
    res<<=1;
    res|=(j>>i)&1;
  }
  return res;
}



#ifdef DEBUG_ALL
uint16_t debug_buff[0x100];
#endif


void (*onLedBlinkCB)(int on_off) = NULL;
#define NOTIFY() if(onLedBlinkCB) onLedBlinkCB(1)


int parse_received_NRZI_buffer(void)
{

  if(!received_NRZI_buffer_bytesCnt) return 0;

  uint32_t   crcb;
  uint32_t   rem;

  crcb = 0b1000000000000101;
  rem =  0b1111111111111111;

  int  res = 0;
  int  cntOnes = 0;

  int terr  = 0;
  uint8_t  current_res = 0xfe;
  uint16_t prev = received_NRZI_buffer[0];
  int      start = -1;
  uint8_t  prev_smb = M_ONE;
#ifdef DEBUG_ALL
  debug_buff[0] = received_NRZI_buffer_bytesCnt;
  uint8_t rcnt = 1;
  debug_buff[received_NRZI_buffer_bytesCnt] = 0xff;
#endif
  for(int i = 1;i<received_NRZI_buffer_bytesCnt;i++) {
    //define 2.5
    uint16_t curr = (prev&0xff00) + (((received_NRZI_buffer[i] - prev))&0xff);
    prev          = received_NRZI_buffer[i];

    uint8_t smb = curr>>8;
    int tm = (curr&0xff);
    //debug_buff[i] = tm | (smb<<8);
    if( tm<2  || (smb == 0) ) {
      //terr+=tm<4?tm : 4;
      terr+=tm;
    } else {
      //terr = 0;
      int delta = ((((curr+terr)&0xff))*TIME_MULT+TIME_SCALE/2)/TIME_SCALE;

      for(int k=0;k<delta;k++) {
        int incc = 1;

        if(prev_smb!=smb) {
          if(cntOnes!=6) {
            current_res = current_res*2+0;
          } else {
            incc = 0;
          }
          cntOnes = 0;
        } else {
          current_res = current_res*2+1;
          cntOnes++;
        }
        if(start>=0) {
          start+=incc;
        }
        if(current_res==0x1 && start<0 ) {
          start = 0;
        }
        if( (start&0x7) == 0 && incc) {
          if(start==8) {
            res = current_res;
          }
          #ifdef DEBUG_ALL
            debug_buff[rcnt++] = current_res;
          #endif
    //NOTIFY();
          decoded_receive_buffer_put(current_res);
          if(start>8) {
            for(int bt =0;bt<8;bt++) {
              int rb = (rem>>15)&1;
              rem   =  (rem<<1)&0b1111111111111111;
              if(rb^((current_res>>(7-bt))&1)) {
                rem ^= crcb;
              }
            }
          }
        }

        prev_smb = smb;
      }
      terr = 0;
    }
  }
  #ifdef DEBUG_ALL
    debug_buff[rcnt++] = 0xff;
  #endif
  rem &=0b1111111111111111;
  if (rem==0b1111111111111111) {
    return res;
  }
  if(rem==0x800d) {
    return  T_NEED_ACK;
  } else {
    return  T_CHK_ERR;
  }
}



//#define WR_SIMULTA
void sendOnly(void)
{
  SET_O(DP_PIN, DM_PIN);
  #ifdef WR_SIMULTA
    uint32_t out_base = GPIO.out;
    sndA[0] = (out_base | DP) &~DM;
    sndA[1] = (out_base | DM) &~DP;
    sndA[2] = (out_base )&~(DP | DM);
    sndA[3] = out_base | (DM | DP);
  #endif
#if defined(ESP32) || defined(__IMXRT1062__)
#define TIMING_PREC 4 //add precision
#endif

#ifndef TIMING_PREC
  uint8_t k;
  #pragma GCC unroll 0
  for(k=0;k<transmit_NRZI_buffer_cnt;++k) {
    cpuDelay(TRANSMIT_TIME_DELAY);
    hal_set_differential_gpio_value(DP_PIN, DM_PIN, transmit_NRZI_buffer[k]);
  }
#else
  #pragma GCC unroll 0
  for(int k=0, td = 0, tdk=0, t1 = cpu_hal_get_cycle_count();;) {
    if((int)(cpu_hal_get_cycle_count() - t1) < tdk) continue;
    hal_set_differential_gpio_value(DP_PIN, DM_PIN, transmit_NRZI_buffer[k]);
    td += TRANSMIT_TIME_DELAY;
    tdk = td/TIMING_PREC;
    if(++k>=transmit_NRZI_buffer_cnt) break;
  }
#endif
  restart();
  SET_I(DP_PIN, DM_PIN);
}

void sendRecieveNParse(void)
{
  register uint32_t R3;
  register uint16_t *STORE = received_NRZI_buffer;
  //hal_disable_irq(); //this is already in IRQ handler
  sendOnly();
  register uint32_t R4;// = READ_BOTH_PINS;

START:
  R4 = READ_BOTH_PINS;
  *STORE = R4 | _getCycleCount8d8();
  STORE++;
  R3 = R4;
  //R4 = READ_BOTH_PINS;
  //if(R4!=R3)  goto START;
  if( R3 ) {
    for(int k=0;k<TOUT;k++) {
      R4   = READ_BOTH_PINS;
      if(R4!=R3)  goto START;
    }
  }
  //hal_enable_irq();
  received_NRZI_buffer_bytesCnt = STORE-received_NRZI_buffer;

#if 1
  //early activation for debugging
  if(received_NRZI_buffer_bytesCnt > 1) NOTIFY();
#endif
}



int sendRecieve(void)
{
  sendRecieveNParse();
  return parse_received_NRZI_buffer();
}



void SOF(void)
{
  if(1) {
    repack();
  }
  sendOnly();
}



void pu_Addr(uint8_t cmd,uint8_t addr,uint8_t eop)
{
  pu_MSB(T_START,8);
  pu_MSB(cmd,8);//setup
  pu_LSB(addr,7);
  pu_LSB(eop,4);
  pu_MSB(cal5(),5);
  repack();
}



void pu_ShortCmd(uint8_t cmd)
{
  pu_MSB(T_START,8);
  pu_MSB(cmd,8);//setup
  pu_MSB(0,16);
  repack();
}



void pu_Cmd(uint8_t cmd,uint8_t bmRequestType, uint8_t bmRequest,uint16_t wValue,uint16_t wIndex,uint16_t wLen)
{
  pu_MSB(T_START,8);
  pu_MSB(cmd,8);//setup
  pu_LSB(bmRequestType,8);
  pu_LSB(bmRequest,8);
  pu_LSB(wValue,16);
  pu_LSB(wIndex,16);
  pu_LSB(wLen,16);
  pu_MSB(cal16(),16);
  repack();
}



uint8_t ACK_BUFF[0x20];
int ACK_BUFF_CNT = 0;


void ACK(void)
{
  transmit_NRZI_buffer_cnt =0;
  if(ACK_BUFF_CNT==0) {
    pu_MSB(T_START,8);
    pu_MSB(T_ACK,8);// ack
    repack();
    memcpy(ACK_BUFF,transmit_NRZI_buffer,transmit_NRZI_buffer_cnt);
    ACK_BUFF_CNT = transmit_NRZI_buffer_cnt;
  } else {
    memcpy(transmit_NRZI_buffer,ACK_BUFF,ACK_BUFF_CNT);
    transmit_NRZI_buffer_cnt = ACK_BUFF_CNT;
  }
  sendOnly();
}


void timerCallBack(void)
{
  decoded_receive_buffer_clear();

  if(current->cb_Cmd==CB_CHECK) {
    SET_I(DP_PIN, DM_PIN);
    current->wires_last_state = READ_BOTH_PINS>>8;
    if(current->wires_last_state==M_ONE) {
      //NOTIFY(); //passes here
      // low speed
    } else if(current->wires_last_state==P_ONE) {
      //high speed
    } else if(current->wires_last_state==0x00) {
      // not connected
    } else if(current->wires_last_state== (M_ONE + P_ONE) ) {
      //????
    }
    current->bComplete = 1;
  } else if (current->cb_Cmd==CB_RESET) {
    SOF();
    sendRecieveNParse();
    SET_O(DP_PIN, DM_PIN);
    SE_0;
    current->cmdTimeOut  =   31;
    current->cb_Cmd         =   CB_WAIT0;
  } else if (current->cb_Cmd==CB_WAIT0) {
    if(current->cmdTimeOut>0) {
      current->cmdTimeOut--;
    } else {
      //sendRecieveNParse();
      current->bComplete     = 1;
    }
  } else if (current->cb_Cmd==CB_WAIT1) {
    SOF();
    if(current->cmdTimeOut>0) {
      current->cmdTimeOut--;
    } else {
      sendRecieveNParse();
      current->wires_last_state = READ_BOTH_PINS>>8;
      current->bComplete     = 1;
    }
  } else if (current->cb_Cmd==CB_POWER) {
    // for TEST
    #ifdef TEST
      SOF();
      sendRecieve();
      SOF();
      SOF();
    #else
      SET_O(DP_PIN, DM_PIN);
      SE_J; //this seems to prepare output pins value for the next use
      SET_I(DP_PIN, DM_PIN);
      current->cmdTimeOut  =    2;
      current->cb_Cmd  = CB_WAIT1;
    #endif
  } else if (current->cb_Cmd==CB_TICK) {
    SOF();
    current->bComplete     =         1;
  } else if(current->cb_Cmd==CB_3) {
    SOF();
    pu_Addr(current->rq.cmd,current->rq.addr,current->rq.eop);
    pu_Cmd(current->rq.dataCmd, current->rq.bmRequestType, current->rq.bmRequest,current->rq.wValue, current->rq.wIndex, current->rq.wLen);
    int res = sendRecieve();
    if(res==T_ACK) {
      current->cb_Cmd=CB_4;
      current->numb_reps_errors_allowed = 8;
      return ;
    } else {
      current->numb_reps_errors_allowed--;
      if(current->numb_reps_errors_allowed>0) {
        return ;
      } else {
        current->cb_Cmd=CB_TICK;
        current->bComplete = 1;
      }
    }
  } else if(current->cb_Cmd==CB_4) {
    SOF();
    pu_Addr(T_OUT,current->rq.addr,current->rq.eop);
    //reB();
    pu_MSB(T_START,8);
    pu_MSB(T_DATA1,8);//setup
    for(int k=0;k<current->transmitL1Bytes;k++) {
      pu_LSB(current->transmitL1[k],8);
    }
    pu_MSB(cal16(),16);
    repack();
    sendRecieveNParse();
    pu_Addr(T_IN,current->rq.addr,current->rq.eop);
    //setup
    sendRecieveNParse();
    if(received_NRZI_buffer_bytesCnt<SMALL_NO_DATA &&  received_NRZI_buffer_bytesCnt >SMALL_NO_DATA/4) {
      ACK();
    } else {
      current->numb_reps_errors_allowed--;
      if(current->numb_reps_errors_allowed>0) {
        return;
      } else {

      }
    }
    current->cb_Cmd=CB_TICK;
    current->bComplete = 1;
  } else if(current->cb_Cmd==CB_5) {
    SOF();
    pu_Addr(current->rq.cmd,current->rq.addr,current->rq.eop);
    pu_Cmd(current->rq.dataCmd, current->rq.bmRequestType, current->rq.bmRequest,current->rq.wValue, current->rq.wIndex, current->rq.wLen);
    sendRecieveNParse();

    int res = parse_received_NRZI_buffer();
    if(res==T_ACK) {
      current->cb_Cmd = CB_6;
      current->in_data_flip_flop = 1;
      current->numb_reps_errors_allowed = 4;
      current->counterAck ++;
      return ;
    } else {
      //SOF();
      current->counterNAck ++;
      current->numb_reps_errors_allowed--;
      if(current->numb_reps_errors_allowed>0) {
        // current->cb_Cmd = CB_TICK;
        current->acc_decoded_resp_counter = 0;
        return ;
      } else {
        current->cb_Cmd = CB_TICK;
        current->bComplete       =  1;
      }
    }
  } else if(current->cb_Cmd==CB_6) {
    SOF();
    pu_Addr(T_IN,current->rq.addr,current->rq.eop);
    //setup
    sendRecieveNParse();
    // if receive something ??
    if(current->asckedReceiveBytes==0 && current->acc_decoded_resp_counter==0 && received_NRZI_buffer_bytesCnt<SMALL_NO_DATA &&  received_NRZI_buffer_bytesCnt >SMALL_NO_DATA/4 ) {
      ACK();

      current->cb_Cmd = CB_TICK;
      current->bComplete       =  1;
      return ;
    }
    int res = parse_received_NRZI_buffer();
    if(res == T_NEED_ACK) {
      //SOF();
      if(decoded_receive_buffer_size()>2) {
        decoded_receive_buffer_get();
        uint8_t sval = decoded_receive_buffer_get();
        if((current->in_data_flip_flop&1)==1) {
          if(sval==T_DATA1) {

          } else {
            current->cb_Cmd = CB_7;
            return ;
          }
        } else {
          if(sval==T_DATA0) {

          } else {
            current->cb_Cmd = CB_7;
            return ;
          }
        }
        current->in_data_flip_flop++;
        int bytes =decoded_receive_buffer_size()-2;
        for(int kk=0;kk<bytes;kk++) {
          current->acc_decoded_resp[current->acc_decoded_resp_counter] = rev8(decoded_receive_buffer_get());
          current->acc_decoded_resp_counter++;
          current->asckedReceiveBytes--;
        }

        if(bytes<=0) {

          current->acc_decoded_resp_counter = 0;
          current->asckedReceiveBytes = 0;
          current->cb_Cmd = CB_TICK;
          current->bComplete = 1;
        } else {
          current->cb_Cmd = CB_7;
          return ;
        }
      } else {
        current->acc_decoded_resp_counter = 0;
        current->asckedReceiveBytes = 0;
        current->cb_Cmd = CB_TICK;
        current->bComplete = 1;
        return ;
      }
    } else {
      current->numb_reps_errors_allowed--;
      if(current->numb_reps_errors_allowed>0) {
        return ;
      } else {
        current->cb_Cmd = CB_TICK;
        current->bComplete = 1;
      }
    }
  } else if(current->cb_Cmd==CB_7) {
    SOF();
    pu_Addr(T_IN,current->rq.addr,current->rq.eop);
    //setup
    sendRecieveNParse();
    ACK();
    if(current->asckedReceiveBytes>0) {
      current->cb_Cmd = CB_6;
      return ;
    }
    current->cb_Cmd = CB_8;
  } else if(current->cb_Cmd==CB_8) {
    SOF();
    pu_Addr(T_OUT,current->rq.addr,current->rq.eop);
    pu_ShortCmd(T_DATA1);
    sendOnly();
    current->cb_Cmd = CB_TICK;
    current->bComplete = 1;
  } else if(current->cb_Cmd==CB_2Ack) {
    SOF();
    pu_Addr(T_IN,current->rq.addr,current->rq.eop);
    //setup
    sendRecieveNParse();
    if(received_NRZI_buffer_bytesCnt<SMALL_NO_DATA/2) {
      // no data , seems NAK or something like this
      current->cb_Cmd = CB_TICK;
      current->bComplete = 1;
      return ;
     }
    ACK();
    current->cb_Cmd = CB_TICK;
    current->bComplete = 1;
  } else if(current->cb_Cmd==CB_2) {
    SOF();
    pu_Addr(T_IN,current->rq.addr,current->rq.eop);
    //setup
    sendRecieveNParse();
    if(received_NRZI_buffer_bytesCnt<SMALL_NO_DATA/2) {
      // no data , seems NAK or something like this
      current->cb_Cmd = CB_TICK;
      current->bComplete = 1;
      return ;
    }
    int res = parse_received_NRZI_buffer();
    if(res==T_NEED_ACK) {
      if(decoded_receive_buffer_size()>2) {
        decoded_receive_buffer_get();
        decoded_receive_buffer_get();
        int bytes =decoded_receive_buffer_size()-2;
        for(int kk=0;kk<bytes;kk++) {
          current->acc_decoded_resp[current->acc_decoded_resp_counter] = rev8(decoded_receive_buffer_get());
          current->acc_decoded_resp_counter++;
          current->asckedReceiveBytes--;
        }
      }
      current->asckedReceiveBytes = 0;
      current->cb_Cmd=CB_2Ack;
      return ;
    } else {
      current->numb_reps_errors_allowed--;
      if(current->numb_reps_errors_allowed>0) {
        return ;
      } else {
        current->cb_Cmd = CB_TICK;
        current->bComplete = 1;
      }
    }
    current->cb_Cmd = CB_TICK;
    current->bComplete = 1;
    current->asckedReceiveBytes = 0;
  }
}



void Request( uint8_t cmd, uint8_t addr, uint8_t eop, uint8_t dataCmd,uint8_t bmRequestType, uint8_t bmRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLen, uint16_t waitForBytes)
{
  current->rq.cmd  = cmd;
  current->rq.addr = addr;
  current->rq.eop  = eop;
  current->rq.dataCmd = dataCmd;
  current->rq.bmRequestType = bmRequestType;
  current->rq.bmRequest = bmRequest;
  current->rq.wValue = wValue;
  current->rq.wIndex = wIndex;
  current->rq.wLen = wLen;

  current->numb_reps_errors_allowed = 4;
  current->asckedReceiveBytes = waitForBytes;
  current->acc_decoded_resp_counter    = 0;
  current->cb_Cmd = CB_5;
}



void RequestSend(uint8_t cmd,   uint8_t addr,uint8_t eop, uint8_t  dataCmd,uint8_t bmRequestType, uint8_t bmRequest,uint16_t wValue,uint16_t wIndex,uint16_t wLen,uint16_t transmitL1Bytes,uint8_t* data)
{
  current->rq.cmd  = cmd;
  current->rq.addr = addr;
  current->rq.eop  = eop;
  current->rq.dataCmd = dataCmd;
  current->rq.bmRequestType = bmRequestType;
  current->rq.bmRequest = bmRequest;
  current->rq.wValue = wValue;
  current->rq.wIndex = wIndex;
  current->rq.wLen = wLen;
  current->transmitL1Bytes = transmitL1Bytes;
  for(int k=0;k<current->transmitL1Bytes;k++) {
    current->transmitL1[k] = data[k];
  }
  current->numb_reps_errors_allowed = 4;
  current->acc_decoded_resp_counter    = 0;
  current->cb_Cmd = CB_3;
}


void RequestIn(uint8_t cmd,   uint8_t addr,uint8_t eop,uint16_t waitForBytes)
{
  current->rq.cmd  = cmd;
  current->rq.addr = addr;
  current->rq.eop  = eop;
  current->numb_reps_errors_allowed = 4;
  current->asckedReceiveBytes = waitForBytes;
  current->acc_decoded_resp_counter    = 0;
  current->cb_Cmd = CB_2;
}


void (*usbMess)(uint8_t src,uint8_t len,uint8_t *data) = NULL;

void set_usb_mess_cb( onusbmesscb_t onUSBMessCb )
{
  usbMess = onUSBMessCb;
}


void (*onDetectCB)(uint8_t usbNum, void *device) = NULL;

void set_ondetect_cb( ondetectcb_t cb )
{
  onDetectCB = cb;
}


void set_onled_blink_cb( onledblinkcb_t cb )
{
  onLedBlinkCB = cb;
}


void fsm_Mashine(void)
{
  if(!current->bComplete) return;
  current->bComplete = 0;

  if(current->fsm_state == 0) {
    current->epCount = 0;
    current->cb_Cmd     = CB_CHECK;
    current->fsm_state   = 1;
  }
  if(current->fsm_state == 1) {
    //NOTIFY(); //passes here
    if(current->wires_last_state==M_ONE) { // if(1)
        //NOTIFY(); //passes here

      current->cmdTimeOut = 100+current->selfNum*73;
      current->cb_Cmd      = CB_WAIT0;
      current->fsm_state   = 2;
    } else {
      current->fsm_state   = 0;
      current->cb_Cmd      = CB_CHECK;
    }
  } else if(current->fsm_state==2) {
    //NOTIFY(); //passes here
    current->cb_Cmd       = CB_RESET;
    current->fsm_state    = 3;
  } else if(current->fsm_state==3) {
    //NOTIFY(); //passes here
    current->cb_Cmd       = CB_POWER;
    #ifdef TEST
      current->fsm_state    =  3;
    #else
      current->fsm_state    =  4;
    #endif
  } else if(current->fsm_state==4) {
    Request(T_SETUP,ZERO_USB_ADDRESS,0b0000,T_DATA0,0x80,0x6,0x0100,0x0000,0x0012,0x0012);
    current->fsm_state    = 5;
  } else if(current->fsm_state==5) {
    //NOTIFY(); //passes here
    if(current->acc_decoded_resp_counter==0x12) {
      memcpy(&current->desc,current->acc_decoded_resp,0x12);
      current->ufPrintDesc |= 1;
    } else {
    //NOTIFY(); //passes here
      if(current->numb_reps_errors_allowed<=0) {
        current->fsm_state    =  0;
        return;
      }
    }

    Request(T_SETUP,ZERO_USB_ADDRESS,0b0000,T_DATA0,0x00,0x5,0x0000+ASSIGNED_USB_ADDRESS,0x0000,0x0000,0x0000);
    current->fsm_state    =  6;

  } else if(current->fsm_state==6) {
    current->cmdTimeOut = 5;
    current->cb_Cmd       = CB_WAIT1;
    current->fsm_state    = 7;
  } else if(current->fsm_state==7) {
    Request(T_SETUP,ASSIGNED_USB_ADDRESS,0b0000,T_DATA0,0x80,0x6,0x0200,0x0000,0x0009,0x0009);
    current->fsm_state    = 8;
  } else if(current->fsm_state==8) {
    if(current->acc_decoded_resp_counter==0x9) {
      memcpy(&current->cfg,current->acc_decoded_resp,0x9);
      current->ufPrintDesc |= 2;
      Request(T_SETUP,ASSIGNED_USB_ADDRESS,0b0000,T_DATA0,0x80,0x6,0x0200,0x0000,current->cfg.wLength,current->cfg.wLength);
      current->fsm_state    = 9;
    } else {
      current->fsm_state    = 0;
      return;
    }
  } else if(current->fsm_state==9) {
    if(current->acc_decoded_resp_counter==current->cfg.wLength) {
      current->ufPrintDesc |= 4;
      current->descrBufferLen = current->acc_decoded_resp_counter;
      memcpy(current->descrBuffer,current->acc_decoded_resp,current->descrBufferLen);
      current->fsm_state    = 97;
    } else {
      current->cmdTimeOut = 5;
      current->cb_Cmd       = CB_WAIT1;
      current->fsm_state    = 7;
    }
  } else if(current->fsm_state==97) {
    Request(T_SETUP,ASSIGNED_USB_ADDRESS,0b0000,T_DATA0,0x00,0x9,0x0001,0x0000,0x0000,0x0000);
     current->fsm_state    = 98;
  } else if(current->fsm_state==98) {
    // config interfaces??
    Request(T_SETUP,ASSIGNED_USB_ADDRESS,0b0000,T_DATA0,0x21,0xa,0x0000,0x0000,0x0000,0x0000);
    current->fsm_state    = 99;
  } else if(current->fsm_state==99) {
    if(current->flags_new!=current->flags) {
      current->flags = current->flags_new;
      RequestSend(T_SETUP,ASSIGNED_USB_ADDRESS,0b0000,T_DATA0,0x21,0x9,0x0200,0x0000,0x0001,0x0001,&current->flags);
    }
    current->fsm_state    = 100;
  } else if(current->fsm_state==100) {
     if( onLedBlinkCB ) onLedBlinkCB(0);
     RequestIn(T_IN,  ASSIGNED_USB_ADDRESS,1,8);
     current->fsm_state    = 101;
  } else if(current->fsm_state==101) {
    if(current->acc_decoded_resp_counter>=1) {
      usbMess(current->selfNum*4+0,current->acc_decoded_resp_counter,current->acc_decoded_resp);
      if( onLedBlinkCB ) onLedBlinkCB(1);
    }
    if(current->epCount>=2) {
      RequestIn(T_IN,  ASSIGNED_USB_ADDRESS,2,8);
      current->fsm_state    = 102;
    } else {
      current->cmdTimeOut = 3;
      current->cb_Cmd        = CB_WAIT1;
      current->fsm_state      = 104;
     }
  } else if(current->fsm_state==102) {
     if(current->acc_decoded_resp_counter>=1) {
       usbMess(current->selfNum*4+1,current->acc_decoded_resp_counter,current->acc_decoded_resp);
       if( onLedBlinkCB ) onLedBlinkCB(1);
     }
    current->cmdTimeOut = 2;
    current->cb_Cmd        = CB_WAIT1;
    current->fsm_state      = 104;
  } else if (current->fsm_state==104) {
    current->cmdTimeOut = 4;
    current->cb_Cmd        = CB_WAIT1;
  #ifdef  DEBUG_REPEAT
    static int rcnt =0;
    rcnt++;  //
    if(  (rcnt&0xff)==0 ||  (current->wires_last_state!=M_ONE))
  #else
    if(current->wires_last_state!=M_ONE)
  #endif
    {
      current->fsm_state      = 0;
      return ;
    }
    current->fsm_state      = 99;
  } else {
    current->cmdTimeOut = 2;
    current->cb_Cmd        = CB_WAIT1;
    current->fsm_state      = 0;
  }
}



void setPins(int DPPin,int DMPin)
{
  DP_PIN = DPPin;
  DM_PIN = DMPin;
  int diff = DPPin - DMPin;
  if(abs(diff)>7) {
    printf("PIN DIFFERENCE MUST BE LESS 8!\n");
    return;
  }
  int MIN_PIN = (DPPin<DMPin)?DPPin:DMPin;

  DM_PIN_M   = (1 << DMPin);
  DP_PIN_M    = (1 << DPPin);
  RD_MASK    = (1<<DPPin)|(1<<DMPin);

  RD_SHIFT = MIN_PIN;
  M_ONE = 1<<(DM_PIN-MIN_PIN);
  P_ONE = 1<<(DP_PIN-MIN_PIN);

}


sUsbContStruct  current_usb[NUM_USB];


int checkPins(int dp,int dm)
{
  int diff = abs(dp-dm);
  if(diff>7||diff==0) {
    return 0;
  }
  if( dp<8 || dp>31) return 0;
  if( dm<8 || dm>31) return 0;

  return 1;
}

/*
int64_t get_system_time_us(void)
{
  return 1000000ull*cpu_hal_get_cycle_count()/F_CPU;
  //return micros(); 
}
*/

float testDelay6(float freq_MHz)
{
  // 6 bits must take 4.0 uSec
  #define SEND_BITS 120
  #define REPS 40
  float res = 1;
  transmit_NRZI_buffer_cnt = 0;

  for(int k=0;k<SEND_BITS/2;k++) {
    transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_K;
    transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_J;
  }


  hal_disable_irq();
  uint64_t stimb = cpu_hal_get_cycle_count64();
  //uint32_t stimb = get_system_time_us();
  for(int k=0;k<REPS;k++) {
    sendOnly();
    transmit_NRZI_buffer_cnt = SEND_BITS;
  }
  uint64_t stim =  cpu_hal_get_cycle_count64()- stimb;
  //uint32_t stim =  get_system_time_us()- stimb;
  //freq_MHz = 1.0f;
  hal_enable_irq();
  
  res = stim*6.0/freq_MHz/(SEND_BITS*REPS);
  printf("%d bits in %f uSec %f MHz  6 ticks in %f uS\n",(SEND_BITS*REPS),(double)stim/(double)freq_MHz,(SEND_BITS*REPS)*(double)freq_MHz/(double)stim,(double)stim*(double)6.0/(double)freq_MHz/(double)(SEND_BITS*REPS));

  return res;
}

uint8_t arr[0x200];

#define F_USB_LOWSPEED 1500000
#define F_TIMING_BIT_ADDPRECISION 8
void gpio_test(void) //test with improved timing
{
    hal_gpio_set_direction(DP_PIN, 1);
    hal_gpio_set_direction(DM_PIN, 1);
    uint32_t xt1 = cpu_hal_get_cycle_count();
    uint8_t b = 0;
    uint32_t td = 0;
    for(int i = 0; i <= 6*2;)
    {
      int32_t t = cpu_hal_get_cycle_count() - xt1;
      if(t < td>>F_TIMING_BIT_ADDPRECISION) continue;
      hal_set_differential_gpio_value(DP_PIN, DM_PIN, b^=1);
      td += ((F_CPU/1000)*(1<<F_TIMING_BIT_ADDPRECISION))/(F_USB_LOWSPEED/1000);
      ++i;
    }
}

void initStates(int DP0,int DM0,int DP1,int DM1,int DP2,int DM2,int DP3,int DM3)
{
  decoded_receive_buffer_head = 0;
  decoded_receive_buffer_tail = 0;
  transmit_bits_buffer_store_cnt = 0;

  int calibrated = 0;

  for(int k=0;k<NUM_USB;k++) {
    current = &current_usb[k];
    if(k==0) {
      current->DP = DP0;
      current->DM = DM0;
    } else if(k==1) {
      current->DP = DP1;
      current->DM = DM1;
    } else if(k==2) {
      current->DP = DP2;
      current->DM = DM2;
    } else if(k==3) {
      current->DP = DP3;
      current->DM = DM3;
    }
    current->isValid = 0;
    if(checkPins(current->DP,current->DM)) {
      printf("USB#%d (pins %d %d) is OK!\n", k, (int)current->DP, (int)current->DM );
      current->selfNum = k;
      current->flags_new  = 0x0;
      current->flags 		= 0x0;
      current->in_data_flip_flop       = 0;
      current->bComplete  = 1;
      current->cmdTimeOut = 0;
      current->ufPrintDesc =0;
      current->cb_Cmd     = CB_CHECK;
      current->fsm_state  = 0;
      current->wires_last_state = 0;
      current->counterNAck = 0;
      current->counterAck = 0;
      current->epCount = 0;
      hal_gpio_pad_select_gpio(current->DP);
      hal_gpio_set_direction(current->DP, GPIO_MODE_OUTPUT);
      hal_gpio_set_level(current->DP, 0);
      hal_gpio_set_direction(current->DP, GPIO_MODE_INPUT);
      hal_gpio_pulldown_en(current->DP);

      hal_gpio_pad_select_gpio(current->DM);
      hal_gpio_set_direction(current->DM, GPIO_MODE_OUTPUT);
      hal_gpio_set_level(current->DM, 0);
      hal_gpio_set_direction(current->DM, GPIO_MODE_INPUT);
      hal_gpio_pulldown_en(current->DM);
      current->isValid = 1;

      // TEST
      setPins(current->DP,current->DM);
      //gpio_test();

      printf("READ_BOTH_PINS = %04x\n",READ_BOTH_PINS);
      SET_O(DP_PIN, DM_PIN);
      SE_0;
      SE_J;
      SE_0;
      SET_I(DP_PIN, DM_PIN);
      printf("READ_BOTH_PINS = %04x\n",READ_BOTH_PINS);
      hal_gpio_set_direction(current->DP, GPIO_MODE_OUTPUT);
      hal_gpio_set_direction(current->DM, GPIO_MODE_OUTPUT);
      printf("READ_BOTH_PINS = %04x\n",READ_BOTH_PINS);
      SET_I(DP_PIN, DM_PIN);
      printf("READ_BOTH_PINS = %04x\n",READ_BOTH_PINS);

      if(!calibrated) {
        //calibrate delay divide 2
        #define DELAY_CORR 2
        int freq_mhz = hal_get_cpu_mhz();
#ifdef ESP32        
        int  uTime = 250;
#else
#ifdef __IMXRT1062__
        int  uTime = freq_mhz;
#ifdef TIMING_PREC
        uTime *= TIMING_PREC;
#endif
#else
        int  uTime = freq_mhz*8;
#endif
#endif
        int  dTime = 0;
        printf("cpu freq = %d MHz\n", freq_mhz);
        TM_OUT = freq_mhz/2;
        // 8  - func divided clock to 8, 1.5 - MHz USB LS //NOTE: N=8 noy anymore constant, depends on clock frequency
        TIME_MULT = (int)(TIME_SCALE/(freq_mhz/(1<<TIME_FACTOR_BITS)/1.5)+0.5); //this fixes the timing bug!!
        printf("TIME_MULT = %d \n",TIME_MULT);

        int     TRANSMIT_TIME_DELAY_OPT = 0;
        TRANSMIT_TIME_DELAY = TRANSMIT_TIME_DELAY_OPT;
        printf("D=%4d ",TRANSMIT_TIME_DELAY);
        setDelay(TRANSMIT_TIME_DELAY);
        float  cS_opt = testDelay6(freq_mhz);
        #define OPT_TIME (4.00f)
        for(int p=0;p<12;p++) {
          TRANSMIT_TIME_DELAY = (uTime+dTime)/2;
          printf("D=%4d ",TRANSMIT_TIME_DELAY);
          setDelay(TRANSMIT_TIME_DELAY);
          float cS = testDelay6(freq_mhz);
          if(fabsf(OPT_TIME-cS)<fabsf(OPT_TIME-cS_opt)) {
            cS_opt = cS;
            TRANSMIT_TIME_DELAY_OPT = TRANSMIT_TIME_DELAY;
          }
          if(cS<OPT_TIME) {
            dTime = TRANSMIT_TIME_DELAY;
          } else {
            uTime = TRANSMIT_TIME_DELAY;
          }
        }
        TRANSMIT_TIME_DELAY = TRANSMIT_TIME_DELAY_OPT+DELAY_CORR;
        setDelay(TRANSMIT_TIME_DELAY);
        printf("TRANSMIT_TIME_DELAY = %d time = %f error = %f%% \n",TRANSMIT_TIME_DELAY,(double)cS_opt,(double)(cS_opt-OPT_TIME)/(double)OPT_TIME*(double)100);
      }
    } else {
      if( (int)current->DP == -1 && (int)current->DM == -1 ) {
#ifndef DEBUG_ALL
        printf("USB#%d is disabled by user configuration\n", k); //maybe called in interrupt handler for disabling the port
#endif
      } else {
        printf("USB#%d (pins %d %d) has errors and will be disabled !\n", k, (int)current->DP, (int)current->DM );
      }
    }
  }
}



void usbSetFlags(int _usb_num,uint8_t flags)
{
  if(_usb_num<NUM_USB&&_usb_num>=0) {
    current_usb[_usb_num].flags_new =  flags;
  }
}



uint8_t usbGetFlags(int _usb_num)
{
  if(_usb_num<NUM_USB&&_usb_num>=0) {
    return current_usb[_usb_num].flags;
  }
  return  0;
}



void usb_process(void)
{
#ifdef ESP32
  #if CONFIG_IDF_TARGET_ESP32C3 || defined ESP32C3
    cpu_ll_enable_cycle_count();
  #endif
#endif
  for(int k=0;k<NUM_USB;k++) {
    current = &current_usb[k];
    if(current->isValid) {
      setPins(current->DP,current->DM);
      timerCallBack();
      fsm_Mashine();
    }
  }
}



void printState(void)
{
  static int cntl = 0;
  cntl++;
  int ref = cntl%NUM_USB;
  sUsbContStruct * pcurrent = &current_usb[ref];
  if(!pcurrent->isValid) return ;
  if((cntl%800)<NUM_USB) {
    #ifdef DEBUG_ALL
      printf("USB%d: Ack = %d Nack = %d %02x pcurrent->cb_Cmd = %d  state = %d epCount = %d --",
        cntl%NUM_USB, pcurrent->counterAck,
        pcurrent->counterNAck,
        pcurrent->wires_last_state,
        pcurrent->cb_Cmd,
        pcurrent->fsm_state,
        pcurrent->epCount
      );
      for(int k=0;k<20;k++) {
        printf("%04x ", debug_buff[k]);
      }
      printf("\n");
    #endif
  }

  if(pcurrent->ufPrintDesc&1) {
    pcurrent->ufPrintDesc &= ~(uint32_t)1;
    if( onDetectCB ) {
      onDetectCB( ref, (void*)&pcurrent->desc );
    } else {
      printf("desc.bcdDevice       = %02x\n",pcurrent->desc.bcdDevice);
      printf("desc.iManufacturer   = %02x\n",pcurrent->desc.iManufacturer);
      printf("desc.iProduct        = %02x\n",pcurrent->desc.iProduct);
      printf("desc.iSerialNumber   = %02x\n",pcurrent->desc.iSerialNumber);
      printf("desc.bNumConfigurations = %02x\n",pcurrent->desc.bNumConfigurations);
    }

  }

  if(pcurrent->ufPrintDesc&2) {
    pcurrent->ufPrintDesc &= ~(uint32_t)2;
  }

  if(pcurrent->ufPrintDesc&4) {
    pcurrent->ufPrintDesc &= ~(uint32_t)4;
    HIDDescriptor hid[4];
    /*sCfgDesc lcfg;
    sIntfDesc sIntf;
    sEPDesc epd;
    int cfgCount   = 0;
    int sIntfCount   = 0;*/
    int hidCount   = 0;
    int pos = 0;
    #define STDCLASS        0x00
    #define HIDCLASS        0x03
    #define HUBCLASS     0x09      // bDeviceClass, bInterfaceClass
    #ifdef DEBUG_ALL
      printf("clear epCount %d self = %d\n",pcurrent->epCount,pcurrent->selfNum);
    #endif
    pcurrent->epCount     = 0;
    while(pos<pcurrent->descrBufferLen-2) {
      uint8_t len  =  pcurrent->descrBuffer[pos];
      uint8_t type =  pcurrent->descrBuffer[pos+1];
      if(len==0) {
        pos = pcurrent->descrBufferLen;
      }
      if(pos+len<=pcurrent->descrBufferLen) {
        //printf("\n");
        if(type == 0x2) {
          sCfgDesc cfg;
          memcpy(&cfg,&pcurrent->descrBuffer[pos],len);
          #ifdef DEBUG_ALL
            printf("cfg.wLength         = %02x\n",cfg.wLength);
            printf("cfg.bNumIntf        = %02x\n",cfg.bNumIntf);
            printf("cfg.bCV             = %02x\n",cfg.bCV);
            printf("cfg.bMaxPower       = %d\n",cfg.bMaxPower);
          #endif
        } else if (type == 0x4) {
          sIntfDesc sIntf;
          memcpy(&sIntf,&pcurrent->descrBuffer[pos],len);
        } else if (type == 0x21) {
          hidCount++;
          int i = hidCount-1;
          memcpy(&hid[i],&pcurrent->descrBuffer[pos],len);
        } else if (type == 0x5) {
          //pcurrent->epCount++;
          sEPDesc epd;
          memcpy(&epd,&pcurrent->descrBuffer[pos],len);
          #ifdef DEBUG_ALL
            printf("pcurrent->epCount = %d\n",pcurrent->epCount);
            printf("epd.bLength       = %02x\n",epd.bLength);
            printf("epd.bType         = %02x\n",epd.bType);
            printf("epd.bEPAdd        = %02x\n",epd.bEPAdd);
            printf("epd.bAttr         = %02x\n",epd.bAttr);
            printf("epd.wPayLoad      = %02x\n",epd.wPayLoad);
            printf("epd.bInterval     = %02x\n",epd.bInterval);
          #endif
        }
      }
      pos+=len;
    }
  }
}


#pragma GCC diagnostic pop


