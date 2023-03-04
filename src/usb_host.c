#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>

#include "sdkconfig.h"
#include "soc/soc.h"
#include "soc/rtc.h"
#include "math.h"
#include "esp_heap_caps.h"

#ifdef PLATFORMIO
  #include "esp32-hal.h"
#else
  #include "hal/cpu_hal.h"
  #include "hal/gpio_hal.h"
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
   Changes by tobozo (aug 2022):
     - Quick & dirty cpuDelay() to enable ESP32-S2 support when CONFIG_ESP_SYSTEM_MEMPROT_FEATURE is enabled (default on Arduino IDE)
     - Some reformatting and refactoring

\*/


#include "usb_host.h"

// Arduino IDE complains about volatile at init, but we don't care
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
#pragma GCC diagnostic ignored "-Wunused-variable"


#define T_START      0b00000001
#define T_ACK        0b01001011
#define T_NACK       0b01011010
#define T_SOF        0b10100101
#define T_SETUP      0b10110100
#define T_DATA0      0b11000011
#define T_DATA1      0b11010010
#define T_DATA2      0b11100001
#define T_OUT        0b10000111
#define T_IN         0b10010110

#define T_ERR        0b00111100
#define T_PRE        0b00111100
#define T_NYET       0b01101001
#define T_STALL      0b01111000

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

uint32_t _getCycleCount32()
{
  uint32_t ccount = cpu_hal_get_cycle_count();
  return  ccount;
}
uint8_t _getCycleCount8d8(void)
{
  uint32_t ccount = cpu_hal_get_cycle_count();
  return ccount>>3;
}


#define SE_J  { *snd[1][0] = (1 << DM_PIN);*snd[1][1] = (1 << DP_PIN); }
#define SE_0  { *snd[2][0] = (1 << DM_PIN);*snd[2][1] = (1 << DP_PIN); }

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
  #define SET_I { PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[DP_PIN]); PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[DM_PIN]); GPIO.enable_w1tc = (1 << DP_PIN) | (1 << DM_PIN);  }
  #define SET_O { GPIO.enable_w1ts = (1 << DP_PIN) | (1 << DM_PIN);  PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[DP_PIN]); PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[DM_PIN]);  }
  #define READ_BOTH_PINS (((GPIO.in&RD_MASK)<<8)>>RD_SHIFT)
  uint32_t * snd[4][2]  =
  {
    {&GPIO.out_w1tc,&GPIO.out_w1ts},
    {&GPIO.out_w1ts,&GPIO.out_w1tc},
    {&GPIO.out_w1tc,&GPIO.out_w1tc},
    {&GPIO.out_w1tc,&GPIO.out_w1tc}
  } ;
#else // ESP32C3
  #define SET_I { PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[DP_PIN]); PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[DM_PIN]);  gpio_ll_output_disable(&GPIO,DM_PIN); gpio_ll_output_disable(&GPIO,DP_PIN);}
  #define SET_O { GPIO.enable_w1ts.val = (1 << DP_PIN) | (1 << DM_PIN);  PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[DP_PIN]); PIN_INPUT_DISABLE(GPIO_PIN_MUX_REG[DM_PIN]);  }
  #define READ_BOTH_PINS (((GPIO.in.val&RD_MASK)<<8)>>RD_SHIFT)
  uint32_t * snd[4][2]  =
  {
    {&GPIO.out_w1tc.val,&GPIO.out_w1ts.val},
    {&GPIO.out_w1ts.val,&GPIO.out_w1tc.val},
    {&GPIO.out_w1tc.val,&GPIO.out_w1tc.val},
    {&GPIO.out_w1tc.val,&GPIO.out_w1tc.val}
  } ;
#endif

//must be setup each time with setPins
uint32_t DP_PIN;
uint32_t DM_PIN;

uint32_t DM_PIN_M;
uint32_t DP_PIN_M;
uint16_t M_ONE;
uint16_t P_ONE;
uint32_t RD_MASK;
uint32_t RD_SHIFT;
//end must be setup each time with setPins

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


void (*cpuDelay)() = NULL;


#if defined CONFIG_ESP_SYSTEM_MEMPROT_FEATURE || defined FORCE_TEMPLATED_NOPS
  // memory protection enabled, MALLOC_CAP_EXEC can't be used
  // use c++ templated nop() instead (see nops.hpp)
  typedef void (*CpuDelay_t)();
  extern CpuDelay_t nops[256];
  void setCPUDelay(uint8_t ticks)
  {
    cpuDelay = nops[ticks];
  }

#else // memprot disabled, make use of MALLOC_CAP_EXEC

  #if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
    #define MAX_DELAY_CODE_SIZE 0x280
    #define SDELAYMALLOC(X) malloc(X)
    #define SDELAYREALLOC(X,Y,Z) heap_caps_realloc(X,Y,Z)
    #define SDELAYRMASK MALLOC_CAP_EXEC
  #elif CONFIG_IDF_TARGET_ESP32C3
    #define MAX_DELAY_CODE_SIZE 0x210
    #define SDELAYMALLOC(X) malloc(X)
    #define SDELAYREALLOC(X,Y,Z) heap_caps_realloc(X,Y,Z)
    #define SDELAYRMASK MALLOC_CAP_32BIT|MALLOC_CAP_EXEC
  #else
    #error "Unsupported target"
  #endif

  void makeOpcodes( uint8_t* ptr, uint8_t ticks )
  {
    #if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
      printf("Making xtensa Opcodes for ESP32/ESP32S2\n");
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
    #elif CONFIG_IDF_TARGET_ESP32C3
      printf("Making RISCV Opcodes for ESP32C3\n");
      //put head of delay procedure
      for(int k=0;k<ticks;k++) {
        //put NOPs
        *ptr++ = 0x1;
        *ptr++ = 0x0;
      }
      //put tail of delay procedure
      *ptr++ = 0x82;
      *ptr++ = 0x80;
    #else
     #error "Unsupported target"
    #endif
  }

  void setCPUDelay(uint8_t ticks)
  {
    uint8_t* pntS;

    printf("Setting delay to %d ticks\n", ticks);

    size_t free_iram = heap_caps_get_free_size(MALLOC_CAP_EXEC);
    printf("Free iram before alloc: %d\n", free_iram );

    // it can't execute but can read & write
    if(!cpuDelay) {
      printf("Malloc MAX_DELAY_CODE_SIZE=%d\n", MAX_DELAY_CODE_SIZE );
      pntS = SDELAYMALLOC( MAX_DELAY_CODE_SIZE );
    } else {
      printf("Realloc MAX_DELAY_CODE_SIZE=%d\n", MAX_DELAY_CODE_SIZE );
      pntS = SDELAYREALLOC( cpuDelay, MAX_DELAY_CODE_SIZE, MALLOC_CAP_8BIT );
    }

    if(!pntS) {
      printf("OOM!\nHalting...\n");
      while(1) { vTaskDelay(1); }
    }

    uint8_t* pnt = (uint8_t*)pntS;
    makeOpcodes( pnt, ticks );
    printf("moving opecodes to executable memory segment\n");
    // it can't  write  but can read & execute

    free_iram = heap_caps_get_free_size(MALLOC_CAP_EXEC);
    printf("Free iram after alloc and before realloc: %d\n", free_iram );

    cpuDelay = SDELAYREALLOC( pntS, MAX_DELAY_CODE_SIZE, SDELAYRMASK );

    free_iram = heap_caps_get_free_size(MALLOC_CAP_EXEC);
    printf("Free iram after realloc to cap_exec: %d\n", free_iram );

    if(!cpuDelay) {
      printf("idf.py menuconfig\n Component config-> ESP System Setting -> Memory protectiom-> Disable.\n memory prot must be disabled!!!\n cpuDelay = %p,\nHalting...\n",cpuDelay);
      while(1) { vTaskDelay(1); }
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

void parseImmed(sUsbContStruct * pcurrent)
{
  static sCfgDesc      cfg;
  static sIntfDesc     sIntf;
  static HIDDescriptor hid[4];
  static sEPDesc       epd;
  static int           cfgCount   = 0;
  static int           sIntfCount   = 0;
  static int           hidCount   = 0;
  int                  pos = 0;

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



void restart()
{
  transmit_NRZI_buffer_cnt = 0;
}



void decoded_receive_buffer_clear()
{
  decoded_receive_buffer_tail = decoded_receive_buffer_head;
}



void decoded_receive_buffer_put(uint8_t val)
{
  decoded_receive_buffer[decoded_receive_buffer_head] = val;
  decoded_receive_buffer_head++;
}



uint8_t decoded_receive_buffer_get()
{
  return decoded_receive_buffer[decoded_receive_buffer_tail++];
}



uint8_t decoded_receive_buffer_size()
{
  return (uint8_t )(decoded_receive_buffer_head-decoded_receive_buffer_tail);
}



uint8_t cal5()
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



uint32_t cal16()
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



void repack()
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



int parse_received_NRZI_buffer()
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
void sendOnly()
{
  uint8_t k;
  SET_O;
  #ifdef WR_SIMULTA
    uint32_t out_base = GPIO.out;
    sndA[0] = (out_base | DP_PIN_M) &~DM_PIN_M;
    sndA[1] = (out_base | DM_PIN_M) &~DP_PIN_M;
    sndA[2] = (out_base )&~(DP_PIN_M | DM_PIN_M);
    sndA[3] =  out_base | (DM_PIN_M | DP_PIN_M);
  #endif
  for(k=0;k<transmit_NRZI_buffer_cnt;k++) {
    //usb_transmit_delay(10);
    cpuDelay();
    #ifdef WR_SIMULTA
      GPIO.out = sndA[transmit_NRZI_buffer[k]];
    #else
      *snd[transmit_NRZI_buffer[k]][0] = DM_PIN_M;
      *snd[transmit_NRZI_buffer[k]][1] = DP_PIN_M;
    #endif
  }
  restart();
  SET_I;
}


void sendRecieveNParse()
{
  register uint32_t _R3;
  register uint16_t *STORE = received_NRZI_buffer;
  //__disable_irq();
  sendOnly();
  register uint32_t _R4;// = READ_BOTH_PINS;

START:
  _R4 = READ_BOTH_PINS;
  *STORE = _R4 | _getCycleCount8d8();
  STORE++;
  _R3 = _R4;
  //_R4 = READ_BOTH_PINS;
  //if(_R4!=_R3)  goto START;
  if( _R3 ) {
    for(int k=0;k<TOUT;k++) {
      _R4   = READ_BOTH_PINS;
      if(_R4!=_R3)  goto START;
    }
  }
  //__enable_irq();
  received_NRZI_buffer_bytesCnt = STORE-received_NRZI_buffer;
}



int sendRecieve()
{
  sendRecieveNParse();
  return parse_received_NRZI_buffer();
}



void SOF()
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


void ACK()
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


void timerCallBack()
{
  decoded_receive_buffer_clear();

  if(current->cb_Cmd==CB_CHECK) {
    SET_I;
    current->wires_last_state = READ_BOTH_PINS>>8;
    if(current->wires_last_state==M_ONE) {
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
    SET_O;
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
      SET_O;
      SE_J;
      SET_I;
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


void (*onLedBlinkCB)(int on_off) = NULL;

void set_onled_blink_cb( onledblinkcb_t cb )
{
  onLedBlinkCB = cb;
}


void fsm_Mashine()
{
  if(!current->bComplete) return;
  current->bComplete = 0;

  if(current->fsm_state == 0) {
    current->epCount = 0;
    current->cb_Cmd     = CB_CHECK;
    current->fsm_state   = 1;
  }
  if(current->fsm_state == 1) {
    if(current->wires_last_state==M_ONE) { // if(1)
      current->cmdTimeOut = 100+current->selfNum*73;
      current->cb_Cmd      = CB_WAIT0;
      current->fsm_state   = 2;
    } else {
      current->fsm_state   = 0;
      current->cb_Cmd      = CB_CHECK;
    }
  } else if(current->fsm_state==2) {
    current->cb_Cmd       = CB_RESET;
    current->fsm_state    = 3;
  } else if(current->fsm_state==3) {
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
    if(current->acc_decoded_resp_counter==0x12) {
      memcpy(&current->desc,current->acc_decoded_resp,0x12);
      current->ufPrintDesc |= 1;
    } else {
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
    printf("Diff not in [1-7] range: %d\n", diff );
    return 0;
  }

  if( dp<8 || dp>31) {
    printf("Dp not in [8-31] range: [%d]\n", dp );
    return 0;
  }
  if( dm<8 || dm>31) {
    printf("Dm not in [8-31] range: [%d]\n", dp );
    return 0;
  }

  return 1;
}


int64_t get_system_time_us()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec * 1000000LL + (tv.tv_usec ));
}


float testDelay6(float freq_MHz)
{
  // 6 bits must take 4.0 uSec
  #define SEND_BITS 120
  #define REPS      40
  float res = 1;
  transmit_NRZI_buffer_cnt = 0;

  for(int k=0;k<SEND_BITS/2;k++) {
    transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_K;
    transmit_NRZI_buffer[transmit_NRZI_buffer_cnt++] = USB_LS_J;
  }

  int64_t stimb = get_system_time_us();
  for(int k=0;k<REPS;k++) {
    sendOnly();
    transmit_NRZI_buffer_cnt = SEND_BITS;
  }

  uint32_t stim =  get_system_time_us()- stimb;
  freq_MHz = 1.0f;
  res = stim*6.0/freq_MHz/(SEND_BITS*REPS);
  printf("%d bits in %f uSec %f MHz  6 ticks in %f uS\n",(SEND_BITS*REPS),stim/(float)freq_MHz,(SEND_BITS*REPS)*freq_MHz/stim,stim*6.0/freq_MHz/(SEND_BITS*REPS));

  return res;
}

uint8_t arr[0x200];



void setupGPIO( int pin )
{
  gpio_pad_select_gpio( pin );
  gpio_set_direction( pin , GPIO_MODE_OUTPUT);
  gpio_set_level( pin , 0);
  gpio_set_direction( pin , GPIO_MODE_INPUT);
  gpio_pulldown_en( pin );
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

      printf("USB#%d (pins %d %d) is OK!\n", k, current->DP, current->DM );

      current->selfNum           = k;
      current->flags_new         = 0x0;
      current->flags 		         = 0x0;
      current->in_data_flip_flop = 0;
      current->bComplete         = 1;
      current->cmdTimeOut        = 0;
      current->ufPrintDesc       = 0;
      current->cb_Cmd            = CB_CHECK;
      current->fsm_state         = 0;
      current->wires_last_state  = 0;
      current->counterNAck       = 0;
      current->counterAck        = 0;
      current->epCount           = 0;

      setupGPIO( current->DP );
      setupGPIO( current->DM );

      current->isValid = 1;

      // TEST
      setPins(current->DP,current->DM);
      printf("READ_BOTH_PINS = %04x\n",READ_BOTH_PINS);
      SET_O;
      SE_0;
      SE_J;
      SE_0;
      SET_I;
      printf("READ_BOTH_PINS = %04x\n",READ_BOTH_PINS);
      gpio_set_direction(current->DP, GPIO_MODE_OUTPUT);
      gpio_set_direction(current->DM, GPIO_MODE_OUTPUT);
      printf("READ_BOTH_PINS = %04x\n",READ_BOTH_PINS);
      SET_I;
      printf("READ_BOTH_PINS = %04x\n",READ_BOTH_PINS);

      if(!calibrated) {
        //calibrate delay divide 2
        #define DELAY_CORR 2
        int  uTime = 254;
        int  dTime = 0;

        rtc_cpu_freq_config_t  out_config;

        rtc_clk_cpu_freq_get_config(&out_config);
        printf("cpu freq = %d MHz\n",out_config.freq_mhz);

        TM_OUT = out_config.freq_mhz/2;

        // 8  - func divided clock to 8, 1.5 - MHz USB LS
        TIME_MULT = (int)( TIME_SCALE / ( out_config.freq_mhz/8/1.5 ) + 0.5 );
        printf("TIME_MULT = %d \n",TIME_MULT);

        int TRANSMIT_TIME_DELAY_OPT = 0;
        TRANSMIT_TIME_DELAY = TRANSMIT_TIME_DELAY_OPT;
        printf( "D=%4d ", TRANSMIT_TIME_DELAY );
        setCPUDelay( TRANSMIT_TIME_DELAY );
        float  cS_opt = testDelay6( out_config.freq_mhz );

        #define OPT_TIME (4.00f)
        for( int p=0;p<9;p++ ) {
          TRANSMIT_TIME_DELAY = (uTime+dTime)/2;
          printf( "D=%4d ", TRANSMIT_TIME_DELAY );
          setCPUDelay( TRANSMIT_TIME_DELAY );
          float cS = testDelay6( out_config.freq_mhz );
          if( fabsf( OPT_TIME-cS )< fabsf( OPT_TIME-cS_opt ) ) {
            cS_opt = cS;
            TRANSMIT_TIME_DELAY_OPT = TRANSMIT_TIME_DELAY;
          }
          if( cS<OPT_TIME ) {
            dTime = TRANSMIT_TIME_DELAY;
          } else {
            uTime = TRANSMIT_TIME_DELAY;
          }
        }
        TRANSMIT_TIME_DELAY = TRANSMIT_TIME_DELAY_OPT+DELAY_CORR;
        setCPUDelay( TRANSMIT_TIME_DELAY );
        printf( "TRANSMIT_TIME_DELAY = %d time = %f error = %f%% \n", TRANSMIT_TIME_DELAY, cS_opt, (cS_opt-OPT_TIME)/OPT_TIME*100 );
      }
    } else {
      if( current->DP == -1 && current->DM == -1 ) {
        printf( "USB#%d is disabled by user configuration\n", k );
      } else {
        printf( "USB#%d (pins %d %d) has errors and will be disabled !\n", k, current->DP, current->DM );
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



void usb_process()
{
  #if CONFIG_IDF_TARGET_ESP32C3 || defined ESP32C3
    cpu_ll_enable_cycle_count();
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



void printState()
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
    sCfgDesc lcfg;
    sIntfDesc sIntf;
    HIDDescriptor hid[4];
    sEPDesc epd;
    int cfgCount   = 0;
    int sIntfCount   = 0;
    int hidCount   = 0;
    int pos = 0;
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

