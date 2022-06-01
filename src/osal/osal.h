#ifndef __OSAL_H__
#define __OSAL_H__

//this somewhat mimics TinyUSB's osal.h
#include <string.h>
#include <stdint.h>
#include "irq.h"
static inline int tu_min16(uint16_t a, uint16_t b) { return a<b ? a:b;} //needed for FIFOs

#endif //__OSAL_H__
