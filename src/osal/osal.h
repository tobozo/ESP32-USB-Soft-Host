#ifndef __OSAL_H__
#define __OSAL_H__

//this somewhat mimics TinyUSB's osal.h
#include <string.h>
#include <stdint.h>
#include "irq.h"
static inline int tu_min16(uint16_t a, uint16_t b) { return a<b ? a:b;} //needed for FIFOs
static inline void tu_unaligned_write32(void *ptr, uint32_t value) { *(uint32_t*)ptr = value; }
static inline uint32_t tu_unaligned_read32(const void * ptr) { return *(uint32_t*)ptr; }

#define FAST_DATA __attribute__ ((section (".fast_data")))  

#endif //__OSAL_H__
