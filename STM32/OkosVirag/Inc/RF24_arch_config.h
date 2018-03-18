#ifndef __ARCH_CONFIG_H__
#define __ARCH_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <stm32l0xx_hal.h>

#include "spi.h"

extern SPI_HandleTypeDef hspi1;
// Additional fixes for LittleWire

//  extern LittleWireSPI _SPI;

// GCC a Arduino Missing
#define _BV(x) (1<<(x))
#define pgm_read_word(p) (*(p))
#define pgm_read_byte(p) (*(p))

//typedef uint16_t prog_uint16_t;
#define PSTR(x) (x)
#define printf_P printf_re
//  #define strlen_P strlen
#define PROGMEM
#define PRIPSTR "%s"

#define LOW false
#define HIGH true
#define delayMicroseconds(x) HAL_Delay(x)
#define delay(x) HAL_Delay(1000UL*x)

#define millis HAL_GetTick

uint8_t SPI_Tansfer(uint8_t data);

void digitalWrite(uint16_t pin, bool mode);

void printf_re(const char* format, ...);

//  #ifdef SERIAL_DEBUG
//	#define IF_SERIAL_DEBUG(x) ({x;})
//  #else
#define IF_SERIAL_DEBUG(x)
//	#if defined(RF24_TINY)
//	  #define printf_P(...)
//    #endif
//  #endif

#ifdef __cplusplus
}
#endif

#endif
