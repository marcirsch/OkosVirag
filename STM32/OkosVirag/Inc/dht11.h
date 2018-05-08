
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DHT11_H
#define __DHT11_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "utils.h"
#include "stm32l0xx_hal.h"
#include "main.h"

/* Exported constants --------------------------------------------------------*/
#define MAX_TICS 10000
#define DHT11_OK 0
#define DHT11_NO_CONN 1
#define DHT11_CS_ERROR 2

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
uint8_t read_DHT11(uint8_t *buf);
uint16_t read_cycle(uint16_t cur_tics, uint8_t neg_tic);

#ifdef __cplusplus
}
#endif

#endif /* __DHT11_H */
