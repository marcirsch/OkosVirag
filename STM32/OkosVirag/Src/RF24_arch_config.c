#include "RF24_arch_config.h"

#include "stm32l0xx_hal.h"

uint8_t SPI_Tansfer(uint8_t data) {
	uint8_t out = data;
	uint8_t in;
//	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
//	HAL_SPI_Receive(&hspi1, &tmp, 1, 1000);
	HAL_SPI_TransmitReceive(&hspi1, &out, &in, 1, 1000);
	return in;
}

void digitalWrite(uint16_t pin, bool mode) {
	GPIO_PinState ps = GPIO_PIN_RESET;
	if (mode) {
		ps = GPIO_PIN_SET;
	}

	if (pin == NRF24_CE_Pin) {
		HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, ps);
	} else if (pin == NRF24_CS_Pin) {
		HAL_GPIO_WritePin(NRF24_CS_GPIO_Port, NRF24_CS_Pin, ps);
	} else {
//		assert_failed((uint8_t *) __FILE__, __LINE__);
	}
}

void printf_re(const char* format, ...) {
	return;
}
