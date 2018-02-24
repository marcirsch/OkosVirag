#include "Sleep.h"

#include "adc.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"

#include "stm32l0xx_hal.h"

void Sleep(int sleepSec) {
	if (sleepSec < 0) {
		return;
	}
	/* Enable Ultra low power mode */
	HAL_PWREx_EnableUltraLowPower();

	/* Enable the fast wake up from Ultra low power mode */
	HAL_PWREx_EnableFastWakeUp();

	/* Select HSI as system clock source after Wake Up from Stop mode */
	__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);

	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

	//Set RTC wakeup timer to sleepSeconds
	if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, sleepSec,
	RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK) {
		Error_Handler();
	}

	//TODO: Wakeup timer works but not going to sleep
	//TODO: enter sleep mode
	//TODO: Reinit after wakeup
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnterSTANDBYMode();
//	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

//	/* Configure the system clock */
	SystemClock_Config();
//
//	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_IWDG_Init();
	MX_RTC_Init();
	MX_ADC_Init();
	MX_SPI1_Init();

}
