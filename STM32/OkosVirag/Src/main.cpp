/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
extern "C" {
#include "main.h"
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"
#include "rcc.h"
#include "stm32l0xx_ll_utils.h"

#include "Sleep.h"
}
#include "RF24.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
#define MSG_TYPE_TEMP_HUM 1
#define MSG_TYPE_DOWNLINK 0
uint32_t myAddr;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t addresses[][6] = { "1Node", "2Node" };

// maximum payload is 32 bytes
// myaddr is 4, type is 1
// msg has 27 left
#pragma pack(1)
typedef struct {
	uint32_t myaddr;
	uint8_t msgType;
	uint32_t msg[27];
}__attribute__ ((packed)) msg_header_t;
#pragma pack()

#pragma pack(1)
typedef struct {
	int8_t temperature;
	int8_t humidity;
}__attribute__ ((packed)) sensor_msg_t;
#pragma pack()

#pragma pack(1)
typedef struct {
	int8_t water_threshold;
	int8_t sleep_seconds;
} __attribute__ ((packed)) downlink_msg_t;
#pragma pack()


#pragma pack(1)
typedef struct {
	int8_t water_threshold;
	int8_t sleep_seconds;
} __attribute__ ((packed)) configuration_t;
#pragma pack()


downlink_msg_t downlinkMsg;
sensor_msg_t sensorMsg;
msg_header_t headerMsg;

configuration_t config = {10,10};

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_IWDG_Init();
	MX_RTC_Init();
//  HAL_RTC_MspInit(&hrtc);
	MX_ADC_Init();
	MX_SPI1_Init();

	/* USER CODE BEGIN 2 */
	myAddr = LL_GetUID_Word0(); //use word0 of unique ID as addr


	RF24 radio(NRF24_CE_Pin, NRF24_CS_Pin);

	radio.begin();
	radio.setPALevel(RF24_PA_HIGH);

	uint8_t addresses[][6] = { "1Node", "2Node" };
	radio.openWritingPipe(addresses[0]);
	radio.openReadingPipe(1, addresses[1]);
	radio.setPayloadSize(32);

	radio.startListening();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		//Do measurements here, update sensor data
		headerMsg.myaddr = myAddr;
		headerMsg.msgType = MSG_TYPE_TEMP_HUM;
		sensorMsg.temperature = 30;
		sensorMsg.humidity = 70;
		memcpy(headerMsg.msg, &sensorMsg, sizeof(sensor_msg_t));

		//send on radio
		radio.stopListening();          // First, stop listening so we can talk.

		if (!radio.write(&headerMsg, sizeof(headerMsg))) {
			//handle send fail
		}

		//receive something from the cloud in 100ms window
		radio.startListening();

		// Now, continue listening
		uint32_t timeout = HAL_GetTick() + 100;
		// if there is data ready

		while (HAL_GetTick() < timeout) {
			if (radio.available()) {
				// Fetch the payload, and see if this was the last one.
				while (radio.available()) {
					radio.read(&headerMsg, sizeof(msg_header_t));
				}

				if (headerMsg.myaddr
						== myAddr&& headerMsg.msgType == MSG_TYPE_DOWNLINK) {
					//msg for me in downlink
					memcpy(&downlinkMsg, headerMsg.msg, sizeof(downlink_msg_t));
					config.sleep_seconds = downlinkMsg.sleep_seconds;
					config.water_threshold = downlinkMsg.water_threshold;

					//handle downlink

				}

			}
		}

		// Try again 1s later

		/* USER CODE BEGIN 3 */
		HAL_IWDG_Refresh(&hiwdg); //reload watchdog

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		//sleep
		Sleep(config.sleep_seconds);

	}
	/* USER CODE END 3 */

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
