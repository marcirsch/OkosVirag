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

#include "math.h"
#include "dht11.h"

#include "Sleep.h"
}
#include "RF24.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
#define MSG_TYPE_TEMP_HUM 1
#define MSG_TYPE_DOWNLINK 0
uint32_t myAddr;

// Define sensor constants
#define B 3950		// K
#define RT0 10000	// Ohm
#define R 10000		// Ohm
#define VCC 3		// V
//#define T0 25.0		// °C

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
	int16_t temperature;
	int8_t humidity;
	int16_t light;
}__attribute__ ((packed)) sensor_msg_t;
#pragma pack()

#pragma pack(1)
typedef struct {
	int16_t water_threshold;
	int16_t sleep_seconds;
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

volatile float VRL = 0;
volatile float VRTh = 0;
//volatile float VHum = 0;

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
	HAL_ADC_Start(&hadc);

	// For value from ADC channels
	float VRTemp, VRLight, RTemp, RLight, ln, T0, TX, Light;
	// For humidity values
	uint8_t buf[5], res;

	// Data for testing communication
	uint8_t cntC = 0;
	float humArray[] = {52.0, 54.0, 51.0, 55.0, 60.0, 61.0, 58.0, 55.0, 56.0, 53.0};
	float lightArray[] = {8.2, 8.8, 8.4, 8.5, 9.3, 12.4, 10.1, 9.8, 9.6, 9.3};
	float tempArray[] = {24.5, 24.3, 24.3, 25.5, 26.3, 27.5, 28.3, 25.2, 24.3, 24.1};


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

		// TODO kiolvasott szenzoradatok
		// Reading data from sensors
	   if(HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
	    	VRL = HAL_ADC_GetValue(&hadc);
	   }

	   if(HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
	    	VRTh = HAL_ADC_GetValue(&hadc);
	   }


	   /* if(HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
	    	VHum = HAL_ADC_GetValue(&hadc);
	   }*/

	   VRL = (VCC / 4095.0) * VRL;		// Conversion to voltage
	   VRTh = (VCC / 4095.0) * VRTh;	// Conversion to voltage
	   //VHum = (VCC / 4096.0) * VHum;	// Conversion to voltage

	   // Calculate temperature
	   VRTemp = VCC - VRTh;
	   RTemp = VRTemp / (VRTh / R);		//Resistance of the thermistor

	   T0 = 25 + 273.15;
	   ln = log10(RTemp / RT0);
	   TX = (1 / ((ln / B) + (1 / T0))); //Temperature from thermistor

	   TX = TX - 273.15;                 //Conversion to Celsius


	   // Calculate light
	   /* EB = 10log(RA/RB) / gamma + log(EA)
	    * RA = 5-10 kOhm -> 8
	    * gamma = 0.5
	    *
	    * |
	    * ¡
	    * E = 640 * R^-2
	    */

	   VRLight = VCC - VRL;
	   RLight = VRLight / (VRL / R);		// Resistance of photoresistor

	   Light = 640 * pow((RLight / 1000), -2);	// Luminance in lux (resistances in kOhm)


	   // Calculate humidity
	   res = read_DHT11(buf);


		//Do measurements here, update sensor data
		headerMsg.myaddr = myAddr;
		headerMsg.msgType = MSG_TYPE_TEMP_HUM;
		sensorMsg.temperature = (uint16_t)TX;
		sensorMsg.humidity = buf[1];
		sensorMsg.light = (uint16_t)Light;
		/* sensorMsg.temperature = (uint16_t)tempArray[cntC];
		sensorMsg.humidity = (uint8_t)humArray[cntC];
		sensorMsg.light = (uint16_t)lightArray[cntC];*/
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

		// TODO beavatkozás
		// too hot: switch red led on
		if (TX > 30) {
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		}
		else {
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		}

		// too dark: switch green led on
		if (Light < 20) {
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			for(int i = 0; i < 10; i++) {
				//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				HAL_Delay(200);
			}
		}
		else {
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		}

		// too dry: flashing green led on
/*		if (Hum < 30) {
			for(int i = 1; i < 10; i++) {
				//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				HAL_Delay(200);
			}
		}
		else {
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		}*/
		if (cntC == 9){
			cntC = 0;
		}
		else {
			cntC++;
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
