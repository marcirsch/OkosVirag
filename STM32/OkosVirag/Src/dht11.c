#include "dht11.h"
#include "gpio.h"

uint16_t read_cycle(uint16_t cur_tics, uint8_t neg_tic){
	uint16_t cnt_tics;
 	if (cur_tics < MAX_TICS) cnt_tics = 0;
	if (neg_tic){
		while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) && (cnt_tics < MAX_TICS)){
			cnt_tics++;
		}
	}
	else {
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) && (cnt_tics < MAX_TICS)){
			cnt_tics++;
		}
	}
 	return cnt_tics;
}

uint8_t read_DHT11(uint8_t *buf){
	uint16_t dt[42];
	uint16_t cnt;
	uint8_t i, check_sum; 
	
	//reset DHT11
	HAL_Delay(500);
 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	 
  //start reading	
 	cnt = 0; 
	for(i=0;i<83 && cnt<MAX_TICS;i++){
		if (i & 1){
			cnt = read_cycle(cnt, 1);
		}
		else {
			cnt = read_cycle(cnt, 0);
			dt[i/2]= cnt;
		}
	}
	
 	//release line
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	
	if (cnt>=MAX_TICS) return DHT11_NO_CONN;
	
	//convert data
 	for(i=2;i<42;i++){
		(*buf) <<= 1;
  	if (dt[i]>20) {
			(*buf)++;
 		}
		if (!((i-1)%8) && (i>2)) {
			buf++;
		}
 	}
	
	//calculate checksum
	buf -= 5;
	check_sum = 0;
 	for(i=0;i<4;i++){
		check_sum += *buf;
		buf++;
	}
	
	if (*buf != check_sum) return DHT11_CS_ERROR;
				
	return DHT11_OK;	
	//return check_sum;
}