/*
 * user_app.c
 *
 *  Created on: Mar 22, 2023
 *      Author: charliegilliland
 */

/*
 * Instead of keeping lots of code in the main.c file, this file contains the application run-time code.
 *
 * Code in this file should only deal with the high-level runtime behaviors.
 * Any peripheral-specific code should go in its own header file (***_mgr.h).
 *
 */


#include <stdio.h>  // printf
#include <string.h> // memset, etc

#include "main.h"
#include "user_app.h"

#include "sd_mgr.h"
#include "aud_mgr.h"
#include "gps_mgr.h"


/*
 * Set FILE_LEN_SEC such that BUFS_PER_FILE is a whole number integer
 */
#define FILE_LEN_SEC 32
#define BUFS_PER_FILE ( FILE_LEN_SEC * SAMPLE_RATE * N_CHAN * BYTES_PER_SAMPLE ) / AUD_BUF_BYTES

/*
 * External peripheral handle
 */
extern TIM_HandleTypeDef htim8;


/*******************************
 ** Flag indicating DAQ state **
 *******************************/
volatile DAQ_STATUS daq_stat = DAQ_IDLE;

/*
 * This function replaces init code before the loop in main.c
 * Assumes the MX_***_Init functions have already been called
 */
void user_app_init()
{
	// initialize the SD card
	uint8_t res = sd_mgr_init();
	if (res) {
		printf("sd_mgr_init returned %d\n", res);
		while (1) ;
	}

	// Initialize the GPS. If GPS_WAIT_LOCK == 1, this function will block until the GPS is locked
	gps_mgr_init();

	// Begin FSYNC generation
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	// wait for stable FSYNC
	HAL_Delay(2000);

	// Indicate GPS is locked
	for (int i = 0; i < 20; i++) {
		HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
		HAL_Delay(100);
	}

	/***************************************************************
	 ** Check MODE_SEL pin to determine Automatic or Manual start **
	 ***************************************************************/
	int run_mode = HAL_GPIO_ReadPin(MODE_SEL_GPIO_Port, MODE_SEL_Pin);
	if (run_mode) {
		printf("MODE Automatic\n");

		// immediately start DAQ
		daq_stat = DAQ_RESTART;
		printf("Line %d\tdaq_stat = %d\n", __LINE__, daq_stat);
	} else {
		printf("MODE Manual\n");

		// require button press for start/stop DAQ
		daq_stat = DAQ_IDLE;
		printf("Line %d\tdaq_stat = %d\n", __LINE__, daq_stat);
	}
}

/*
 * This function replaces the endless loop in main.c
 */
void user_app_loop()
{
	uint8_t res;
	int32_t *new_audio_data = NULL;

	uint8_t *new_gps_data = NULL;
	uint32_t gps_data_len = 0;

	/******************************
	 ** MAIN LOOP - runs forever **
	 ******************************/
	while (1) {
		// start new audio file
		res = sd_mgr_new_audio_file(BUFS_PER_FILE * AUD_BUF_BYTES);
		if (res) {
			printf("sd_mgr_new_audio_file returned %d on line %d\n", res, __LINE__);
			while (1) ;
		}

		// start new GPS file
		res = sd_mgr_new_gps_file(FILE_LEN_SEC * GPS_BUF_SIZE);
		if (res) {
			printf("sd_mgr_new_gps_file returned %d on line %d\n", res, __LINE__);
			while (1) ;
		}

		/*************************************
		 ** Continue DAQ, or wait for start **
		 *************************************/
		if (daq_stat != DAQ_RUNNING) {
			// wait for PPS sync
			while (daq_stat != DAQ_READY) ;

			// start audio
			res = aud_mgr_start();
			if (res) {
				printf("aud_mgr_start failed: %d\n", res);

				daq_stat = DAQ_RESTART;
				printf("Line %d\tdaq_stat = %d\n", __LINE__, daq_stat);

				continue; // go back to the main loop
			}

			// signal that DAQ is running
			daq_stat = DAQ_RUNNING;
			printf("Line %d\tdaq_stat = %d\n", __LINE__, daq_stat);
		}

		uint32_t n_writes = 0;

		/********************
		 ** RECORDING LOOP **
		 ********************/
		while (daq_stat == DAQ_RUNNING) {

			// Read data from the circular buffer
			new_audio_data = aud_read();
			if (new_audio_data == NULL) {
				continue;
			}

			// Write the buffer to the audio file
			res = sd_mgr_write_audio((uint8_t *)new_audio_data, AUD_BUF_BYTES);
			if (res) {
				printf("sd_mgr_write failed: %d\n", res);
				while (1) ;
			}

			// check if this is the last write of the file
			if (++n_writes == BUFS_PER_FILE) {
				break; // return to main loop
			}

			// check if there is GPS data available, write it to a text file if yes
			new_gps_data = gps_mgr_read(&gps_data_len);
			if (gps_data_len > 0 && new_gps_data != NULL) {
				sd_mgr_write_gps(new_gps_data, gps_data_len);

				// simple feedback with UART and LED
				HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
				printf("n_writes = %lu\n", n_writes);
			}
		}

		/*******************************
		 ** Continue DAQ, or stop DAQ **
		 *******************************/
		if (daq_stat == DAQ_RUNNING) {
			// still running, but start a new file
			continue;
		} else {
			// stop condition (button press) or SAI error
			res = aud_mgr_stop();
			if (res) {
				printf("aud_mgr_stop failed: %d\n", res);
			}
		}

		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, 1); // turn off the LED
	}

}

/*
 * GPIO interrupt callback function
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static const uint32_t deb_thresh = 200; // milliseconds
	static uint32_t this_press, last_press = 0;

	if(GPIO_Pin == USER_BTN_Pin) {
		// Button press interrupt

		// De-bounce
		this_press = HAL_GetTick();
		if (this_press - last_press < deb_thresh) {
			last_press = this_press;
			return;
		}
		last_press = this_press;

		/**********************************************
		 ** Signal to stop DAQ, or wait for PPS sync **
		 **********************************************/
		if (daq_stat == DAQ_RUNNING) {
			// signal to stop the DAQ
			daq_stat = DAQ_IDLE;
			printf("Line %d\tdaq_stat = %d\n", __LINE__, daq_stat);
		} else if (daq_stat == DAQ_IDLE) {
			// signal to wait for PPS sync
			daq_stat = DAQ_RESTART;
			printf("Line %d\tdaq_stat = %d\n", __LINE__, daq_stat);
		}

		printf("Button pressed\n");
	} else if (GPIO_Pin == PPS_INPUT_Pin) {
		// PPS falling edge interrupt

		/*******************************
		 ** Signal ready to start DAQ **
		 *******************************/
		if (daq_stat == DAQ_RESTART) {
			daq_stat = DAQ_READY;
			printf("Line %d\tdaq_stat = %d\n", __LINE__, daq_stat);
		}

	}
}


