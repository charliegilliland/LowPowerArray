/*
 * gps_mgr.h
 *
 *  Created on: Apr 14, 2023
 *      Author: charliegilliland
 */

/*
 * This code handles all things related to the GPS UART messages
 *
 * GPS is connected through UART2. We use DMA to read UART messages 'in the background'.
 * Once the messages have been received, the HAL_UARTEx_RxEventCallback function gets called by the hardware.
 * In that callback function, we copy the DMA buffer to a local buffer, then restart the UART DMA receive.
 *
 */

#ifndef INC_GPS_MGR_H_
#define INC_GPS_MGR_H_

#include <stdio.h>
#include <string.h>

#include "main.h"

#define GPS_WAIT_LOCK 0

// DMA buffer size
// TODO: these could possibly be smaller
#define GPS_BUF_SIZE 1024 // size of the DMA buffer
#define GPS_MSG_SIZE 512 // size of a single message buffer

/*
 * Peripheral handles
 */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;


/*
 * Function prototypes
 */
static inline uint8_t verify_checksum(char *msg);

/*
 * String buffers
 */
static uint8_t gps_buf[GPS_BUF_SIZE]; // used by DMA
static uint8_t msg_buf[GPS_BUF_SIZE]; // used by me (copied from gps_buf)

// specific message buffers, used to parse the messages (check if the GPS is locked or not)
static char gns_msg[GPS_MSG_SIZE];
static char zda_msg[GPS_MSG_SIZE];

// Indicates there is new gps data ready
volatile uint8_t gps_flag = 0;


// Initialization function
// if GPS_WAIT_LOCK == 1, wait for GPS lock
void gps_mgr_init()
{
	memset(gps_buf, 0, GPS_BUF_SIZE);
	memset(msg_buf, 0, GPS_BUF_SIZE);
	memset(gns_msg, 0, GPS_MSG_SIZE);
	memset(zda_msg, 0, GPS_MSG_SIZE);

	// Begin UART DMA with idle interrupt
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gps_buf, GPS_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

#if GPS_WAIT_LOCK
	while (1) {
		if (!gps_flag) {
			continue;
		}

		// separate and copy the two messages to their respective strings
		char *msg_start = (char *)msg_buf;
		char *msg_end = msg_start;

		for (int i = 0; i < 2; i++) {
			// find the first character
			msg_start = strchr(msg_start, '$');
			if (!msg_start) {
				printf("didnt find start\n");
				break;
			}

			// find the last character
			msg_end = strchr(msg_start, 0x0D);
			if (!msg_end) {
				printf("didnt find end\n");
				break;
			}

			// check the length of the message
			int msg_len = msg_end - msg_start;
			if (msg_len <= 0 || msg_len > GPS_MSG_SIZE-1) {
				printf("msg_len out of range: %d\n", msg_len);
				break;
			}

			// determine which message and copy to the respective string
			if (strncmp(&msg_start[3], "GNS", 3) == 0) {
				memcpy(gns_msg, msg_start, (msg_end - msg_start));
				gns_msg[msg_end - msg_start] = 0;

				if (!verify_checksum(gns_msg)) {
					printf("gns checksum mismatch!\n");
				} else {
					printf("gns_msg = '%s'\n", gns_msg);
				}
			} else if (strncmp(&msg_start[3], "ZDA", 3) == 0) {
				memcpy(zda_msg, msg_start, (msg_end - msg_start));
				zda_msg[msg_end - msg_start] = 0;

				if (!verify_checksum(zda_msg)) {
					printf("zda checksum mismatch!\n");
				} else {
					printf("zda_msg = '%s'\n", zda_msg);
				}
			} else {
				printf("Unrecognized message: '%s'\n", msg_start);
			}

			msg_start = msg_end; // set the starting point to the end, so we can find the next message
		}

		// pos_mode is the 7th field in the GNS message
		char *comma = gns_msg;
		for (int i = 0; i < 6; i++) {
			comma = strchr(comma+1, ',');
		}

		// check if the GPS is locked, AKA pos_mode == "AA"
		if (strncmp(comma+1, "AA", 2) == 0) {
			printf("GPS is locked\n");
			break;
		} else {
			printf("waiting for GPS lock\n");
		}

		// clear the msg_buf
		memset(msg_buf, 0, GPS_BUF_SIZE);
		gps_flag = 0;
	}
#endif

}

/*
 * GPS read function - gets called by user_app code
 */
__attribute__((always_inline))
static __inline uint8_t *gps_mgr_read(uint32_t *size)
{
	if (!gps_flag) {
		return NULL;
	}

	*size = strlen((const char *)msg_buf);
	gps_flag = 0;
	return msg_buf;
}


/*
 * UART callback function - will get called whenever the GPS is done sending messages (ie when the UART bus is idle)
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart2) {
		// copy to msg_buf
		memcpy(msg_buf, gps_buf, GPS_BUF_SIZE);

		// clear gps_buf
   	    memset(gps_buf, 0, GPS_BUF_SIZE);

		// set flag
		gps_flag = 1;

		// start the DMA again
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)gps_buf, GPS_BUF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
}

/*
 *
 * Utility functions
 *
 */

__attribute__((always_inline))
static __inline uint8_t verify_checksum(char *msg)
{
	// calculate the expected checksum and compare to the one received
	char *star = NULL;
	star = strchr(msg, '*');
	if (!star) {
		return 0;
	}

	// length of the message without the checksum
	size_t len_wo_cs = star - msg;
	if (len_wo_cs <= 0 || len_wo_cs >= GPS_MSG_SIZE) {
		return 0;
	}

	uint8_t calc_cs = 0;
	size_t i = 0;
	if (msg[0] == '$') {
		i++;
	}

	// calculate the expected checksum
	for (calc_cs = 0; (i < len_wo_cs) && msg[i]; i++) {
		calc_cs ^= ((uint8_t) msg[i]);
	}

	// parse the received checksum & compare to the calculated checksum
	uint8_t recv_cs = (uint8_t) strtol((star+1), NULL, 16);
	if (calc_cs != recv_cs) {
		return 0;
	}

	return 1; // 1 indicates success
}



#endif /* INC_GPS_MGR_H_ */
