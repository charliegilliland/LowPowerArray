/*
 * aud_mgr.h
 *
 *  Created on: Mar 22, 2023
 *      Author: charliegilliland
 */

/*
 * This code handles all things related to the audio interface (SAI).
 *
 * The aud_mgr_start function begins the audio interface in DMA mode, so that the audio buffer is filled "in the background"
 * Once a buffer is filled, the HAL_SAI_RxCpltCallback function gets called by the hardware.
 * In this callback function, we change the pointer to the next segment in a circular buffer, so that we continue to receive audio while doing other tasks.
 *
 * The user_app code will call the aud_read function, which returns a pointer to a segment in the audio buffer.
 * The user_app code must be fast enough to keep up with the audio data rate - this is why a circular buffer is used.
 *
 */

#ifndef INC_AUD_MGR_H_
#define INC_AUD_MGR_H_

#include <stdio.h>
#include <string.h>

#include "main.h"

/*
 * Audio buffer size defines
 */
#define AUD_BUF_BYTES ( 16 * 1024 ) // 16kB audio buffer
#define AUD_BUF_WORDS ( AUD_BUF_BYTES / sizeof(int32_t) )
#define BUF_MULT 6


/*
 * Channel count / sample rate
 */
#define N_CHAN 8
#define SAMPLE_RATE 48000
#define BYTES_PER_SAMPLE 4

// HAL status flag
static HAL_StatusTypeDef stat;


/*
 * External peripheral handle
 */
extern SAI_HandleTypeDef hsai_BlockA1;

// state indicator - declared in user_app.c
extern volatile DAQ_STATUS daq_stat;

volatile int32_t audio_buffer[AUD_BUF_WORDS * BUF_MULT];
volatile uint8_t rd_idx = 0, wr_idx = 0, num_ready = 0;


// Starts the audio rx
__attribute__((always_inline))
static __inline uint8_t aud_mgr_start()
{
	// reset error code
	hsai_BlockA1.ErrorCode = HAL_SAI_ERROR_NONE;

	// start SAI in DMA mode
	stat = HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t *)audio_buffer, AUD_BUF_WORDS);
	if (stat != HAL_OK) {
		printf("HAL_SAI_Receive_DMA failed on line %d stat = %d\n", __LINE__, stat);
		return 1;
	}

	return 0; // 0 indicates success
}

// Stops the audio rx
__attribute__((always_inline))
static __inline uint8_t aud_mgr_stop()
{
	// reset flags
	rd_idx = 0;
	wr_idx = 0;
	num_ready = 0;

	// stop SAI
	stat = HAL_SAI_DMAStop(&hsai_BlockA1);
	if (stat != HAL_OK) {
		printf("HAL_SAI_DMAStop failed: %d\n", stat);
		return 1;
	} else {
		printf("HAL_SAI_DMAStop success: %d\n", stat);
	}

	// abort DMA interrupt if its still running
	if (hsai_BlockA1.hdmarx->State == HAL_DMA_STATE_BUSY) {
		stat = HAL_DMA_Abort_IT(hsai_BlockA1.hdmarx);
		if (stat != HAL_OK) {
			printf("HAL_DMA_Abort_IT failed: %d\n", stat);
		}
	}

	return 0; // 0 indicates success
}

/*
 * This function gets called from the user_app super-loop
 */
__attribute__((always_inline))
static __inline int32_t *aud_read()
{
	if (!num_ready) {
		return NULL;
	}

	int32_t *buf = (int32_t *) &audio_buffer[rd_idx * AUD_BUF_WORDS];

	if (++rd_idx == BUF_MULT) {
		rd_idx = 0;
	}
	num_ready--;

	return buf;
}


/*
 * SAI callback functions
 */

// Receive complete callback
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	// change the buffer pointer
	stat = HAL_SAI_DMAPause(hsai);
	if (stat != HAL_OK) {
		printf("HAL_SAI_DMAPause returned %d\n", stat);
	}

	if (++wr_idx >= BUF_MULT) {
		wr_idx = 0;
	}

	hsai->pBuffPtr = (uint8_t *) &audio_buffer[wr_idx * AUD_BUF_WORDS];

	stat = HAL_DMA_Start_IT(hsai->hdmarx, (uint32_t)&hsai->Instance->DR, (uint32_t)hsai->pBuffPtr, hsai->XferSize);
	if (stat != HAL_OK) {
		printf("HAL_DMA_Start_IT returned %d\n", stat);
	}

	stat = HAL_SAI_DMAResume(hsai);
	if (stat != HAL_OK) {
		printf("HAL_SAI_DMAResume returned %d\n", stat);
	}

	num_ready++;
	if (num_ready == BUF_MULT) {
		printf("warn\n");
	} else if (num_ready > BUF_MULT) {
		printf("rb overflow\n");

		/******************************
		 ** Signal to re-sync to PPS **
		 ******************************/
		daq_stat = DAQ_RESTART;
		printf("Line %d\tdaq_stat = %d\n", __LINE__, daq_stat);
	}
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
	printf("HAL_SAI_ErrorCallback: 0x%lX\n", hsai->ErrorCode);

	if (hsai->ErrorCode != HAL_SAI_ERROR_OVR) { // ignore overrun error

		/******************************
		 ** Signal to re-sync to PPS **
		 ******************************/
		daq_stat = DAQ_RESTART;
		printf("Line %d\tdaq_stat = %d\n", __LINE__, daq_stat);
	}
	hsai->ErrorCode = 0;
}

#endif /* INC_AUD_MGR_H_ */
