/*
 * sd_mgr.h
 *
 *  Created on: Mar 22, 2023
 *      Author: charliegilliland
 */

/*
 * This code handles all things related to the SD card and file writing.
 *
 * sd_mgr_init will initialize the SD card hardware and create a new directory in which to store audio and gps files.
 * user_app code will call the sd_mgr_write_* functions to write data to a file
 * user_app code will also call the sd_mgr_new_*_file to close the current file and begin a new file
 *
 */

#ifndef INC_SD_MGR_H_
#define INC_SD_MGR_H_

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "fatfs.h"

static FATFS sd_card;
static FIL sd_aud_file;
static FIL sd_gps_file;

static FRESULT sd_res;

static uint32_t aud_file_num, gps_file_num;

static char aud_file_name[64];
static char gps_file_name[64];
static const char aud_fname_fmt[] = "%s/audio%04lu.pcm";
static const char gps_fname_fmt[] = "%s/gps%04lu.txt";

static char directory_name[32];
static const char dir_fname_fmt[] = "files%04lu";
static const char dir_fname_pat[] = "files*";

extern uint8_t SD_initialize();

/*
 * TODO: there is a bug when the SD card is idle for some time... new writes fail
 * Need to have a way to re-initialize the SD card, or simply reset the entire device
 */

__attribute__((always_inline))
static __inline uint8_t sd_mgr_init()
{
	aud_file_num = 0;
	gps_file_num = 0;

	uint8_t bsp_res = 0;

	// Check if SD card is present
	bsp_res = BSP_SD_IsDetected();
	if(bsp_res != SD_PRESENT) {
		printf("SD card not detected!\n");
		return 1;
	} else {
		printf("SD card is detected\n");
	}

	// Initialize SD card
	bsp_res = SD_initialize();
	if(bsp_res != RES_OK) {
		printf("SD init failed: %d\n", bsp_res);
		return 1;
	} else {
		printf("SD init success: %d\n", bsp_res);
	}

	HAL_Delay(10);

	// Mount the SD card
	sd_res = f_mount(&sd_card, SDPath, 1);
	if (sd_res) {
		printf("Failed to mount: %d\n", sd_res);
		return 1;
	} else {
		printf("Successfully mounted\n");
	}

	// Read the files* directories, find the largest index
	FILINFO fno;
	DIR dir;
	int32_t tmp, largest = -1;

	/* Doing this twice, because it misses the oldest directory the first time around (idk why) */
	for (int i = 0; i < 2; i++) {
		sd_res = f_findfirst(&dir, &fno, "", dir_fname_pat);
		while (sd_res == FR_OK && fno.fname[0]) {
			int r = sscanf(fno.fname, dir_fname_fmt, &tmp);
			if (r == 1 && tmp > largest) {
				largest = tmp;
			}
			sd_res = f_findnext(&dir, &fno);
		}
	}

	// make a new directory
	memset(directory_name, 0, sizeof(directory_name));
	sprintf(directory_name, dir_fname_fmt, largest+1);
	sd_res = f_mkdir(directory_name);
	if (sd_res) {
		printf("mkdir failed: %d\n", sd_res);
		return 1;
	}

	printf("Created directory '%s'\n", directory_name);

	return 0; // 0 indicates success
}

/*
 * Closes file if open, and open new file
 */
__attribute__((always_inline))
static __inline uint8_t sd_mgr_new_audio_file(uint32_t size)
{
	// check if audio file is already open
	if (sd_aud_file.obj.fs != 0) {
		// close the file
		sd_res = f_close(&sd_aud_file);
		if (sd_res) {
			printf("Failed to close: %d\n", sd_res);
			return 1;
		} else {
			printf("Successfully closed '%s'\n", aud_file_name);
		}
	}

	// open a new audio file
	memset(aud_file_name, 0, sizeof(aud_file_name));
	sprintf(aud_file_name, aud_fname_fmt, directory_name, aud_file_num++);
	sd_res = f_open(&sd_aud_file, (void *)aud_file_name, FA_WRITE | FA_CREATE_ALWAYS);
	if (sd_res) {
		printf("Failed to open: %d\n", sd_res);
		return 1;
	} else {
		printf("Successfully opened '%s'\n", aud_file_name);
	}

	// use f_expand to get a continuous file to write
	sd_res = f_expand(&sd_aud_file, size, 0); // 0 lazy, 1 immediate
	if (sd_res) {
		printf("Failed to expand: %d\n", sd_res);
		return 1;
	}

	return 0; // 0 indicates success
}

/*
 * Closes file if open, and open new file
 */
__attribute__((always_inline))
static __inline uint8_t sd_mgr_new_gps_file(uint32_t size)
{
	// check if gps file is already open
	if (sd_gps_file.obj.fs != 0) {
		// close the file
		sd_res = f_close(&sd_gps_file);
		if (sd_res) {
			printf("Failed to close: %d\n", sd_res);
			return 1;
		} else {
			printf("Successfully closed '%s'\n", gps_file_name);
		}
	}

	// open a new gps file
	memset(gps_file_name, 0, sizeof(gps_file_name));
	sprintf(gps_file_name, gps_fname_fmt, directory_name, gps_file_num++);
	sd_res = f_open(&sd_gps_file, (void *)gps_file_name, FA_WRITE | FA_CREATE_ALWAYS);
	if (sd_res) {
		printf("Failed to open: %d\n", sd_res);
		return 1;
	} else {
		printf("Successfully opened '%s'\n", gps_file_name);
	}

	// use f_expand to get a continuous file to write
	sd_res = f_expand(&sd_gps_file, size, 0); // 0 lazy, 1 immediate
	if (sd_res) {
		printf("Failed to expand: %d\n", sd_res);
		return 1;
	}

	return 0; // 0 indicates success
}

/*
 * Writes a buffer to the audio file
 */
__attribute__((always_inline))
static __inline uint8_t sd_mgr_write_audio(uint8_t *data, size_t size)
{
	static uint32_t bytes_writ = 0;

	sd_res = f_write(&sd_aud_file, (void *)data, (UINT)size, (UINT *)&bytes_writ);
	if (sd_res) {
		printf("error on audio write: sd_res: %d\n", sd_res);
		return 1;
	}

	return 0;
}

/*
 * Writes a buffer to the gps file
 */
__attribute__((always_inline))
static __inline uint8_t sd_mgr_write_gps(uint8_t *data, size_t size)
{
	static uint32_t bytes_writ = 0;

	sd_res = f_write(&sd_gps_file, (void *)data, (UINT)size, (UINT *)&bytes_writ);
	if (sd_res) {
		printf("error on gps write: sd_res: %d\n", sd_res);
		return 1;
	}

	return 0;
}

#endif /* INC_SD_MGR_H_ */
