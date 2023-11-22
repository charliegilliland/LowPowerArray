/*
 * user_app.h
 *
 *  Created on: Mar 22, 2023
 *      Author: charliegilliland
 */

#ifndef INC_USER_APP_H_
#define INC_USER_APP_H_

/*
 * Data acquisition state
 */
typedef enum {
	DAQ_IDLE = 0,	/* Waiting for a start condition (button press) */
	DAQ_RESTART,	/* re-sync to PPS - tells the PPS interrupt to signal when ready  */
	DAQ_READY,		/* PPS synced, ready to start data acquisition */
	DAQ_RUNNING,	/* Data acquisition is actively running */
	DAQ_ERROR		/* idk error */
} DAQ_STATUS;

/*
 * Function prototypes
 */
void user_app_init();
void user_app_loop();


#endif /* INC_USER_APP_H_ */
