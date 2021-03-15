/*
 * kw_detection.h
 *
 *  Created on: 9 Feb 2021
 *      Author: user
 */

#ifndef INCLUDE_KW_DETECTION_H_
#define INCLUDE_KW_DETECTION_H_

/* **** Constants **** */
typedef enum {
    STOP = 0,     /* No processing  */
    SILENCE = 1,  /* Threshold not detected yet  */
    KEYWORD = 2,   /* Threshold has been detected, gathering keyword samples */
	KWDETECTED=3  /* KW has been detected and we wait for system processing */
} mic_processing_state_t;

void kw_detection_setup( void );
mic_processing_state_t kw_detection_task(void);
void StopSpotting();
void ResumeSpotting();
char* GetKeywordFromIndex(int16_t idx);
int16_t GetDetectedKeywordID(void);
void AckKeyWord(void );

#endif /* INCLUDE_KW_DETECTION_H_ */
