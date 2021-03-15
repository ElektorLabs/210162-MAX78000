/*
 * camera_utils.h
 *
 *  Created on: 9 Feb 2021
 *      Author: user
 */

#ifndef INCLUDE_CAMERA_UTILS_H_
#define INCLUDE_CAMERA_UTILS_H_

void capture_camera_img(void);
int init_Camera( void );

void capture_camera_start( void );
uint8_t capture_camera_done(void);

#endif /* INCLUDE_CAMERA_UTILS_H_ */
