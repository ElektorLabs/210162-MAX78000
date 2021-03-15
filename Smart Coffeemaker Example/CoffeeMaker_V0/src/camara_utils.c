/*
 * camara_utils.c
 *
 *  Created on: 9 Feb 2021
 *      Author: user
 */
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_sys.h"
#include "fcr_regs.h"
#include "mxc.h"
#include "icc.h"
#include "camera.h"


#define IMAGE_SIZE_X  (64)
#define IMAGE_SIZE_Y  (64)
#define CAMERA_FREQ   (10 * 1000 * 1000)

void capture_camera_img(void)
{
    camera_start_capture_image();

    while (1) {
        if (camera_is_image_rcv()) {
            return;
        }
    }
}

void capture_camera_start( void ){
	camera_start_capture_image(); //This will start the capture process
}

uint8_t capture_camera_done(void){
    if (camera_is_image_rcv()) {
        return 1;
    } else {
    	return 0;
    }
}


int init_Camera(){
	 /* Enable camera power */
	 Camera_Power(POWER_ON);
	 MXC_Delay(300000);
	 int dma_channel = MXC_DMA_AcquireChannel();
	 int ret = 0;
	 // Initialize camera.
	    printf("Init Camera.\n");
	    camera_init(CAMERA_FREQ);

	    ret = camera_setup(IMAGE_SIZE_X, IMAGE_SIZE_Y, PIXFORMAT_RGB888, FIFO_THREE_BYTE, USE_DMA, dma_channel);

	    if (ret != STATUS_OK) {
	        printf("Error returned from setting up camera. Error %d\n", ret);
	        return -1;
	    }
	    return ret;
}
