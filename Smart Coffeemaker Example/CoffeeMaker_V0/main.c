/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*
******************************************************************************/

/**
 * @file    main.c
 * @brief   TFT Demo Example!
 *
 * @details
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_sys.h"
#include "fcr_regs.h"
#include "mxc.h"
#include "icc.h"

#include "tft_fthr.h"
#include "led.h"
#include "pb.h"

#include "camera.h"
#include "dma.h"
#include "tmr.h"
#include "lvgl.h"

#include "utils.h"
#include "camera_utils.h"
#include "kw_detection.h"

#define CONT_FREQ       	1000                   // (Hz)
#define CONT_TIMER      	MXC_TMR3            // Can be MXC_TMR0 through MXC_TMR5
#define CONT_CLOCK_SOURCE   MXC_TMR_APB_CLK     // \ref mxc_tmr_clock_t


typedef enum {
	s_idle =0,
	s_happy,
	s_haveacoffee,
	s_gotcup,
	s_error
}ui_state_t;


/**********************************************************/
/* This part is for the GUI
 *
 */
static lv_disp_buf_t disp_buf;
static volatile lv_color_t buf[LV_HOR_RES_MAX * 20];


#define CANVAS_WIDTH   64
#define CANVAS_HEIGHT  64
static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(CANVAS_WIDTH, CANVAS_HEIGHT)]; //This consumes 8192bytes RAM

lv_obj_t * canvas=NULL;
lv_obj_t *label=NULL;
lv_obj_t * img1=NULL;

LV_IMG_DECLARE(MAXIMG);
LV_IMG_DECLARE(BREWIMG); //License problems not included in final release
LV_IMG_DECLARE(MUGIMG);  //License problems not included in final release

void copy_camera_img(lv_obj_t* cv);

/**********************************************************/
/* Basically time ceeping for the Coffeemaker FSM
 *
 */

volatile static uint32_t ticks = 0; //1ms ticks since start
ui_state_t fsmstate = 0; //FSM State

uint8_t req_happyfsm_call=0; //Flag to request a call back for the FSM
uint32_t happyfsm_nextcall=0; //Timestamp for netx call to FSM

/*
 * Function prototypes
 */
void happy_fsm(uint8_t haskw, uint32_t t_now);
void setup(void);
void loop(void);




void ContinuousTimerHandler()
{
	ticks++;
	MXC_TMR_ClearFlags(CONT_TIMER);

}

void ContinuousTimer()
{

    NVIC_SetVector(TMR3_IRQn, ContinuousTimerHandler);
    NVIC_SetPriority(TMR3_IRQn,7); //Lowest priority in system;
    NVIC_EnableIRQ(TMR3_IRQn);

    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks = MXC_TMR_GetPeriod(CONT_TIMER, CONT_CLOCK_SOURCE, 128, CONT_FREQ);

    /*
    Steps for configuring a timer for PWM mode:
    1. Disable the timer
    2. Set the prescale value
    3  Configure the timer for continuous mode
    4. Set polarity, timer parameters
    5. Enable Timer
    */

    MXC_TMR_Shutdown(CONT_TIMER);

    tmr.pres = TMR_PRES_128;
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.bitMode = TMR_BIT_MODE_16B;
    tmr.clock = CONT_CLOCK_SOURCE;
    tmr.cmp_cnt = periodTicks;      //SystemCoreClock*(1/interval_time);
    tmr.pol = 0;

    if (MXC_TMR_Init(CONT_TIMER, &tmr, true) != E_NO_ERROR) {
        printf("Failed Continuous timer Initialization.\n");
        return;
    }

    printf("Continuous timer started.\n\n");
}




void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    MXC_TFT_WriteBufferRGB565(area->x1,area->y1,(uint8_t*)(color_p) ,w,h );
    lv_disp_flush_ready(disp);
}

/**************************************************************************************************
 *    Function      : last_key
 *    Description   : return the last key read by a dummy keyboard
 *    Input         : none
 *    Output        : uint32_t
 *    Remarks       : return the scan or ascii code
 **************************************************************************************************/
uint32_t last_key(){
	return 0;
}

/**************************************************************************************************
 *    Function      : key_pressed
 *    Description   : return if a key is pressed
 *    Input         : none
 *    Output        : bool
 *    Remarks       : none
 **************************************************************************************************/
bool key_pressed(){
	return false;
}

/**************************************************************************************************
 *    Function      : keyboard_read
 *    Description   : callback to read the dummy keyboard
 *    Input         : lv_indev_drv_t*, lv_indev_data_t
 *    Output        : bool
 *    Remarks       : none
 **************************************************************************************************/
bool keyboard_read(lv_indev_drv_t * drv, lv_indev_data_t*data){
  data->key = last_key();            /*Get the last pressed or released key*/

  if(false != key_pressed()){
	  data->state = LV_INDEV_STATE_PR;
  } else {
	  data->state = LV_INDEV_STATE_REL;
  }

  return false; /*No buffering now so no more data read*/
}

/**************************************************************************************************
 *    Function      : copy_camera_img
 *    Description   : copy camera image to a canvas object
 *    Input         : lv_obj_t*
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void copy_camera_img(lv_obj_t* cv) //to access data put a pointer here we will just update the canvas
{
	//IMAGE_SIZE_X
	//IMAGE_SIZE_Y
    uint8_t*   frame_buffer;
    uint32_t  imgLen;
    uint32_t  w, h, x, y;
    uint8_t* buffer;

    camera_get_image(&frame_buffer, &imgLen, &w, &h);
    buffer = frame_buffer;

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            uint8_t red=*buffer++; //R 5
            uint8_t green=*buffer++; //G 6
            uint8_t blue=*buffer++; //B 5
            lv_color_t c;
            uint16_t b = (blue >> 3) & 0x1f;
            uint16_t g = ((green >> 2) & 0x3f) << 5;
            uint16_t r = ((red >> 3) & 0x1f) << 11;
            c.full=(uint16_t) (r | g | b);

            //lv_canvas_set_px(canvas, x, y, c );
            lv_canvas_set_px(cv, (h-x), (w-y), c );
        }
    }
}

/**************************************************************************************************
 *    Function      : create_overlay_area
 *    Description   : creates a 64*64 pixel canvas to draw data on
 *    Input         : none
 *    Output        : none
 *    Remarks       : Not used in this code
 **************************************************************************************************/
void create_overlay_area(void)
{

    canvas = lv_canvas_create(lv_scr_act(), NULL);
    lv_canvas_set_buffer(canvas, cbuf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(canvas, NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 0);
    lv_canvas_fill_bg(canvas, LV_COLOR_SILVER, LV_OPA_COVER);
}

/**************************************************************************************************
 *    Function      : main
 *    Description   : main function
 *    Input         : none
 *    Output        : none
 *    Remarks       : Should not return
 **************************************************************************************************/
int main(void)
{
	setup();

	while(1==1){
		loop();
	}
}

/**************************************************************************************************
 *    Function      : setup
 *    Description   : only called once at startup
 *    Input         : none
 *    Output        : none
 *    Remarks       : put all your initalization code here
 **************************************************************************************************/
void setup(){
		/* Put all code for startup here */

		// Wait for PMIC 1.8V to become available, about 180ms after power up.
		MXC_Delay(200000);
		// Initialize DMA for camera interface
		MXC_DMA_Init();

		/* Enable cache */
	    MXC_ICC_Enable(MXC_ICC0);

	    /* Set system clock to 100 MHz */
	    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
	    SystemCoreClockUpdate();

	    /* Initialize TFT display */
	    MXC_TFT_Init(MXC_SPI0, 1, NULL, NULL);
	    //TFT_Feather_test();
	    MXC_TFT_SetRotation(ROTATE_90);

	    if (MXC_RTC_Init(0, 0) != E_NO_ERROR) { } //We don't care if this works
	    if (MXC_RTC_Start() != E_NO_ERROR) { } //We don't care if this works

	    kw_detection_setup(); //This loads the CNN into the AI accelerator

		/* The following part will setup the UI
		 *
		 * in this case we are using the LVGL for drawing
		 *
		 */
	    lv_init(); //Library initalization
	    lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 20); //Initializes buffer

	    /*Initialize the display*/
	    lv_disp_drv_t disp_drv;
	    lv_disp_drv_init(&disp_drv);
	    disp_drv.hor_res = 320;
	    disp_drv.ver_res = 240;
	    disp_drv.flush_cb = my_disp_flush; //This is the callback to write display data
	    disp_drv.buffer = &disp_buf; //The is the initialized drawing buffer
	    lv_disp_drv_register(&disp_drv);

	    /*Initialize the (dummy) input device driver*/
	    lv_indev_drv_t indev_drv;
	    lv_indev_drv_init(&indev_drv);
	    indev_drv.type = LV_INDEV_TYPE_KEYPAD;
	    indev_drv.read_cb = keyboard_read;
	    lv_indev_drv_register(&indev_drv);

	    /* Create simple label that we use to display text*/
	   label = lv_label_create(lv_scr_act(), NULL);
	   lv_label_set_long_mode(label, LV_LABEL_LONG_BREAK);
	   lv_obj_set_width(label, 320);
	   lv_label_set_text(label, "Just call HAPPY for a coffee");

	   lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);

	   /* This load the maxim image to our display
	    * it is stored as index image with 16 colors
	    * not nice but works quiet well
	    */
	   img1 = lv_img_create(lv_scr_act(), NULL);
	   lv_img_set_src(img1, &MAXIMG);
	   lv_obj_align(img1, NULL, LV_ALIGN_CENTER, 0, 0);

		/*
		 * init_Camera(); //We avoid it here as we have resource conficts tpreventing a startup
		 * create_overlay_area();
		 */


	   lv_task_handler(); /* let the GUI do its work */
	   ContinuousTimer(); //This will setup our 1ms timer
	    
	 

}

/**************************************************************************************************
 *    Function      : loop
 *    Description   : will be called as oftern as possible from main
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void loop(){
	/* we get a few more flags form the ms timer, alos flags for the lvgl takshander */
	/* Everyhing that shall run infinte will go here */

	uint32_t time = ticks;
	//Be aware that kw_detection_task() is timing sensitive for what ever reasons...
	if(KWDETECTED == kw_detection_task() ){
		happy_fsm(1,time); //If a keyword is detected we call our FSM to process it
	}

	if(req_happyfsm_call>0){ //If we need to do a callback for some image updates
		if(happyfsm_nextcall < time ){ //Not aware of an overflow but seems to work most of the time
			happy_fsm(0,time);
		}
	}


}

/**************************************************************************************************
 *    Function      : happy_fsm
 *    Description   : will be called by loop and execute the fsm
 *    Input         : uint32_t , uint32_t
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void happy_fsm(uint8_t haskw, uint32_t t_now){
		
	static uint32_t last_guiupdate=0;
		uint32_t elapsed = t_now - last_guiupdate;
		req_happyfsm_call=0;
		int16_t kw_idx=20;
		char* kw = NULL;
		//KW detected we do data processing and we need to restart the KWS as will most likly kill the audio portion with ethe tft update
		if(haskw > 0){
			kw_idx = GetDetectedKeywordID();
			kw = GetKeywordFromIndex(kw_idx);
			AckKeyWord();
		}

		if( (haskw>0) && (req_happyfsm_call>0)){
			return; //If we wait we don't care abot keywords....
		}

		switch(fsmstate){
			case s_idle:{
				//We display something not usefull
				
				if(kw_idx==1){
					lv_label_set_text(label, "My name is HAPPY do you like t coffee?");
					fsmstate = s_happy;
				} else {
					lv_label_set_text(label, "Just call HAPPY for a coffee");
					fsmstate = s_idle;
				}
				
			}break;

			case s_happy:{
				if(kw_idx==6) /*YES*/ {
					lv_label_set_text(label, "Have you inserted a Cup?");
					lv_img_set_src(img1, &MUGIMG);
					fsmstate = s_haveacoffee;
				} else if (kw_idx==7){
					lv_label_set_text(label, "Just call HAPPY for a coffee");
					fsmstate = s_haveacoffee;
				} else {					
					lv_label_set_text(label, "My name is HAPPY do you like t coffee?");
					fsmstate = s_happy;
				}
			
			}break;

			case s_haveacoffee:{
				if(kw_idx==6) /*YES*/ {
					lv_label_set_text(label, "Coffee will be served");
					//This one needs a timeout
					lv_img_set_src(img1, &BREWIMG);
					req_happyfsm_call=1;
					happyfsm_nextcall=t_now+10000;
					printf("Sleep till %i\r\n", happyfsm_nextcall);
					fsmstate = s_gotcup;
				} else if (kw_idx==7){
					lv_label_set_text(label, "My name is HAPPY do you like t coffee?");
					fsmstate = s_happy;
				} else {					
					lv_label_set_text(label, "Have you inserted a Cup?");
					fsmstate = s_haveacoffee;
				}
				
			} break;

			case s_gotcup:{
				printf("Got called at %i\r\n", t_now);
				lv_label_set_text(label, "Just call HAPPY for a coffee");
				lv_img_set_src(img1, &MAXIMG);
				fsmstate=s_idle;
			} break;

			case s_error:{
				lv_label_set_text(label, "Error!");
			}break;

			default:{
				lv_label_set_text(label, "?!");
			}break;
		}
		lv_refr_now(NULL);
		lv_tick_inc(elapsed);
		lv_task_handler(); /* let the GUI do its work */

		//We will do a I2S restart here...)
		last_guiupdate=t_now;



}


