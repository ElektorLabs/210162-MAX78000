/* **** Includes **** */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "mxc_sys.h"
#include "fcr_regs.h"
#include "icc.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "i2s_regs.h"
#include "board.h"
#include "mxc.h"
#include "i2s.h"
#include "dma.h"
#include "led.h"
#include "pb.h"
#include "cnn.h"

#include "kw_detection.h"

#define VERSION   "0.0.1 (09/02/21)"
/* **** Definitions **** */

/* Enable/Disable Features */
//#define ENABLE_CLASSIFICATION_DISPLAY  // enables printing classification result
#define ENABLE_SILENCE_DETECTION         // Starts collecting only after avg > THRESHOLD_HIGH, otherwise starts from first sample
	#undef EIGHT_BIT_SAMPLES                 // samples from Mic or Test vectors are eight bit, otherwise 16-bit
#define ENABLE_MIC_PROCESSING            // enables capturing Mic, otherwise a header file Test vector is used as sample data

/*-----------------------------*/
/* keep following unchanged */
#define SAMPLE_SIZE         16384   // size of input vector for CNN, keep it multiple of 128
#define CHUNK               128     // number of data points to read at a time and average for threshold, keep multiple of 128
#define TRANSPOSE_WIDTH     128     // width of 2d data model to be used for transpose
#define NUM_OUTPUTS         21      // number of classes
#define I2S_RX_BUFFER_SIZE  128      // I2S buffer size

/*-----------------------------*/

/* Adjustables */
#define SAMPLE_SCALE_FACTOR         4       // multiplies 16-bit samples by this scale factor before converting to 8-bit
#define THRESHOLD_HIGH              250     // voice detection threshold to find beginning of a keyword
#define THRESHOLD_LOW               100     // voice detection threshold to find end of a keyword
#define SILENCE_COUNTER_THRESHOLD   20      // [>20] number of back to back CHUNK periods with avg < THRESHOLD_LOW to declare the end of a word
#define PREAMBLE_SIZE               30*CHUNK// how many samples before beginning of a keyword to include
#define INFERENCE_THRESHOLD         95      // min probability (0-100) to accept an inference

/* **** Globals **** */
volatile uint32_t cnn_time; // Stopwatch

static int32_t ml_data[NUM_OUTPUTS];
static q15_t ml_softmax[NUM_OUTPUTS];
uint8_t pAI85Buffer[SAMPLE_SIZE];
uint8_t pPreambleCircBuffer[PREAMBLE_SIZE];
int16_t Max, Min;
uint16_t thresholdHigh = THRESHOLD_HIGH;
uint16_t thresholdLow = THRESHOLD_LOW;

volatile uint8_t i2s_flag = 0;
int32_t i2s_rx_buffer[I2S_RX_BUFFER_SIZE];

/* Set of detected words */
const char keywords[NUM_OUTPUTS][10] = { "MARVIN", "HAPPY", "BACKWARD", "FORWARD", "STOP",
                                         "GO", "YES", "NO", "ON", "OFF", "ONE", "TWO", "THREE", "FOUR", "FIVE",
                                         "SIX", "SEVEN", "EIGHT", "NINE", "ZERO", "Unknown"
                                       };

mic_processing_state_t procState = STOP;
mic_processing_state_t savedprocState = STOP;

static int16_t detected_keyword=-1;


void StopSpotting(){
	savedprocState = procState;
	procState = STOP;
}

void ResumeSpotting(){
	if(procState==STOP){
		procState=savedprocState;
	}
}


char* GetKeywordFromIndex(int16_t idx){
	if(idx<20){
		return keywords[idx];
	} else {
		return NULL;
	}
}

int16_t GetDetectedKeywordID(void){

	int16_t ret=detected_keyword;
	return ret;

}

void AckKeyWord(void ){
	if( procState == KWDETECTED ){
		//I2SInit();
		procState = SILENCE;
	}
}



void i2s_isr(void)
{
    if(i2s_flag == 1){
    	printf("e"); //We have lost samples.....
    }
	i2s_flag = 1;
    /* Clear I2S interrupt flag */
    LED_Toggle(LED1);
    MXC_I2S_ClearFlags(MXC_F_I2S_INTFL_RX_THD_CH0);

}

/* **** Functions Prototypes **** */
void fail(void);
uint8_t cnn_load_data(uint8_t* pIn);
int8_t MicReader(int16_t* sample);
uint8_t MicReadChunk(uint8_t* pBuff, uint16_t* avg);
uint8_t AddTranspose(uint8_t* pIn, uint8_t* pOut, uint16_t inSize,
                     uint16_t outSize, uint16_t width);
uint8_t check_inference(q15_t* ml_soft, int32_t* ml_data,
                        int16_t* out_class, double* out_prob);
void I2SInit();
void HPF_init(void);
int16_t HPF(int16_t input);
/* **************************************************************************** */
uint32_t sampleCounter = 0;


   uint8_t pChunkBuff[CHUNK];

   uint16_t avg = 0;
   uint16_t preambleCounter = 0;
   uint16_t ai85Counter = 0;
   uint16_t wordCounter = 0;

   uint16_t avgSilenceCounter = 0;



void kw_detection_setup( void ){



	    /* Enable microphone power on Feather board */
	    Microphone_Power(POWER_ON);

	    /* Enable peripheral, enable CNN interrupt, turn on CNN clock */
	    /* CNN clock: 50 MHz div 1 */
	    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);

	    /* Configure P2.5, turn on the CNN Boost */
	    cnn_boost_enable(MXC_GPIO2, MXC_GPIO_PIN_5);

	    printf("Maxim Integrated \nKeyword Spotting Demo\nVer. %s \n", VERSION);
	    printf("\n***** Init *****\n");
	    memset(pAI85Buffer, 0x0, sizeof(pAI85Buffer));
	    memset(pPreambleCircBuffer, 0x0, sizeof(pPreambleCircBuffer));

	    printf("pChunkBuff: %d\n", sizeof(pChunkBuff));
	    printf("pPreambleCircBuffer: %d\n", sizeof(pPreambleCircBuffer));
	    printf("pAI85Buffer: %d\n", sizeof(pAI85Buffer));

	    /* Bring state machine into consistent state */
	    cnn_init();
	    /* Load kernels */
	    cnn_load_weights();
	    /* Configure state machine */
	    cnn_configure();

	    /* switch to silence state*/
	    procState = SILENCE;

	    /* initialize I2S interface to Mic */
	    I2SInit();
	    printf("\n*** READY ***\n");
}


mic_processing_state_t kw_detection_task(void)
{
	if(procState==KWDETECTED){
		return procState;
	}

	if(procState==STOP){
			return procState;
	}

        /* Read from Mic driver to get CHUNK worth of samples, otherwise next sample*/
        if (MicReadChunk(pChunkBuff, &avg) == 0) {
            return procState;
        }

        sampleCounter += CHUNK;


        /* copy the preamble data*/
        /* add the new chunk to the end of circular buffer*/
        memcpy(&pPreambleCircBuffer[preambleCounter], pChunkBuff, sizeof(uint8_t) * CHUNK);

        /* increment circular buffer pointer*/
        preambleCounter = (preambleCounter + CHUNK) % (PREAMBLE_SIZE);

        /* if we have not detected voice, check the average*/
        if (procState == SILENCE) {

            /* compute average, proceed if greater than threshold */
            if (avg >= thresholdHigh) {
                /* switch to keyword data collection*/
                procState = KEYWORD;
                printf("%.6d Word starts from index: %d, avg:%d > %d \n",
                       sampleCounter, sampleCounter - PREAMBLE_SIZE - CHUNK,
                       avg, thresholdHigh);

                /* reorder circular buffer according to time at the beginning of pAI85Buffer */
                if (preambleCounter == 0) {
                    /* copy latest samples afterwards */
                    if (AddTranspose(&pPreambleCircBuffer[0], pAI85Buffer,
                                     PREAMBLE_SIZE, SAMPLE_SIZE, TRANSPOSE_WIDTH)) {
                        printf("ERROR: Transpose ended early \n");
                    }
                }
                else {
                    /* copy oldest samples to the beginning*/
                    if (AddTranspose(&pPreambleCircBuffer[preambleCounter],
                                     pAI85Buffer, PREAMBLE_SIZE - preambleCounter,
                                     SAMPLE_SIZE, TRANSPOSE_WIDTH)) {
                        printf("ERROR: Transpose ended early \n");
                    }

                    /* copy latest samples afterwards */
                    if (AddTranspose(&pPreambleCircBuffer[0], pAI85Buffer,
                                     preambleCounter, SAMPLE_SIZE, TRANSPOSE_WIDTH)) {
                        printf("ERROR: Transpose ended early \n");
                    }

                }

                /* preamble is copied and state is changed, start adding keyword samples next run */
                ai85Counter += PREAMBLE_SIZE;
                return procState;

            }
        }
        /* if it is in data collection, add samples to buffer*/
        else if (procState == KEYWORD)
        {
            uint8_t ret = 0;

            /* add sample, rearrange buffer */
            ret = AddTranspose(pChunkBuff, pAI85Buffer, CHUNK, SAMPLE_SIZE,
                               TRANSPOSE_WIDTH);

            /* increment number of stored samples */
            ai85Counter += CHUNK;

            /* if there is silence after at least 1/3 of samples passed, increment number of times back to back silence to find end of keyword */
            if ((avg < thresholdLow) && (ai85Counter >= SAMPLE_SIZE / 3)) {
                avgSilenceCounter ++;
            }
            else {
                avgSilenceCounter = 0;
            }


            /* if this is the last sample and there are not enough samples to
             * feed to CNN, or if it is long silence after keyword,  append with zero (for reading file)
             */
            if (avgSilenceCounter > SILENCE_COUNTER_THRESHOLD)
            {
                memset(pChunkBuff, 0, CHUNK);
                printf("%.6d: Word ends, Appends %d zeros \n", sampleCounter,
                       SAMPLE_SIZE - ai85Counter);
                ret = 0;

                while (!ret) {
                    ret = AddTranspose(pChunkBuff, pAI85Buffer, CHUNK,
                                       SAMPLE_SIZE, TRANSPOSE_WIDTH);
                    ai85Counter += CHUNK;
                }
            }

            /* if enough samples are collected, start CNN */
            if (ai85Counter >= SAMPLE_SIZE) {
                int16_t out_class = -1;
                double probability = 0;

                /* reset counters */
                ai85Counter = 0;
                avgSilenceCounter = 0;

                /* new word */
                wordCounter++;

                /* change state to silence */
                procState = SILENCE;

                /* sanity check, last transpose should have returned 1, as enough samples should have already been added */
                if (ret != 1) {
                    printf("ERROR: Transpose incomplete!\n");
                    fail();
                }

                //----------------------------------  : invoke AI85 CNN
                printf("%.6d: Starts CNN: %d\n", sampleCounter, wordCounter);

                /* load to CNN */
                if (!cnn_load_data(pAI85Buffer)) {
                    printf("ERROR: Loading data to CNN! \n");
                    fail();
                }

                /* Start CNN */
                if (!cnn_start()) {
                    printf("ERROR: Starting CNN! \n");
                    fail();
                }

                /* Wait for CNN  to complete */
                while (cnn_time == 0) {

                }

                /* read data */
                cnn_unload((uint32_t*)ml_data);

                /* run softmax */
                softmax_q17p14_q15((const q31_t*) ml_data, NUM_OUTPUTS,
                                   ml_softmax);

                /* find detected class with max probability */
                ret = check_inference(ml_softmax, ml_data, &out_class, &probability);

                printf("----------------------------------------- \n");

                if (!ret) {
                    printf("LOW CONFIDENCE!: ");
                } else {
                	detected_keyword = out_class;
                	procState = KWDETECTED;
                }

                printf("Detected word: %s (%0.1f%%)", keywords[out_class],
                       probability);

                printf("\n----------------------------------------- \n");

                Max = 0;
                Min = 0;
                //------------------------------------------------------------

            }
        }



 return procState;

}

/* **************************************************************************** */


void I2SInit()
{
    mxc_i2s_req_t req;
    int32_t err;

    printf("\n*** I2S & Mic Init ***\n");
    /* Initialize High Pass Filter */
    HPF_init();
    /* Initialize I2S RX buffer */
    memset(i2s_rx_buffer, 0, sizeof(i2s_rx_buffer));
    /* Configure I2S interface parameters */
    req.wordSize    = MXC_I2S_DATASIZE_WORD;
    req.sampleSize  = MXC_I2S_SAMPLESIZE_THIRTYTWO;
    req.justify     = MXC_I2S_MSB_JUSTIFY;
    req.wsPolarity  = MXC_I2S_POL_NORMAL;
    req.channelMode = MXC_I2S_INTERNAL_SCK_WS_0;
    /* Get only left channel data from on-board microphone. Right channel samples are zeros */
    req.stereoMode  = MXC_I2S_MONO_LEFT_CH;
    req.bitOrder    = MXC_I2S_MSB_FIRST;
    /* I2S clock = PT freq / (2*(req.clkdiv + 1)) */
    /* I2S sample rate = I2S clock/64 = 16kHz */
    req.clkdiv      = 5;
    req.rawData     = NULL;
    req.txData      = NULL;
    req.rxData      = i2s_rx_buffer;
    req.length      = I2S_RX_BUFFER_SIZE;


    if ((err = MXC_I2S_Init(&req)) != E_NO_ERROR) {
        printf("\nError in I2S_Init: %d\n", err);

        while (1);
    }

    /* Set I2S RX FIFO threshold to generate interrupt */
    MXC_I2S_SetRXThreshold(4);
    NVIC_SetVector(I2S_IRQn, i2s_isr);
    NVIC_EnableIRQ(I2S_IRQn);
    NVIC_SetPriority(TMR1_IRQn,1); //highest priority -1 in system;
    /* Enable RX FIFO Threshold Interrupt */
    MXC_I2S_EnableInt(MXC_F_I2S_INTEN_RX_THD_CH0);
    MXC_I2S_RXEnable();
    __enable_irq();
}


/* **************************************************************************** */
uint8_t check_inference(q15_t* ml_soft, int32_t* ml_data,
                        int16_t* out_class, double* out_prob)
{

    int32_t temp[NUM_OUTPUTS];
    q15_t max = 0;    // soft_max output is 0->32767
    int32_t max_ml = 1 << 31; // ml before going to soft_max
    int16_t max_index = -1;

    memcpy(temp, ml_data, sizeof(int32_t) * NUM_OUTPUTS);

    /* find the top 5 classes */
    for (int top = 0; top < 5; top++) {
        /* find the class with highest */
        for (int i = 0; i < NUM_OUTPUTS; i++) {
            if ((int32_t) temp[i] > max_ml) {
                max_ml = (int32_t) temp[i];

                max = ml_soft[i];
                max_index = i;
            }
        }

        /* print top 1 separately */
        if (top == 0) {

            *out_class = max_index;
            *out_prob = 100.0 * max / 32768.0;
            break;
        }

    }

    printf("Min: %d,   Max: %d \n", Min, Max);

    /* check if probability is low */
    if (*out_prob > INFERENCE_THRESHOLD)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/* **************************************************************************** */
void fail(void)
{
    printf("\n*** FAIL ***\n\n");

    while (1);
}
/* **************************************************************************** */
uint8_t cnn_load_data(uint8_t* pIn)
{
    uint32_t mem;
    uint16_t index = 0;

    /* data should already be formated correctly */
    /* pIn is 16KB, each 1KB belongs to a memory group */
    for (mem = 0x50400000; mem <= 0x50418000; mem += 0x8000) {
        memcpy((uint8_t*)mem, &pIn[index], 1024);
        //printf("%.10X \n",(uint8_t *)mem);
        index += 1024;
    }

    for (mem = 0x50800000; mem <= 0x50818000; mem += 0x8000) {
        memcpy((uint8_t*)mem, &pIn[index], 1024);
        index += 1024;
    }

    for (mem = 0x50C00000; mem <= 0x50C18000; mem += 0x8000) {
        memcpy((uint8_t*)mem, &pIn[index], 1024);
        index += 1024;
    }

    for (mem = 0x51000000; mem <= 0x51018000; mem += 0x8000) {
        memcpy((uint8_t*)mem, &pIn[index], 1024);
        index += 1024;
    }

    return CNN_OK;
}
/* **************************************************************************** */
uint8_t AddTranspose(uint8_t* pIn, uint8_t* pOut, uint16_t inSize,
                     uint16_t outSize, uint16_t width)
{
    /* Data order in Ai85 memory (transpose is included):
    input(series of 8 bit samples): (0,0) ...  (0,127)  (1,0) ... (1,127) ...... (127,0)...(127,127)    16384 samples
    output (32bit word): 16K samples in a buffer. Later, each 1K goes to a seperate CNN memory group
    0x0000:
        (0,3)(0,2)(0,1)(0,0)
        (0,67)(0,66)(0,65)(0,64)
        (1,3)(1,2)(1,1)(1,0)
        (1,67)(1,66)(1,65)(1,64)
        ....
        (127,67)(127,66)(127,65)(127,64)
    0x0400:
        (0,7)(0,6)(0,5)(0,4)
        (0,71)(0,70)(0,69)(0,68)
        ....
        (127,71)(127,70)(127,69)(127,68)
    ...
    0x3C00:
        (0,63)(0,62)(0,61)(0,60)
        (0,127)(0,126)(0,125)(0,124)
        ....
        (127,127)(127,126)(127,125)(127,124)
    */

    static uint16_t row = 0, col = 0, total = 0;
    uint16_t  secondHalf = 0, wordRow = 0, byteInWord = 0, group = 0, index = 0;

    for (int i = 0; i < inSize; i++) {
        /* is it above 63? */
        if (col >= (width >> 1)) {
            secondHalf = 1; // odd word rows
        }
        else {
            secondHalf = 0; // even word rows
        }

        /* which group (0-15) it should be */
        group = (col % (width >> 1)) / 4;

        /* which word row (0-255) within the group */
        wordRow = secondHalf + (row << 1);

        /* which byte (0-3) in the word */
        byteInWord = col % 4;

        /* find output index */
        index = 1024 * group + 4 * wordRow + byteInWord;

        /* place sample in correct output location */
        pOut[index] = pIn[i];

        total++;

        /* increment row and col index */
        col++;

        if (col >= width) {
            col = 0;
            row++;
        }
    }

    if (total >= outSize) {
        /* sanity check */
        if (row != width) {
            printf("ERROR: Rearranging!\n");
        }

        total = 0;
        row = 0;
        col = 0;
        return 1;
    }
    else {
        return 0;
    }

}

/* **************************************************************************** */
uint8_t MicReadChunk(uint8_t* pBuff, uint16_t* avg)
{
    static uint16_t chunkCount = 0;
    static uint16_t sum = 0;

    static uint32_t index = 0;

    int32_t sample = 0;
    int16_t temp = 0;
    uint32_t rx_size = 0;

    /* clear overflow flags if and restart transfer */
    int flags = MXC_I2S_GetFlags();
    if(flags & MXC_F_I2S_INTFL_RX_OV_CH0){
     //printf("Overflow\n\r");
  	  MXC_I2S_ClearFlags(MXC_F_I2S_INTFL_RX_OV_CH0);
    }



    /* sample not ready */
    if (!i2s_flag) {
        *avg = 0;
        return 0;
    }

    /* Clear flag */
    i2s_flag = 0;
    /* Read number of samples in I2S RX FIFO */
    rx_size = MXC_I2S->dmach0 >> MXC_F_I2S_DMACH0_RX_LVL_POS;
//  printf("%d ", rx_size);

    /* read until fifo is empty or enough samples are collected */
    while ((rx_size--) && (chunkCount < CHUNK)) {
        /* Read microphone sample from I2S FIFO */
        sample = (int32_t)MXC_I2S->fifoch0;
        /* The actual value is 18 MSB of 32-bit word */
        temp = sample >> 14;

        /* Remove DC from microphone signal */
        sample = HPF((int16_t)temp); // filter needs about 1K sample to converge

        /* Discard first 10k samples due to microphone charging cap effect */
        if (index++ < 10000) {
            continue;
        }

        /* Turn on LED2 (Red) */
        LED_On(LED2);

        /* absolute for averaging */
        if (sample >= 0) {
            sum += sample;
        }
        else {
            sum -= sample;
        }

        /* Convert to 8 bit unsigned */
        pBuff[chunkCount] = (uint8_t)((sample) * SAMPLE_SCALE_FACTOR / 256);

        temp = (int8_t)pBuff[chunkCount];

        chunkCount++;

        /* record max and min */
        if (temp > Max) {
            Max = temp;
        }

        if (temp < Min) {
            Min = temp;
        }

    }

    /* if not enough samples, return 0 */
    if (chunkCount < CHUNK) {
        *avg = 0;
        return 0;
    }

    /* enough samples are collected, calculate average and return 1 */
    *avg = ((uint16_t)(sum / CHUNK));

    chunkCount = 0;
    sum = 0;
    return 1;
}

static int16_t  x0, x1, Coeff;
static int32_t  y0, y1;

/************************************************************************************/
void HPF_init(void)
{
    Coeff = 32604; //0.995
    x0 = 0;
    y0 = 0;
    y1 = y0;
    x1 = x0;
}

/************************************************************************************/
int16_t HPF(int16_t input)
{
    int16_t Acc, output;
    int32_t tmp;

    /* a 1st order IIR high pass filter (100 Hz cutoff frequency)  */
    /* y(n)=x(n)-x(n-1)+A*y(n-1) and A =.995*2^15 */

    x0 = input;

    tmp = (Coeff * y1);
    Acc = (int16_t)((tmp + (1 << 14)) >> 15);
    y0 = x0 - x1 + Acc;

    /* Clipping */
    if (y0 > 32767) {
        y0 = 32767;
    }

    if (y0 < -32768) {
        y0 = -32768;
    }

    /* Update filter state */
    y1 = y0;
    x1 = x0;

    output = (int16_t)y0;

    return (output);
}
/************************************************************************************/



