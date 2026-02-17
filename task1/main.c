#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "bsp.h"

#include "images_alt.h"
#include "skeleton_v2.h"
#include "vector.h"

#define PPM_WIDTH 32
#define PPM_HEIGHT 32

TaskHandle_t    blinkTsk; /* Handle for the LED task. */
TaskHandle_t    ReadImageTsk; /* Handle for reading image data */
TaskHandle_t    graySDFTsk; /* Handle for graySDF task */
TaskHandle_t    asciiSDFTsk; /* Handle for asciiSDF task. */
QueueHandle_t   s_in;
QueueHandle_t   s_1;
QueueHandle_t   s_out;

/**
 * @brief Blink task.
 * 
 * @param args Task period (uint32_t).
 */
void blink_task(void *args);

void graySDF_task(void *args);
void asciiSDF_task(void *args);


/*************************************************************/

/* ASCII char table */
uint8_t NR_ASCII_CHARS = 16;
char ascii[] = {' ','.',':','-','=','+','/','t','z','U','w','*','0','#','%','@'};

/* Definition of the channel */
//typedef cbuf_handle_t channel;

/* global variable matrix(s)*/
vect_t mRGB;
vect_t mGRY1;
vect_t mGRY2;
vect_t mASCII;


/* Definition of the functions 'readToken' and 'writeToken' */
void readToken(QueueHandle_t ch, char* data) {
    xQueueReceive(ch, data, (TickType_t) portMAX_DELAY);
    //return circular_buf_get(ch, data);
}

void writeToken(QueueHandle_t ch, char data) {
    char c = data;
    xQueueSend(ch, &c, (TickType_t) portMAX_DELAY);
    //circular_buf_put(ch, data);
}

void actor11SDF(uint16_t consum, uint16_t prod,
                QueueHandle_t* ch_in, QueueHandle_t* ch_out,
                void (*f) (char*, char*, uint8_t, uint8_t), uint8_t w, uint8_t h)
{
    char input[consum], output[prod];
    uint16_t i;

    for(i = 0; i < consum; i++) {
        readToken(*ch_in, &input[i]);
    }
    f(input, output, w, h);
    for(i = 0; i< prod; i++) {	 
        writeToken(*ch_out, output[i]);
    }
}


void wrapImageRGB(char* in, vect_handle_t vec_out, uint8_t w, uint8_t h){
    vect_handle_t temp;

    for(int i = 0; i < h; i++){
        //printf("h=%d | ", i);
        for(int j = 0; j < w; j++){
            int idx = (i * w + j)*3;
            pixel_t p = {in[idx], in[idx+1], in[idx+2]};
            vect_write(&((vect_t *) vec_out->data)[i], j, &p);
            //printf("%d,%d,%d\t",p.r,p.g,p.b);
        }
        //printf("\n");
    }
}

void wrapImageGRY(char* in, vect_handle_t vec_out, uint8_t w, uint8_t h){
    vect_handle_t temp;

    for(int i = 0; i < h; i++){
        //printf("h=%d | ", i);
        for(int j = 0; j < w; j++){
            int idx = i*w + j;
            vect_write(&((vect_t *) vec_out->data)[i], j, &in[i*w + j]);
            //printf("i=%d,%d\t",idx, in[i*w+j]);
        }
        //printf("\n");
    }
}


void* grayscale(void* pin){
    pixel_t *in = (pixel_t *)pin;
    static char tmp = 0;

    float gray = ((float)(in->r) * 0.3125) + ((float)(in->g) * 0.5625) + ((float)(in->b) * 0.125);
    tmp = (char) gray;
    //printf("%2.2f %d ", gray, tmp);

    return &tmp;
}

void* convert_ascii(void* pin){
    char *in = (char *)pin;
    static char tmp = 0;

    uint8_t idx = *in * (NR_ASCII_CHARS-1) / 255;
    tmp = ascii[idx];

    return &tmp;
}

/*
-- | The graySDF actor, which transforms a every RGB image from a stream
-- into a grayscale one. In this case it is assumed that the X dimension holds the additional information.
graySDF :: Int -- ^ dimension X for input images
        -> Int -- ^ dimension Y for input images
        -> Signal Int -- ^ stream of pixels from the RGB image
        -> Signal Double -- ^ stream of resulting grayscale pixels
graySDF dimX dimY = actor11SDF (3 * dimX * dimY) (dimX * dimY) (wrapImageF (3 * dimX) dimY grayscale)


-- | Converts an image of pixels into its grayscale equivalent using
-- the formula <<resources/grayscale.png>>
grayscale :: Image Int -> Image Double
grayscale = mapMatrix (convert . fromVector) . mapV (groupV 3)
  where
    convert [r,g,b] = fromIntegral r * 0.3125
                    + fromIntegral g * 0.5625
                    + fromIntegral b * 0.125
    convert _ = error "X length is not a multiple of 3"
*/
void f_grayscale(char* in, char* out, uint8_t w, uint8_t h) {
    // read data stream to matrix
    //vect_t image = createMatrix_pixel(in, w, h);
    wrapImageRGB(in, &mRGB, w, h);

    // mapMatrix operation with grayscale function on each element
    //vect_t grayImage = mapV(&image,f_gray); 
    //vect_t grayImage = mapMatrix(&image, grayscale); 
    mapMatrix(&mRGB, &mGRY1, grayscale);

    //write matrix to data stream
    vect_handle_t vT;
    for(uint8_t i = 0; i < mGRY1.len; i++){
        vT = (vect_t *) vect_read(&mGRY1,i);
        for(uint8_t j = 0; j < vT->len; j++){
            char val = *(char *) vect_read(vT,j);
            out[i*(vT->len) + j] = val;
            //printf("%d\t", val);
        }
        //printf("\n");
    }
}


/*
-- | The ASCII actor, which outputs the ASCII 'art' of the image stream.
asciiSDF :: Int -- ^ dimension X for input images
         -> Int -- ^ dimension Y for input images
         -> Signal Double -- ^ stream of pixels to be printed
         -> Signal Char -- ^ stream of resulting ASCII pixels
asciiSDF dimX dimY = actor11SDF (dimX * dimY) (dimX * dimY) (wrapImageF dimX dimY toAsciiArt)


-- | Converts a 256-value grayscale image to a 16-value ASCII art
-- image.
toAsciiArt :: Image Double -> Image Char
toAsciiArt = mapMatrix num2char
  where
    num2char n = asciiLevels !! level n
    level n = truncate $ nLevels * (n / 255)
    nLevels = fromIntegral $ length asciiLevels - 1
*/
void f_ascii(char* in, char* out, uint8_t w, uint8_t h) {
    // read data stream to matrix
    //vect_t grayImage = createMatrix_gry(in, w, h);
    wrapImageGRY(in, &mGRY2, w, h);

    // mapMatrix operation with convert_ascii function on each element
    //vect_t asciiImage = mapMatrix(&grayImage, convert_ascii); 
    mapMatrix(&mGRY2, &mASCII, convert_ascii);

    //write matrix to data stream
    vect_handle_t vT;
    for(uint8_t i = 0; i < mASCII.len; i++){
        vT = vect_read(&mASCII,i);
        for(uint8_t j = 0; j < vT->len; j++){
            char val = *(char *) vect_read(vT,j);
            out[i*(vT->len) + j] = val;
            //printf("%c", val);
        }
        //printf("\n");
    }
}

/*************************************************************/

uint32_t tStart;
uint32_t tStop;
/**
 * @brief Main function.
 * 
 * @return int 
 */
int main()
{
    BSP_Init();             /* Initialize all components on the lab-kit. */
    
    /* Create the tasks. */
    xTaskCreate(blink_task, "Blink Task", 512, (void*) 1000, 2, &blinkTsk);
    xTaskCreate(graySDF_task, "graySDF Task", 2048, (void*) 3000, 2, &graySDFTsk);
    xTaskCreate(asciiSDF_task, "asciiSDF Task", 2048, (void*) 3000, 2, &asciiSDFTsk);

    uint8_t width = PPM_WIDTH;
    uint8_t height = PPM_HEIGHT;

    vect_init(&mRGB,height,sizeof(vect_t));
    for(int i = 0; i < height; i++){
        vect_init(&((vect_t *) mRGB.data)[i], width, sizeof(pixel_t));
    }    

    vect_init(&mGRY1,height,sizeof(vect_t));
    for(int i = 0; i < height; i++){
        vect_init(&((vect_t *) mGRY1.data)[i], width, sizeof(char));
    }

    vect_init(&mGRY2,height,sizeof(vect_t));
    for(int i = 0; i < height; i++){
        vect_init(&((vect_t *) mGRY2.data)[i], width, sizeof(char));
    }

    vect_init(&mASCII,height,sizeof(vect_t));
    for(int i = 0; i < height; i++){
        vect_init(&((vect_t *) mASCII.data)[i], width, sizeof(char));
    }

    /* Message Queue(s) --> SDF arc */
    s_in = xQueueCreate(3*width*height, sizeof(char));
    s_1 = xQueueCreate(width*height, sizeof(char));
    s_out = xQueueCreate(width*height, sizeof(char));

    vTaskStartScheduler();  /* Start the scheduler. */
    
    while (true) { 
        sleep_ms(1000); /* Should not reach here... */
    }
}
/*-----------------------------------------------------------*/

void blink_task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    for (;;) {
        BSP_ToggleLED(LED_GREEN);

        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/

void graySDF_task(void *args){
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */
    TickType_t time_start;

    //uint32_t tStart;

    uint8_t img_id = 0;

    for(;;){
        /* reset iteration */
        if (img_id == 4) {
            img_id = 0;
        }

        uint8_t w = sequence1[img_id][0];
        uint8_t h = sequence1[img_id][1];
        uint8_t max_val = sequence1[img_id][2];
        
        // for(uint16_t i = 0; i < (3*w*h); i++){
        //     xQueueSend(s_in, &sequence1[img_id][i+3], (TickType_t) portMAX_DELAY);
        // }
        //time_start = xTaskGetTickCount();
        tStart = time_us_32();

        /* Read input tokens */
        for(uint16_t j = 0; j < (3*w*h); j++) {
            writeToken(s_in, sequence1[img_id][j+3]);
        }

        /* graySDF actor */
        actor11SDF(3*w*h, w*h, &s_in, &s_1, f_grayscale, w, h);

        //printf("st=%d\t", time_start);
        printf("st=%d\t", tStart);
        img_id++;
    }
}


void asciiSDF_task(void *args){
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */
    TickType_t time_stop;

    char output;

    //uint32_t tStop;
    
    uint8_t w = PPM_WIDTH;
    uint8_t h = PPM_HEIGHT;

    for(;;){
        /* asciiSDF actor */
        actor11SDF(w*h, w*h, &s_1, &s_out, f_ascii, w, h);

        //time_stop = xTaskGetTickCount();
        //printf("sp=%d\n", time_stop);

        tStop = time_us_32();
        printf("sp=%d, diff=%.3f\n", tStop, (tStop-tStart)/1000.0f);
        //xQueueReceive(s_out, &c, (TickType_t) portMAX_DELAY);
        //printf("%d ",c);

        /* Write output tokens */
        //printf("Output:\n");
        for(uint16_t j = 0; j<h; j++) {
            for(uint16_t i = 0; i < w; i++){
                readToken(s_out, &output);
                //printf("%c", output);
            }
            //printf("\n");
        }
        //printf("\n");

    }
}

void graySDF123_task(void *args){
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    uint8_t img_id = 0;
    
    for(;;){
        if (img_id == 4) {
            img_id = 0;
        }
        
        uint8_t w = sequence1[img_id][0];
        uint8_t h = sequence1[img_id][1];
        uint8_t max_val = sequence1[img_id][2];
    //    for (uint32_t i = 3; i < (w*h); i++){
    //        char x = sequence1[img_id][i];
    //         if(i % w == 0){
    //             printf("\n");
    //         }
    //        printf("%d ",x);

    //    }

       vect_t A = createMatrix_pixel(sequence1[img_id]+3,w,h);

        for(uint i = 0; i < h; i++){
            vect_t vT = *(vect_t *)vect_read(&A, i);
            for(uint8_t j = 0; j < w; j++){
                pixel_t p = *(pixel_t *) vect_read(&vT, j);
                printf("%d,%d,%d ",p.r,p.g,p.b);
            }
            printf("\n");
        }
        printf("SEQ IMG %d == END\n", img_id);
        img_id++;
    
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
