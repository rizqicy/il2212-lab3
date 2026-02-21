#include <stdio.h>
#include "bsp.h"
#include "images_alt.h"
#include <pico/multicore.h>
#include <stdlib.h>  // add malloc support

#define PPM_WIDTH 32
#define PPM_HEIGHT 32


/*************************************************************/


/* ASCII char table */
uint8_t NR_ASCII_CHARS = 16;
char ascii[] = {' ','.',':','-','=','+','/','t','z','U','w','*','0','#','%','@'};


char gry_div4(char r, char g, char b){
    char tmp;
    float gray = ((float)(r) * 0.3125) + ((float)(g) * 0.5625) + ((float)(b) * 0.125);
    tmp = (char) gray/4;

    return tmp;
}

char to_ascii(char in){
    char tmp;
    uint16_t idx = in * (NR_ASCII_CHARS-1) / 255;
    tmp = (char) idx;
    return ascii[tmp];
}

/******************MULTI CORE SUPPORT*************************/

typedef enum{
    WRAP_IMAGE=0,
    UNWRAP_IMAGE,
    MAP_MATRIX,
    SUMCOLS_SUMROWS
} function_type_t;

static void core1_entry() {
    while (true) {
        uintptr_t in_raw = multicore_fifo_pop_blocking();
        char * in = (char *) in_raw;

        uint8_t w = (uint8_t) multicore_fifo_pop_blocking();

        uint8_t h = (uint8_t) multicore_fifo_pop_blocking();

        uintptr_t gry_raw = multicore_fifo_pop_blocking();
        char * imgGRY = (char *) gry_raw;

        uintptr_t ascii_raw = multicore_fifo_pop_blocking();
        char * imgASCII = (char *) ascii_raw;

        uint16_t i=w*h/2*3;
        uint16_t k=w/2 * h/2 /2;
        while(i < w*h*3){

            //RGB to gray and resize operation simultaneously
            imgGRY[k] = gry_div4(in[i],in[i+1],in[i+2]) + gry_div4(in[i+3],in[i+4],in[i+5]) + gry_div4(in[i+w*3],in[i+1+w*3],in[i+2+w*3]) + gry_div4(in[i+3+w*3],in[i+4+w*3],in[i+5+w*3]);
            //convert resized gray image to ascii image
            imgASCII[k] = to_ascii(imgGRY[k]);

            i += 6; //skip every 2 pixels
            if(i % (w*3) == 0) { //finished one row of image, skip next row due to resize
                i += w*3;
            }
            k++;
        }

        //send finished (to be acknowledge by core0)
        multicore_fifo_push_blocking(1);
    }
}



/*************************************************************/

char output;
uint32_t tStart;
uint32_t tStop;

/*************************************************************/

/**
 * @brief Main function.
 * 
 * @return int 
 */
int main()
{
    BSP_Init();             /* Initialize all components on the lab-kit. */

    uint8_t width = PPM_WIDTH;
    uint8_t height = PPM_HEIGHT;

    char* imgGRY;
    char* imgASCII;

    imgGRY = malloc(width*height/4*sizeof(char));
    imgASCII = malloc(width*height/4*sizeof(char));

    uint8_t img_id = 0;

    multicore_launch_core1(core1_entry);
    
    while (true) { 
        //BSP_ToggleLED(LED_GREEN);
        if(img_id == 4){
            img_id = 0;
        }

        uint8_t w = sequence1[img_id][0];
        uint8_t h = sequence1[img_id][1];
        uint8_t max_val = sequence1[img_id][2];
        char * in = &sequence1[img_id][3];
        
        tStart = time_us_32();

        // send data to core1
        multicore_fifo_push_blocking((uintptr_t)(char *)(uintptr_t) in);
        multicore_fifo_push_blocking(w);
        multicore_fifo_push_blocking(h);
        multicore_fifo_push_blocking((uintptr_t)(char *)(uintptr_t) imgGRY);
        multicore_fifo_push_blocking((uintptr_t)(char *)(uintptr_t) imgASCII);

        uint16_t i=0;
        uint16_t k=0;
        while(i < w*h/2*3){

            //RGB to gray and resize operation simultaneously
            imgGRY[k] = gry_div4(in[i],in[i+1],in[i+2]) + gry_div4(in[i+3],in[i+4],in[i+5]) + gry_div4(in[i+w*3],in[i+1+w*3],in[i+2+w*3]) + gry_div4(in[i+3+w*3],in[i+4+w*3],in[i+5+w*3]);
            //convert resized gray image to ascii image
            imgASCII[k] = to_ascii(imgGRY[k]);

            i += 6; //skip every 2 pixels
            if(i % (w*3) == 0) { //finished one row of image, skip next row due to resize
                i += w*3;
            }
            k++;
        }

        //acknowledge
        multicore_fifo_pop_blocking();

        tStop = time_us_32();
        printf("T diff=%.3f\n", tStop, (tStop-tStart)/1000.0f);

        /* PRINT RESULT */
        printf("Output:\n");
        for(uint16_t j = 0; j< h/2; j++) {
            for(uint16_t i = 0; i < w/2; i++){
                output = imgASCII[j*w/2 + i];
                printf("%c", output);
            }
            printf("\n");
        }
        printf("\n");

        //sleep_ms(1000); 
        img_id++;
    }
}
/*-----------------------------------------------------------*/
