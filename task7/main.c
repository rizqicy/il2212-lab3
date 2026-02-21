#include <stdio.h>
#include <math.h>
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

bool controlActor(uint8_t min_brightness, uint8_t max_brightness){
    static uint8_t state[] = {255,255,255};
    bool output = false;

    uint16_t sum = (state[0] + state[1] + state[2])/3;
    if(sum < 128) {
        output = true;
    }
    else{
        output = false;
    }
    //update state
    state[2] = state[1];
    state[1] = state[0];
    state[0] = max_brightness - min_brightness;

    return output;
}

char correction(uint8_t min_brightness, uint8_t max_brightness, char in){
    // rescale h
    //   | hmax - hmin > 127 =  h
    //   | hmax - hmin > 63  = (h - hmin) * 2
    //   | hmax - hmin > 31  = (h - hmin) * 4
    //   | hmax - hmin > 15  = (h - hmin) * 8
    //   | otherwise         = (h - hmin) * 16

    if((max_brightness - min_brightness) > 127){
        return in;
    }
    else if((max_brightness - min_brightness) > 63){
        return (in - min_brightness)*2;
    }
    else if((max_brightness - min_brightness) > 31){
        return (in - min_brightness)*4;
    }
    else if((max_brightness - min_brightness) > 15){
        return (in - min_brightness)*8;
    }
    else{
        return (in - min_brightness)*16;
    }
}


char sobel_0(char *in, uint16_t index, uint8_t width, uint8_t height){
    int8_t k_x[]= {1,0,-1,2,0,-2,1,0,-1}; //sobel kernel X
    int8_t k_y[]= {1,2,1,0,0,0,-1,-2,-1}; //sobel kernel Y

    uint8_t a,b,c,d,e,f,g,h,i;

    a = in[index-width-1];
    b = in[index-width];
    c = in[index-width+1];

    d = in[index-1];
    e = in[index];
    f = in[index+1];

    g = in[index+width-1];
    h = in[index+width];
    i = in[index+width+1];

    // add padding pixel on the edges
    if(index < width){ //edge top
       a = d;
       b = e;
       c = f; 
    }
    if((index % width) == 0){   //edge left
        a = b;
        d = e;
        g = h;
    }
    if((index % width) == (width-1)){   //edge right
        c = b;
        f = e;
        i = h;
    }
    // if((index/width) == (height-1)){     //edge bottom
    //     g = d;
    //     h = e;
    //     i = f;
    // }

    //convolution
    int16_t gx = a*k_x[0] + c*k_x[2] + d*k_x[3] + f*k_x[5] + g*k_x[6] + i*k_x[8];
    int16_t gy = a*k_y[0] + b*k_y[1] + c*k_y[2] + g*k_y[6] + h*k_y[7] + i*k_y[8];

    float G = sqrt(gx*gx + gy*gy) / 4;

    return (char) G;
}

char sobel_1(char *in, uint16_t index, uint8_t width, uint8_t height){
    int8_t k_x[]= {1,0,-1,2,0,-2,1,0,-1}; //sobel kernel X
    int8_t k_y[]= {1,2,1,0,0,0,-1,-2,-1}; //sobel kernel Y

    uint8_t a,b,c,d,e,f,g,h,i;

    a = in[index-width-1];
    b = in[index-width];
    c = in[index-width+1];

    d = in[index-1];
    e = in[index];
    f = in[index+1];

    g = in[index+width-1];
    h = in[index+width];
    i = in[index+width+1];

    // add padding pixel on the edges
    // if(index < width){ //edge top
    //    a = d;
    //    b = e;
    //    c = f; 
    // }
    if((index % width) == 0){   //edge left
        a = b;
        d = e;
        g = h;
    }
    if((index % width) == (width-1)){   //edge right
        c = b;
        f = e;
        i = h;
    }
    if((index/width) == (height-1)){     //edge bottom
        g = d;
        h = e;
        i = f;
    }

    //convolution
    int16_t gx = a*k_x[0] + c*k_x[2] + d*k_x[3] + f*k_x[5] + g*k_x[6] + i*k_x[8];
    int16_t gy = a*k_y[0] + b*k_y[1] + c*k_y[2] + g*k_y[6] + h*k_y[7] + i*k_y[8];

    float G = sqrt(gx*gx + gy*gy) / 4;

    return (char) G;
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

        uint16_t i=w*h/2*3;
        uint16_t k=w/2 * h/2 /2;
        uint8_t min=255;
        uint8_t max=0;
        while(i < w*h*3){

            imgGRY[k] = gry_div4(in[i],in[i+1],in[i+2]) + gry_div4(in[i+3],in[i+4],in[i+5]) + gry_div4(in[i+w*3],in[i+1+w*3],in[i+2+w*3]) + gry_div4(in[i+3+w*3],in[i+4+w*3],in[i+5+w*3]);
            
            //brightness actor
            if (imgGRY[k] < min)    min = imgGRY[k];
            if (imgGRY[k] > max)    max = imgGRY[k];

            i += 6; //skip every 2 pixels
            if(i % (w*3) == 0) { //finished one row of image, skip next row due to resize
                i += w*3;
            }
            k++;
        }

        //send finished (to be acknowledge by core0)
        multicore_fifo_push_blocking(min);
        multicore_fifo_push_blocking(max);

        //receive data from core0
        uintptr_t ascii_raw = multicore_fifo_pop_blocking();
        char * imgASCII = (char *) ascii_raw;

        bool ctrl = (bool) multicore_fifo_pop_blocking();

        uint8_t half_w = w/2;
        uint8_t half_h = h/2;

        k = half_w * half_h/2;
        while(k < half_w * half_h){
            
            //correction operation
            if(ctrl){
                imgGRY[k] = correction(min, max, imgGRY[k]);         
            }

            //apply sobel filter (edge detection)
            char tmp = sobel_1(imgGRY,k,half_w,half_h);
            
            //convert corrected image to ascii image
            imgASCII[k] = to_ascii(tmp);

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
        
        uint8_t half_w = w/2;
        uint8_t half_h = h/2;

        tStart = time_us_32();

        // send data to core1
        multicore_fifo_push_blocking((uintptr_t)(char *)(uintptr_t) in);
        multicore_fifo_push_blocking(w);
        multicore_fifo_push_blocking(h);
        multicore_fifo_push_blocking((uintptr_t)(char *)(uintptr_t) imgGRY);

        uint16_t i=0;
        uint16_t k=0;
        uint8_t min=255;
        uint8_t max=0;
        while(i < w*h/2*3){

            //RGB to gray and resize operation simultaneously
            imgGRY[k] = gry_div4(in[i],in[i+1],in[i+2]) + gry_div4(in[i+3],in[i+4],in[i+5]) + gry_div4(in[i+w*3],in[i+1+w*3],in[i+2+w*3]) + gry_div4(in[i+3+w*3],in[i+4+w*3],in[i+5+w*3]);
            
            //brightness actor
            if (imgGRY[k] < min)    min = imgGRY[k];
            if (imgGRY[k] > max)    max = imgGRY[k];

            i += 6; //skip every 2 pixels
            if(i % (w*3) == 0) { //finished one row of image, skip next row due to resize
                i += w*3;
            }
            k++;
        }

        //acknowledge and receive (max,min) brightness from core1
        uint8_t min2 = (uint8_t) multicore_fifo_pop_blocking();
        uint8_t max2 = (uint8_t) multicore_fifo_pop_blocking();

        // update min and max based on core1 data
        if(min2 < min)  min = min2;
        if(max2 > max)  max = max2;

        // control signal for correction
        bool ctrl = controlActor(min, max);


        k = 0;
        while(k < half_w * half_h/2){
            
            //correction operation
            if(ctrl){
                imgGRY[k] = correction(min, max, imgGRY[k]);         
            }

            //apply sobel filter (edge detection)
            char tmp = sobel_0(imgGRY,k,half_w,half_h);
            
            //convert corrected image to ascii image
            imgASCII[k] = to_ascii(tmp);

            k++;
        }

        //CAVEAT: due to dependency of the corrected pixel, sobel operation on the core1 slightly delayed until first part finished
        //send data to core1
        multicore_fifo_push_blocking((uintptr_t)(char *)(uintptr_t) imgASCII);
        multicore_fifo_push_blocking(ctrl);

        //acknowledge
        multicore_fifo_pop_blocking();

        tStop = time_us_32();
        printf("T diff=%.3f\n", (tStop-tStart)/1000.0f);

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
