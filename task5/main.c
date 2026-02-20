#include <stdio.h>
#include "bsp.h"
#include "images_alt.h"
#include "vector.h"
#include "skeleton_v2.h"
#include <pico/multicore.h>

#define PPM_WIDTH 32
#define PPM_HEIGHT 32


/*************************************************************/


/* ASCII char table */
uint8_t NR_ASCII_CHARS = 16;
char ascii[] = {' ','.',':','-','=','+','/','t','z','U','w','*','0','#','%','@'};

/* global variable matrix(s)*/
vect_t mRGB;
vect_t mGRY;
vect_t mRSZ;
vect_t mASCII;

void wrapImageRGB_0(char* in, vect_handle_t vec_out){
    vect_handle_t vT;

    for(uint8_t i = 0; i < (vec_out->len)/2; i++){
        //printf("h=%d | ", i);
        vT = (vect_t *) vect_read(vec_out, i);
        for(uint8_t j = 0; j < (vT->len); j++){
            uint16_t idx = (i * (vT->len) + j)*3;
            pixel_t p = {in[idx], in[idx+1], in[idx+2]};
            vect_write(vT, j, &p);
            //printf("%d,%d,%d\t",p.r,p.g,p.b);
        }
        //printf("\n");
    }
}

void wrapImageRGB_1(char* in, vect_handle_t vec_out){
    vect_handle_t vT;

    for(uint8_t i = (vec_out->len)/2; i < (vec_out->len); i++){
        //printf("h=%d | ", i);
        vT = (vect_t *) vect_read(vec_out, i);
        for(uint8_t j = 0; j < (vT->len); j++){
            uint16_t idx = (i * (vT->len) + j)*3;
            pixel_t p = {in[idx], in[idx+1], in[idx+2]};
            vect_write(vT, j, &p);
            //printf("%d,%d,%d\t",p.r,p.g,p.b);
        }
        //printf("\n");
    }
}

void mapMatrix_0(vect_handle_t mat_in, vect_handle_t mat_out, void* (*f)(void*)){
    vect_handle_t vT;

    for(int i=0; i < (mat_in->len)/2; i++){
        vT = (vect_t *)vect_read(mat_in,i);
        for(int j=0; j < (vT->len); j++){
            //pixel_t a = *(pixel_t *) vect_read(&temp,j);
            //int sum = a.r + a.g + a.b;
            //printf("j=%d,%d,%d,%d,%d\t",j,a.r,a.g,a.b, sum);
            vect_write(&((vect_t *) mat_out->data)[i],j, f(vect_read(vT,j)));
            //vect_write(&vec_out, i, f(vect_read(vec_in,i)));
        }
   }
}


void mapMatrix_1(vect_handle_t mat_in, vect_handle_t mat_out, void* (*f)(void*)){
    vect_handle_t vT;

    for(int i=(mat_in->len)/2; i < (mat_in->len); i++){
        vT = (vect_t *)vect_read(mat_in,i);
        for(int j=0; j < (vT->len); j++){
            //pixel_t a = *(pixel_t *) vect_read(&temp,j);
            //int sum = a.r + a.g + a.b;
            //printf("j=%d,%d,%d,%d,%d\t",j,a.r,a.g,a.b, sum);
            vect_write(&((vect_t *) mat_out->data)[i],j, f(vect_read(vT,j)));
            //vect_write(&vec_out, i, f(vect_read(vec_in,i)));
        }
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

void* grayscale_1(void* pin){
    pixel_t *in = (pixel_t *)pin;
    static char tmp = 0;

    float gray = ((float)(in->r) * 0.3125) + ((float)(in->g) * 0.5625) + ((float)(in->b) * 0.125);
    tmp = (char) gray;
    //printf("%2.2f %d ", gray, tmp);

    return &tmp;
}

void* grayscale_div4(void* pin){
    pixel_t *in = (pixel_t *)pin;
    static char tmp = 0;

    float gray = ((float)(in->r) * 0.3125) + ((float)(in->g) * 0.5625) + ((float)(in->b) * 0.125);
    tmp = (char) gray/4;
    //printf("%2.2f %d ", gray, tmp);

    return &tmp;
}

void* grayscale_div4_1(void* pin){
    pixel_t *in = (pixel_t *)pin;
    static char tmp = 0;

    float gray = ((float)(in->r) * 0.3125) + ((float)(in->g) * 0.5625) + ((float)(in->b) * 0.125);
    tmp = (char) gray/4;
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

void* convert_ascii_1(void* pin){
    char *in = (char *)pin;
    static char tmp = 0;

    uint8_t idx = *in * (NR_ASCII_CHARS-1) / 255;
    tmp = ascii[idx];

    return &tmp;
}

void* f_sum (void* pin1,void* pin2){
    char *in1 = (char *)pin1;
    char *in2 = (char *)pin2;
    static char tmp = 0;     //temporary memory

    tmp = *in1 + *in2;
    return &tmp;
}

void* f_sum_1 (void* pin1,void* pin2){
    char *in1 = (char *)pin1;
    char *in2 = (char *)pin2;
    static char tmp = 0;     //temporary memory

    tmp = *in1 + *in2;
    return &tmp;
}

void* div_4(void* pin){
    char* in = (char *) pin;
    static char tmp =0;

    tmp = *in/4;

    return &tmp;
}

void* div_4_1(void* pin){
    char* in = (char *) pin;
    static char tmp =0;

    tmp = *in/4;

    return &tmp;
}

void sumCols_sumRows_0(vect_handle_t vec_in, vect_handle_t vec_out){
    vect_handle_t vM0, vM1, vR;
    for(uint8_t i =0; i< (vec_in->len)/2; i+=2){
        vM0 = (vect_t *) vect_read(vec_in,i);
        vM1 = (vect_t *) vect_read(vec_in,i+1);

        zipWithV(vM0,vM1, vM0, f_sum);

        vR = (vect_t *) vect_read(vec_out, i/2);
        for(uint8_t j = 0; j< vM0->len; j+=2){
            char sum = *(char *)vect_read(vM0,j) + *(char *)vect_read(vM0,j+1);
            vect_write(vR, j/2, &sum);
        }
    }

}

void sumCols_sumRows_1(vect_handle_t vec_in, vect_handle_t vec_out){
    vect_handle_t vM0, vM1, vR;
    for(uint8_t i =(vec_in->len)/2; i< (vec_in->len); i+=2){
        vM0 = (vect_t *) vect_read(vec_in,i);
        vM1 = (vect_t *) vect_read(vec_in,i+1);

        zipWithV(vM0,vM1, vM0, f_sum_1);

        vR = (vect_t *) vect_read(vec_out, i/2);
        for(uint8_t j = 0; j< vM0->len; j+=2){
            char sum = *(char *)vect_read(vM0,j) + *(char *)vect_read(vM0,j+1);
            vect_write(vR, j/2, &sum);
        }
    }

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
        char * in1 = (char *) in_raw;

        wrapImageRGB_1(in1, &mRGB);

        mapMatrix_1(&mRGB, &mGRY, grayscale_div4_1);

        // sum columns and rows
        sumCols_sumRows_1(&mGRY, &mRSZ);

        mapMatrix_1(&mRSZ, &mASCII, convert_ascii_1);

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

    vect_init(&mRGB,height,sizeof(vect_t));
    for(int i = 0; i < height; i++){
        vect_init(&((vect_t *) mRGB.data)[i], width, sizeof(pixel_t));
    }    

    vect_init(&mGRY,height,sizeof(vect_t));
    for(int i = 0; i < height; i++){
        vect_init(&((vect_t *) mGRY.data)[i], width, sizeof(char));
    }

    vect_init(&mRSZ,height/2,sizeof(vect_t));
    for(int i = 0; i < height/2; i++){
        vect_init(&((vect_t *) mRSZ.data)[i], width/2, sizeof(char));
    }

    vect_init(&mASCII,height/2,sizeof(vect_t));
    for(int i = 0; i < height/2; i++){
        vect_init(&((vect_t *) mASCII.data)[i], width/2, sizeof(char));
    }

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

        // de-serialize the data
        multicore_fifo_push_blocking((uintptr_t)(char *)(uintptr_t) in);

        wrapImageRGB_0(in, &mRGB);

        mapMatrix_0(&mRGB, &mGRY, grayscale_div4);

        // sum columns and rows
        sumCols_sumRows_0(&mGRY, &mRSZ);

        mapMatrix_0(&mRSZ, &mASCII, convert_ascii);

        //acknowledge
        multicore_fifo_pop_blocking();

        tStop = time_us_32();
        printf("T diff=%.3f\n", tStop, (tStop-tStart)/1000.0f);

        /* PRINT RESULT */
        printf("Output:\n");
        for(uint16_t j = 0; j< mASCII.len; j++) {
            vect_handle_t v = (vect_t *) vect_read(&mASCII, j);
            for(uint16_t i = 0; i < (v->len); i++){
                output = *(char *)vect_read(v, i); 
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
