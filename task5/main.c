#include <stdio.h>
#include "bsp.h"
#include "images_alt.h"
#include "vector.h"
#include "skeleton_v2.h"
#include <pico/multicore.h>

#define PPM_WIDTH 32
#define PPM_HEIGHT 32

/******************MULTI CORE SUPPORT*************************/

typedef enum{
    WRAP_IMAGE=0,
    UNWRAP_IMAGE,
    MAP_MATRIX,
    SUMCOLS_SUMROWS
} function_type_t;

typedef void (*wrp_fnc)(char* in, vect_handle_t vec_out);
typedef void (*unwrp_fnc)(vect_handle_t vec_in, char* out);
typedef void (*mm_fnc)(vect_handle_t mat_in, vect_handle_t mat_out, void* (*f)(void*));
typedef void (*ss_fnc)(vect_handle_t vec_in, vect_handle_t vec_out);

static void core1_entry() {
    while (true) {
        function_type_t ft = multicore_fifo_pop_blocking();
        switch(ft){
            case WRAP_IMAGE:
                uintptr_t fn0_raw  = multicore_fifo_pop_blocking();
                uintptr_t arg_stream0 = multicore_fifo_pop_blocking();
                uintptr_t arg_vec0 = multicore_fifo_pop_blocking();

                wrp_fnc fn0 = (wrp_fnc)fn0_raw;
                char *stream_in  = (char *)arg_stream0;
                vect_handle_t vecOUT  = (vect_handle_t)arg_vec0;

                fn0(stream_in,vecOUT);
            break;
            case UNWRAP_IMAGE:
                uintptr_t fn1_raw  = multicore_fifo_pop_blocking();
                uintptr_t arg_vec1 = multicore_fifo_pop_blocking();
                uintptr_t arg_stream1 = multicore_fifo_pop_blocking();

                unwrp_fnc fn1 = (unwrp_fnc)fn1_raw;
                vect_handle_t vecIN  = (vect_handle_t)arg_vec1;
                char *stream_out  = (char *)arg_stream1;

                fn1(vecIN,stream_out);
            break;
            case MAP_MATRIX:
                uintptr_t fn2_raw  = multicore_fifo_pop_blocking();
                uintptr_t arg_vecI = multicore_fifo_pop_blocking();
                uintptr_t arg_vecO = multicore_fifo_pop_blocking();
                uintptr_t arg_ff = multicore_fifo_pop_blocking();

                mm_fnc fn2 = (mm_fnc)fn2_raw;
                vect_handle_t vecI  = (vect_handle_t)arg_vecI;
                vect_handle_t vecO  = (vect_handle_t)arg_vecO;
                void * ff = (void *) arg_ff;

                fn2(vecI,vecO,ff);
            break;
            case SUMCOLS_SUMROWS:
                uintptr_t fn3_raw  = multicore_fifo_pop_blocking();
                uintptr_t arg3_vecI = multicore_fifo_pop_blocking();
                uintptr_t arg3_vecO = multicore_fifo_pop_blocking();

                ss_fnc fn3 = (ss_fnc)fn3_raw;
                vect_handle_t vecI3  = (vect_handle_t)arg3_vecI;
                vect_handle_t vecO3  = (vect_handle_t)arg3_vecO;

                fn3(vecI3,vecO3);
            break;


            default:
                //ft = 99;
            break;
        }
        multicore_fifo_push_blocking(ft);
    }
}

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


void wrapImageGRY_0(char* in, vect_handle_t vec_out){
    vect_handle_t vT;

    for(uint8_t i = 0; i < (vec_out->len)/2; i++){
        //printf("h=%d | ", i);
        vT = (vect_t *) vect_read(vec_out, i);
        for(uint8_t j = 0; j < (vT->len); j++){
            uint16_t idx = i* (vT->len) + j;
            //printf("i=%d,%.1f\t",idx, in[i*w+j]);
            vect_write(vT, j, &in[idx]);
        }
        //printf("\n");
    }
}

void wrapImageGRY_1(char* in, vect_handle_t vec_out){
    vect_handle_t vT;

    for(uint8_t i = (vec_out->len)/2; i < (vec_out->len); i++){
        //printf("h=%d | ", i);
        vT = (vect_t *) vect_read(vec_out, i);
        for(uint8_t j = 0; j < (vT->len); j++){
            uint16_t idx = i* (vT->len) + j;
            //printf("i=%d,%.1f\t",idx, in[i*w+j]);
            vect_write(vT, j, &in[idx]);
        }
        //printf("\n");
    }
}

void unwrapImage(vect_handle_t vec_in, char* out){
    vect_handle_t vT;
    for(uint8_t i = 0; i < vec_in->len; i++){
        vT = (vect_t *) vect_read(vec_in,i);
        for(uint8_t j = 0; j < vT->len; j++){
            char val = *(char *) vect_read(vT,j);
            uint16_t idx = i* (vT->len) + j;

            out[idx] = val;
            //printf("%d\t", val);
        }
        //printf("\n");
    }

}

void unwrapImage_0(vect_handle_t vec_in, char* out){
    vect_handle_t vT;
    for(uint8_t i = 0; i < (vec_in->len)/2; i++){
        vT = (vect_t *) vect_read(vec_in,i);
        for(uint8_t j = 0; j < vT->len; j++){
            char val = *(char *) vect_read(vT,j);
            uint16_t idx = i* (vT->len) + j;

            out[idx] = val;
            //sleep_us(2);
            //memcpy(&out[idx], val, sizeof(char));
            //printf("%d\t", val);
        }
        //printf("\n");
    }

}

void unwrapImage_1(vect_handle_t vec_in, char* out){
    vect_handle_t vT;
    for(uint8_t i = (vec_in->len)/2; i < vec_in->len; i++){
        vT = (vect_t *) vect_read(vec_in,i);
        for(uint8_t j = 0; j < vT->len; j++){
            char val = *(char *) vect_read(vT,j);
            uint16_t idx = i* (vT->len) + j;

            out[idx] = val;
            //memcpy(&out[idx], val, sizeof(char));
            //printf("%d %d\t",idx, val);
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
        function_type_t z = WRAP_IMAGE;
        multicore_fifo_push_blocking(z);
        multicore_fifo_push_blocking((uintptr_t)wrapImageRGB_1);
        multicore_fifo_push_blocking((uintptr_t)(char *)(uintptr_t) in);
        multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mRGB);

        wrapImageRGB_0(in, &mRGB);

        //acknowledge
        multicore_fifo_pop_blocking();

        // gray process
        z = MAP_MATRIX;
        multicore_fifo_push_blocking(z);
        multicore_fifo_push_blocking((uintptr_t)mapMatrix_1);
        multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mRGB);
        multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mGRY);
        multicore_fifo_push_blocking((uintptr_t)(void *)(uintptr_t) grayscale_1);

        // mapMatrix operation with grayscale function on each element
        //vect_t grayImage = mapV(&image,f_gray); 
        //vect_t grayImage = mapMatrix(&image, grayscale); 
        mapMatrix_0(&mRGB, &mGRY, grayscale);

        //acknowledge
        multicore_fifo_pop_blocking();

        // div4 (resize process)
        z = MAP_MATRIX;
        multicore_fifo_push_blocking(z);
        multicore_fifo_push_blocking((uintptr_t)mapMatrix_1);
        multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mGRY);
        multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mGRY);
        multicore_fifo_push_blocking((uintptr_t)(void *)(uintptr_t) div_4_1);

        // mapMatrix operation with grayscale function on each element
        mapMatrix_0(&mGRY, &mGRY, div_4);

        //acknowledge
        multicore_fifo_pop_blocking();

        // reduce (resize process)
        z = SUMCOLS_SUMROWS;
        multicore_fifo_push_blocking(z);
        multicore_fifo_push_blocking((uintptr_t)sumCols_sumRows_1);
        multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mGRY);
        multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mRSZ);

        // sum columns and rows
        sumCols_sumRows_0(&mGRY, &mRSZ);

        //acknowledge
        multicore_fifo_pop_blocking();

        // convert to ASCII
        z = MAP_MATRIX;
        multicore_fifo_push_blocking(z);
        multicore_fifo_push_blocking((uintptr_t)mapMatrix_1);
        multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mRSZ);
        multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mASCII);
        multicore_fifo_push_blocking((uintptr_t)(void *)(uintptr_t) convert_ascii_1);

        // mapMatrix operation with grayscale function on each element
        //vect_t grayImage = mapV(&image,f_gray); 
        //vect_t grayImage = mapMatrix(&image, grayscale); 
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
