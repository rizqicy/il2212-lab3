#include <stdio.h>
#include "bsp.h"
#include "images_alt.h"
#include "vector.h"
#include "skeleton_v2.h"
#include <pico/util/queue.h>
#include <pico/multicore.h>

#define PPM_WIDTH 32
#define PPM_HEIGHT 32

queue_t   s_in;
queue_t   s_1;
queue_t   s_2;
queue_t   s_out;

/******************MULTI CORE SUPPORT*************************/

typedef enum{
    WRAP_IMAGE=0,
    UNWRAP_IMAGE,
    MAP_MATRIX,
} function_type_t;

typedef void (*wrp_fnc)(char* in, vect_handle_t vec_out);
typedef void (*unwrp_fnc)(vect_handle_t vec_in, char* out);
typedef void (*mm_fnc)(vect_handle_t mat_in, vect_handle_t mat_out, void* (*f)(void*));

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

/* Definition of the channel */
//typedef cbuf_handle_t channel;

/* global variable matrix(s)*/
vect_t mRGB;
vect_t mGRY1;
vect_t mGRY2;
vect_t mRSZ1;
vect_t mRSZ2;
vect_t mASCII;


/* Definition of the functions 'readToken' and 'writeToken' */
void readToken(queue_t *ch, char* data) {
    queue_remove_blocking(ch, data);
    //xQueueReceive(ch, data, (TickType_t) portMAX_DELAY);
    //return circular_buf_get(ch, data);
}

void writeToken(queue_t *ch, char data) {
    char c = data;
    queue_add_blocking(ch, &c);
    //xQueueSend(ch, &c, (TickType_t) portMAX_DELAY);
    //circular_buf_put(ch, data);
}

void actor11SDF(uint16_t consum, uint16_t prod,
                queue_t* ch_in, queue_t* ch_out,
                void (*f) (char*, char*))
{
    char input[consum], output[prod];
    uint16_t i;

    for(i = 0; i < consum; i++) {
        readToken(ch_in, &input[i]);
    }
    f(input, output);
    for(i = 0; i< prod; i++) {	 
        writeToken(ch_out, output[i]);
    }
}


void wrapImageRGB(char* in, vect_handle_t vec_out){
    vect_handle_t vT;

    for(uint8_t i = 0; i < (vec_out->len); i++){
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

void wrapImageGRY(char* in, vect_handle_t vec_out){
    vect_handle_t vT;

    for(uint8_t i = 0; i < (vec_out->len); i++){
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
            out[i*(vT->len) + j] = val;
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
            out[i*(vT->len) + j] = val;
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
            out[i*(vT->len) + j] = val;
            //printf("%d\t", val);
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
static void f_grayscale(char* in, char* out) {
    // read data stream to matrix
    // Send task to core1
    function_type_t z = WRAP_IMAGE;
    multicore_fifo_push_blocking(z);
    multicore_fifo_push_blocking((uintptr_t)wrapImageRGB_1);
    multicore_fifo_push_blocking((uintptr_t)(char *)(uintptr_t) in);
    multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mRGB);

    wrapImageRGB_0(in, &mRGB);

    //acknowledge
    multicore_fifo_pop_blocking();

    // Send task to core1
    z = MAP_MATRIX;
    multicore_fifo_push_blocking(z);
    multicore_fifo_push_blocking((uintptr_t)mapMatrix_1);
    multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mRGB);
    multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mGRY1);
    multicore_fifo_push_blocking((uintptr_t)(void *)(uintptr_t) grayscale_1);

    // mapMatrix operation with grayscale function on each element
    //vect_t grayImage = mapV(&image,f_gray); 
    //vect_t grayImage = mapMatrix(&image, grayscale); 
    mapMatrix_0(&mRGB, &mGRY1, grayscale);

    //acknowledge
    multicore_fifo_pop_blocking();

    // Send task to core1
    // z = UNWRAP_IMAGE;
    // multicore_fifo_push_blocking(z);
    // multicore_fifo_push_blocking((uintptr_t)unwrapImage_1);
    // multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mGRY1);
    // multicore_fifo_push_blocking((uintptr_t)(char *)(uintptr_t) out);

    //write matrix to data stream
    unwrapImage(&mGRY1, out);

    //acknowledge
    // multicore_fifo_pop_blocking();
    //uint32_t w = multicore_fifo_pop_blocking();

    //printf("%d %d %d\n",x,y,w);
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
static void f_ascii(char* in, char* out) {

    // read data stream to matrix
    // Send task to core1
    function_type_t z = WRAP_IMAGE;
    multicore_fifo_push_blocking(z);
    multicore_fifo_push_blocking((uintptr_t)wrapImageGRY_1);
    multicore_fifo_push_blocking((uintptr_t)(char *)(uintptr_t) in);
    multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mRSZ2);

    wrapImageGRY_0(in, &mRSZ2);

    //acknowledge
    multicore_fifo_pop_blocking();

    // mapMatrix operation with convert_ascii function on each element
    //vect_t asciiImage = mapMatrix(&grayImage, convert_ascii); 

    // Send task to core1
    z = MAP_MATRIX;
    multicore_fifo_push_blocking(z);
    multicore_fifo_push_blocking((uintptr_t)mapMatrix_1);
    multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mRSZ2);
    multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mASCII);
    multicore_fifo_push_blocking((uintptr_t)(void *)(uintptr_t) convert_ascii_1);

    // mapMatrix operation with grayscale function on each element
    //vect_t grayImage = mapV(&image,f_gray); 
    //vect_t grayImage = mapMatrix(&image, grayscale); 
    mapMatrix_0(&mRSZ2, &mASCII, convert_ascii);

    //acknowledge
    multicore_fifo_pop_blocking();

    //write matrix to data stream
    unwrapImage(&mASCII, out);
}

/*
-- | the resize actor, which performs a interpolation for eavery image in the stream.
-- In this case from the defining function the dimensions get halved.
resizeSDF :: Int -- ^ dimension X for input images
          -> Int -- ^ dimension Y for input images
          -> Signal Double -- ^ stream of pixels to te interpolated
          -> Signal Double -- ^ stream of resulting interpolated pixels
resizeSDF dimX dimY = actor11SDF (dimX * dimY) (resizedDimX * resizedDimY) (wrapImageF dimX dimY resize)
  where
    resizedDimX = dimX `div` 2
    resizedDimY = dimY `div` 2

-- | Resizes an image using a technique based on interpolation, using
-- the transformation below:
--
-- <<resources/resize.png>>
resize :: Image Double -> Image Double
resize = mapMatrix (/ 4) . sumRows . sumCols
  where
    sumCols = mapV (mapV (reduceV (+)) . groupV 2)
    sumRows = mapV (reduceV (zipWithV (+))) . groupV 2
*/
static void f_resize(char* in, char* out){
    static char sum = 0;

    // Send task to core1
    function_type_t z = WRAP_IMAGE;
    multicore_fifo_push_blocking(z);
    multicore_fifo_push_blocking((uintptr_t)wrapImageGRY_1);
    multicore_fifo_push_blocking((uintptr_t)(char *)(uintptr_t) in);
    multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mGRY2);

    wrapImageGRY_0(in, &mGRY2);

    //acknowledge
    multicore_fifo_pop_blocking();

    // Send task to core1
    z = MAP_MATRIX;
    multicore_fifo_push_blocking(z);
    multicore_fifo_push_blocking((uintptr_t)mapMatrix_1);
    multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mGRY2);
    multicore_fifo_push_blocking((uintptr_t)(vect_handle_t)(uintptr_t) &mGRY2);
    multicore_fifo_push_blocking((uintptr_t)(void *)(uintptr_t) div_4_1);

    // mapMatrix operation with grayscale function on each element
    mapMatrix_0(&mGRY2, &mGRY2, div_4);

    //acknowledge
    multicore_fifo_pop_blocking();
    
    vect_handle_t vM0, vM1, vR;
    for(uint8_t i =0; i< mGRY2.len; i+=2){
        vM0 = (vect_t *) vect_read(&mGRY2,i);
        vM1 = (vect_t *) vect_read(&mGRY2,i+1);

        zipWithV(vM0,vM1, vM0, f_sum);

        vR = (vect_t *) vect_read(&mRSZ1, i/2);
        for(uint8_t j = 0; j< vM0->len; j+=2){
            sum = *(char *)vect_read(vM0,j) + *(char *)vect_read(vM0,j+1);
            vect_write(vR, j/2, &sum);
        }
    }

    //write matrix to data stream
    unwrapImage(&mRSZ1, out);
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

    vect_init(&mGRY1,height,sizeof(vect_t));
    for(int i = 0; i < height; i++){
        vect_init(&((vect_t *) mGRY1.data)[i], width, sizeof(char));
    }

    vect_init(&mGRY2,height,sizeof(vect_t));
    for(int i = 0; i < height; i++){
        vect_init(&((vect_t *) mGRY2.data)[i], width, sizeof(char));
    }

    vect_init(&mRSZ1,height/2,sizeof(vect_t));
    for(int i = 0; i < height/2; i++){
        vect_init(&((vect_t *) mRSZ1.data)[i], width/2, sizeof(char));
    }

    vect_init(&mRSZ2,height/2,sizeof(vect_t));
    for(int i = 0; i < height/2; i++){
        vect_init(&((vect_t *) mRSZ2.data)[i], width/2, sizeof(char));
    }

    vect_init(&mASCII,height/2,sizeof(vect_t));
    for(int i = 0; i < height/2; i++){
        vect_init(&((vect_t *) mASCII.data)[i], width/2, sizeof(char));
    }

    queue_init(&s_in, sizeof(char), 3*width*height);
    queue_init(&s_1, sizeof(char), width*height);
    queue_init(&s_2, sizeof(char), width*height/4);
    queue_init(&s_out, sizeof(char), width*height/4);

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
        
        tStart = time_us_32();

        /* Read input tokens */
        for(uint16_t j = 0; j < (3*w*h); j++) {
            writeToken(&s_in, sequence1[img_id][j+3]);
        }

        /* graySDF actor */
        actor11SDF(3*w*h, w*h, &s_in, &s_1, f_grayscale);
        /* resizeSDF actor*/
        actor11SDF(w*h, w*h/4, &s_1, &s_2, f_resize);
        /* asciiSDF actor */
        actor11SDF(w*h/4, w*h/4, &s_2, &s_out, f_ascii);

        tStop = time_us_32();
        printf("T diff=%.3f\n", tStop, (tStop-tStart)/1000.0f);

        /* Write output tokens */
        printf("Output:\n");
        for(uint16_t j = 0; j< (h/2); j++) {
            for(uint16_t i = 0; i < (w/2); i++){
                readToken(&s_out, &output);
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
