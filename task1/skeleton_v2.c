#include "skeleton_v2.h" /* additional data type */

/////////////////
void mapV(vect_handle_t vec_in, vect_handle_t vec_out, void* (*f)(void*)){
    
    size_t len = (vec_in->len < vec_out->len) ? vec_in->len : vec_out->len;

    for(int i=0; i < len; i++){
        vect_write(vec_out, i, f(vect_read(vec_in,i)));
    }
}

void mapMatrix(vect_handle_t mat_in, vect_handle_t mat_out, void* (*f)(void*)){
    vect_handle_t temp;

    for(int i=0; i < (mat_in->len); i++){
        temp = (vect_t *)vect_read(mat_in,i);
        for(int j=0; j < (*temp).len; j++){
            //pixel_t a = *(pixel_t *) vect_read(&temp,j);
            //int sum = a.r + a.g + a.b;
            //printf("j=%d,%d,%d,%d,%d\t",j,a.r,a.g,a.b, sum);
            vect_write(&((vect_t *) mat_out->data)[i],j, f(vect_read(temp,j)));
            //vect_write(&vec_out, i, f(vect_read(vec_in,i)));
        }
   }
}

void* f_mapV_example(void* pin){
    int *in = (int *)pin;
    static int tmp = 0;     //temporary memory

    tmp = *in * 5;

    return &tmp;
}


///////////////////
void zipWithV(vect_handle_t vecA, vect_handle_t vecB, vect_handle_t vec_out, void* (*f)(void*,void*)){
    int len;

    if(vecA->len < vecB->len){
        len = vecA->len;
    }
    else{
        len = vecB->len;
    }

    for(int i=0; i < len; i++){
        vect_write(vec_out, i, f(vect_read(vecA,i),vect_read(vecB,i)));
    }
}

void* f_zipWithV_example(void* pin1,void* pin2){
    int *in1 = (int *)pin1;
    int *in2 = (int *)pin2;
    static int tmp = 0;     //temporary memory

    tmp = *in1 + *in2;
    return &tmp;
}


///////////////////
void reduceV(vect_handle_t vec_in, void* out, void* (*f)(void*, void*)){
    int i;
    void* ptr;

    if(vec_in->len < 1){
        ptr = vect_read(vec_in,0);
    }
    else{
        ptr = vect_read(vec_in,0);
        for(i=1; i < (vec_in->len); i++){
            ptr = f(ptr,vect_read(vec_in,i));
            //printf("out=%d,i=%d, p=%p\n",*(int *)ptr,i,ptr);
        }
    }
    memcpy(out,ptr,vec_in->type);
}

void* f_reduceV_example(void* pin1, void* pin2){
    int *in1 = (int *)pin1;
    int *in2 = (int *)pin2;
    static int tmp = 0;     //temporary memory

    tmp = *in1 + *in2;
    return &tmp;
}



/////////////////////////////
vect_t groupV(vect_handle_t vec_in, int num){
    int groups;
    vect_t vec_out;

    groups = vec_in->len / num;

    vect_init(&vec_out,groups,sizeof(vect_t));

    for(int i = 0; i < groups; i++){
        vect_init(&((vect_t *) vec_out.data)[i],num,vec_in->type);
        for(int j = 0; j < num; j++){
            vect_write(&((vect_t *) vec_out.data)[i],j,vect_read(vec_in,i*num+j));
        }
    }

    return vec_out;
}



