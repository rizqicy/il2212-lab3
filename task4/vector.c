#include "vector.h"

void vect_init(vect_handle_t vec, size_t len, size_t type){
    //TODO add checker
    vec->len = len;
    vec->type = type;
    vec->data = malloc(len * type);

    //TODO add checker
}

void vect_write(vect_handle_t vec, size_t index, void* data){
    unsigned char* ptr = vec->data;
    memcpy(ptr + index * vec->type,data, vec->type);
}

void* vect_read(vect_handle_t vec, size_t index){
    //TODO add checker
    unsigned char* ptr = vec->data;
    return (void *) ptr + index * vec->type;
}

void vect_free(vect_handle_t vec){
    //TODO add checker
    free(vec);
}

