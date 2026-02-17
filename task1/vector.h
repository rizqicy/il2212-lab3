#ifndef VECTOR_H_
#define VECTOR_H_

#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <string.h>


typedef struct pixel{
    char r;
    char g;
    char b;
} pixel_t;

typedef struct vect {
    void* data;
    size_t len;
    size_t type;
} vect_t;

typedef vect_t* vect_handle_t;

void vect_init(vect_handle_t vec, size_t len, size_t type);
void vect_write(vect_handle_t vec, size_t index, void* data);
void* vect_read(vect_handle_t vec, size_t index);
void vect_free(vect_handle_t vec);

#endif //VECTOR_H_
