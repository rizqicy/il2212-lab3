#ifndef SKELETON_H_
#define SKELETON_H_

#include "vector.h"


void mapMatrix(vect_handle_t mat_in, vect_handle_t mat_out, void* (*f)(void*));
void mapV(vect_handle_t vec_in, vect_handle_t vec_out, void* (*f)(void*));
void zipWithV(vect_handle_t vecA, vect_handle_t vecB, vect_handle_t vec_out, void* (*f)(void*,void*));
void reduceV(vect_handle_t vec_in, void* out, void* (*f)(void*, void*));
vect_t groupV(vect_handle_t vec_in, int num);


#endif
