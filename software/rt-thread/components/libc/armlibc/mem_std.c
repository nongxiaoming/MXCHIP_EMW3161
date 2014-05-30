/*
 * File:    mem_std.c
 * Brief:   Replace memory management functions of arm standard c library
 *
 */

#include "rtthread.h"
#include "stdio.h"
/* avoid the heap and heap-using library functions supplied by arm */
#pragma import(__use_no_heap)
#pragma import(__use_no_semihosting_swi)

int fputc(int ch, FILE *f) {
  return ch;
}

int fgetc(FILE *f) {
  return 0;
}


int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}
void * malloc(int n)
{
    return rt_malloc(n);
}
void * calloc(int n,int c)
{
    return rt_calloc(n,c);
}
void * realloc(void *rmem, rt_size_t newsize)
{
    return rt_realloc(rmem, newsize);
}

void free(void *rmem)
{
    rt_free(rmem);
}
