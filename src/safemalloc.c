/**
 * author: Nate Mathews
 * contact: njm3308@rit.edu
 * date: 2017-09-24
 **/

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include "safemalloc.h"

void * safemalloc(size_t size)
{
    void* ptr = malloc(size);
    if(ptr == NULL)
    {
        printf("Failed to allocate memory. Exiting application.");
        exit(EXIT_FAILURE);
    }
    else
    {
        return ptr;
    }
}

void * safecalloc(size_t num, size_t size)
{
    void* ptr = calloc(num, size);
    if(ptr == NULL)
    {
        printf("Failed to allocate memory. Exiting application.");
        exit(EXIT_FAILURE);
    }
    else
    {
        return ptr;
    }
}

void * saferealloc(void *ptr, size_t size)
{
    void* newptr = realloc(ptr, size);
    if(newptr == NULL)
    {
        printf("Failed to allocate memory. Exiting application.");
        exit(EXIT_FAILURE);
    }
    else
    {
        return newptr;
    }
}
