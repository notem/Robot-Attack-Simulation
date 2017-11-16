/**
 * author: Nate Mathews
 * contact: njm3308@rit.edu
 * date: 2017-09-24
 *
 * Malloc, calloc, and realloc wrappers
 **/

#include <stddef.h>

#ifndef SAFEMALLOC_H
#define SAFEMALLOC_H

/// malloc() which will never return null
void * safemalloc(size_t size);

/// calloc() which will never return null
void * safecalloc(size_t num, size_t size);

/// realloc() which will never return null
void * saferealloc(void *ptr, size_t size);

#endif //SAFEMALLOC_H
