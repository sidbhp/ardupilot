/*
 * Copyright (C) Siddharth Bharat Purohit 2017
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  wrappers for allocation functions

  Relies on linker wrap options

  Note that not all functions that have been wrapped are implemented
  here. The others are wrapped to ensure the function is not used
  without an implementation. If we need them then we can implement as
  needed.
 */

#include <stdio.h>
#include <string.h>
#include <hal.h>
#include <chheap.h>
#include <stdarg.h>

#define MIN_ALIGNMENT 8

#if defined(CCM_RAM_SIZE)
#ifndef CCM_BASE_ADDRESS
#define CCM_BASE_ADDRESS 0x10000000
#endif
static memory_heap_t ccm_heap;
static bool ccm_heap_initialised = false;
#endif

size_t get_thd_free_stack(void *wsp, size_t size)
{
  size_t n = 0;
#if CH_DBG_FILL_THREADS
  uint8_t *startp = (uint8_t *)wsp + sizeof(thread_t);
  uint8_t *endp = (uint8_t *)wsp + size;
  while (startp < endp)
    if(*startp++ == CH_DBG_STACK_FILL_VALUE) ++n;
#endif
  return n;
}

void *malloc_ccm(size_t size)
{
    void *p = NULL;
#if defined(CCM_RAM_SIZE)
    if (!ccm_heap_initialised) {
        ccm_heap_initialised = true;
        chHeapObjectInit(&ccm_heap, (void *)CCM_BASE_ADDRESS, CCM_RAM_SIZE*1024);
    }
    p = chHeapAllocAligned(&ccm_heap, size, CH_HEAP_ALIGNMENT);
    if (p != NULL) {
        memset(p, 0, size);
    }
#else
    (void)size;
#endif
    return p;
}

void *malloc(size_t size)
{
    if (size == 0) {
        return NULL;
    }
    void *p = chHeapAllocAligned(NULL, size, MIN_ALIGNMENT);
    if (p) {
        memset(p, 0, size);
    } else {
        // fall back to CCM memory when main memory full
        p = malloc_ccm(size);
    }
    return p;
}

void *malloc_ccm_aligned(size_t size, size_t align)
{
    void *p = NULL;
#if defined(CCM_RAM_SIZE)
    if (!ccm_heap_initialised) {
        ccm_heap_initialised = true;
        chHeapObjectInit(&ccm_heap, (void *)CCM_BASE_ADDRESS, CCM_RAM_SIZE*1024);
    }
    p = chHeapAllocAligned(&ccm_heap, size, align);
    if (p != NULL) {
        memset(p, 0, size);
    }
#else
    (void)size;
    (void)align;
#endif
    return p;
}

void *malloc_aligned(size_t size, size_t align)
{
    if (size == 0) {
        return NULL;
    }
    void *p = chHeapAllocAligned(NULL, size, align);
    if (p) {
        memset(p, 0, size);
    } else {
        // fall back to CCM memory when main memory full
        p = malloc_ccm_aligned(size, align);
    }
    return p;
}

void *calloc_aligned(size_t nmemb, size_t size, size_t align)
{
    return malloc_aligned(nmemb * size, align);
}

void *calloc(size_t nmemb, size_t size)
{
    return malloc(nmemb * size);
}

void free(void *ptr)
{
    if(ptr != NULL) {
        chHeapFree(ptr);
    }
}

/*
  return total available memory in bytes
 */
size_t mem_available(void)
{
    size_t totalp = 0;
    // get memory available on main heap
    chHeapStatus(NULL, &totalp, NULL);

    // we also need to add in memory that is not yet allocated to the heap
    totalp += chCoreGetStatusX();

#if defined(CCM_RAM_SIZE)
    size_t ccm_available = 0;
    chHeapStatus(&ccm_heap, &ccm_available, NULL);
    totalp += ccm_available;
#endif
    
    return totalp;
}
