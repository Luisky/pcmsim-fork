/*
 * memory.h
 * PCM Simulator: Memory hierarchy code
 *
 * Copyright 2010
 *      The President and Fellows of Harvard College.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef __PCMSIM_MEMORY_H
#define __PCMSIM_MEMORY_H

#include "config.h"

/**
 * Memory cache codes
 */
#define PCMSIM_MEM_UNCACHED 0
#define PCMSIM_MEM_CACHED 1

// Max iterations
#define MMC_MAX_COUNT 100

#ifndef __PCMSIM_MEM_NO_EXTERN

/**
 * The average overhead of memory_read() per number of sectors
 */
extern unsigned memory_overhead_read[2][PCMSIM_MEM_SECTORS + 1];
/* 0 = uncached, 1 = cached */

#endif

/**
 * Calibrate the timer to determine whether there was an L2 cache miss or not
 */
void memory_calibrate(char *proc_buf, int *proc_buf_len);

/**
 * Determine whether the given buffer was present in its entirety
 * in the L2 cache before this function has been called. This function
 * loads the buffer to the cache as a part of its function, and in order
 * to function properly, it assumes that the buffer offset and the size
 * are aligned to a cache-line size.
 */
int memory_was_cached(const void *buffer, size_t size);

/**
 * noinline needed for ARM
 * Read the contents of a buffer
 */
void memory_read(const void *buffer, size_t size);

/**
 * noinline needed for ARM
 * Copy a memory buffer
 */
void memory_copy(void *dest, const void *buffer, size_t size);

#endif
