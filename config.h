/*
 * pcm.h
 * PCM Simulator: Configuration
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

/**
 * this file contains the constants needed to initialize the module
 * especially information about memory (DRAM)
 * 
 */

#ifndef __PCMSIM_CONFIG_H
#define __PCMSIM_CONFIG_H

// Used in memory.c
// CPU frequency
#define PCMSIM_CPU_KHZ 300000
// DDR version
#define PCMSIM_DDR_VER 3
// DDR rating
#define PCMSIM_DDR_RATING 1866 // 600
// Logical memory row width (bytes per row-to-row advance)
#define PCMSIM_DDR_ROW_WIDTH 128
// Memory timing informations
#define PCMSIM_DDR_TRCD 15
#define PCMSIM_DDR_TRP 15
#define PCMSIM_DDR_TCLx10 25

/**
 * Maximum number of sectors the memory system considers
 */
#define PCMSIM_MEM_SECTORS 8 //TODO: search what sector means in this context

/**
 * Uncomment to disable prefetch during cache tests
 */
//#define PCMSIM_MEM_DISABLE_PREFETCH

/**
 * Uncomment to turn the device into a pure ramdisk
 */
#define PCMSIM_RAMDISK_ONLY

/**
 * Uncomment to get the ground truth cache measurements (slow)
 */
//#define PCMSIM_GROUND_TRUTH

/**
 * Uncomment to ignore L2 cache effects
 */
//#define PCMSIM_IGNORE_L2

/**
 * Uncomment to check the accuracy of   ()
 */
#define PCMSIM_CHECK_ACCURACY

#endif
