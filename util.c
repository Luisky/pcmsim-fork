/*
 * util.c
 * PCM Simulator: Miscellaneous utilities
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/highmem.h>
#include <linux/gfp.h>
#include <linux/timex.h>

#include <asm/uaccess.h>

#define __PCMSIM_UTIL_NO_EXTERN
#include "config.h"
#include "util.h"

/**
 * The overhead of _rdtsc()
 */
unsigned overhead_rdtsc = 0;

/**
 * Return the current value of the processor's tick counter, but do not flush the pipeline
 */
u64 _rdtsc(void)
{
#if defined(__i386__) || defined(__amd64__)

	unsigned a, d;
	asm volatile("rdtsc" : "=a"(a), "=d"(d));
	return (((u64)a) | (((u64)d) << 32));

#elif __arm__

	unsigned long int a;
	asm volatile("mrc p15, 0, %0, c9, c13, 0" : "=r"(a));
	return (u64)a * 64;

#endif
}

/**
 * Calibrate the timers in utilities
 */
void util_calibrate(void)
{
	unsigned max_count = 128;
	unsigned u, s, t;

	// Measure the overhead of _rdtsc()

	t = 0;
	for (u = 0; u < max_count; u++) {
		s = _rdtsc();
		t += _rdtsc() - s;
	}

	overhead_rdtsc = t / max_count;
}
