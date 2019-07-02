/*
 * pcm.c
 * PCM Simulator: PCM Model
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
#include <linux/slab.h>
#include <linux/timex.h>

#include <asm/uaccess.h>

#include "memory.h"
#include "pcm.h"
#include "ramdisk.h"
#include "util.h"

/**
 * The original PCM timings
 */
unsigned pcm_org_tRCD = PCMSIM_PCM_ORG_TRCD; //TODO: make this configurable
unsigned pcm_org_tRP  = PCMSIM_PCM_ORG_TRP; //TODO: make this configurable

/**
 * The original PCM frequency
 */
unsigned pcm_org_mhz = PCMSIM_PCM_ORG_MHZ; //TODO: make this configurable

/**
 * The extrapolated PCM timings
 */
unsigned pcm_tRCD;
unsigned pcm_tRP;

/**
 * The PCM latency for reading and writing
 */
unsigned pcm_latency[2 /* 0 = read, 1 = write */][PCMSIM_MEM_SECTORS + 1];

/**
 * The PCM delta latency for reading pcm_tRPand writing
 */
int pcm_latency_delta[2 /* 0 = read, 1 = write */][PCMSIM_MEM_SECTORS + 1];

/**
 * Calibrate the PCM model. This function can be called only after
 * the memory subsystem has been initialized.
 */
void pcm_calibrate(int pcmsim_pcm_lat_factor_write,
		   int pcmsim_pcm_lat_factor_read)
{
	unsigned sectors, n;
	unsigned mem_time, d_read, d_write;

	WARN_ON(sizeof(unsigned) != 4);

	// Compute the PCM latencies and the deltas
	// The deltas are useful, the PCM latencies just get printed
	for (sectors = 1; sectors <= PCMSIM_MEM_SECTORS; sectors++) {
		mem_time = memory_overhead_read[PCMSIM_MEM_UNCACHED][sectors];

		d_read  = mem_time * pcmsim_pcm_lat_factor_read;
		d_write = mem_time * pcmsim_pcm_lat_factor_write;

		printk("0 | d_read %d | d_write %d\n", d_read, d_write);

		if (d_read % 10 >= 5)
			d_read += 10;
		if (d_write % 10 >= 5)
			d_write += 10;
		d_read /= 10;
		d_write /= 10;

		printk("1 | d_read %d | d_write %d\n", d_read, d_write);

		pcm_latency_delta[PCM_READ][sectors]  = d_read;
		pcm_latency_delta[PCM_WRITE][sectors] = d_write;

		pcm_latency_delta[PCM_READ][sectors] =
			pcm_latency[PCM_READ][sectors] - mem_time;
		pcm_latency_delta[PCM_WRITE][sectors] =
			pcm_latency[PCM_WRITE][sectors] - mem_time;
	}

	// Print a report

	printk("\n");
	printk("  PCMSIM PCM Settings  \n");
	printk("-----------------------\n");
	printk("\n");
	printk("tRCD          : %4d bus cycles\n", pcm_tRCD);
	printk("tRP           : %4d bus cycles\n", pcm_tRP);
	printk("\n");
	printk("pcm\n");
	for (n = 1; n <= PCMSIM_MEM_SECTORS; n++) {
		printk("%4d sector%s  : %5d cycles read, %6d cycles write\n", n,
		       n == 1 ? " " : "s", pcm_latency[PCM_READ][n],
		       pcm_latency[PCM_WRITE][n]);
	}
	printk("\n");
	printk("pcm delta\n");
	for (n = 1; n <= PCMSIM_MEM_SECTORS; n++) {
		printk("%4d sector%s  : %5d cycles read, %6d cycles write\n", n,
		       n == 1 ? " " : "s", pcm_latency_delta[PCM_READ][n],
		       pcm_latency_delta[PCM_WRITE][n]);
	}
	printk("\n");
}

/**
 * Allocate PCM model data
 */
struct pcm_model *pcm_model_allocate(unsigned sectors)
{
	struct pcm_model *model;

	// Allocate the model struct

	model = (struct pcm_model *)kzalloc(sizeof(struct pcm_model),
					    GFP_KERNEL);
	if (model == NULL)
		goto out;

	// Allocate the dirty bits array

	model->dirty = (unsigned *)kzalloc(sectors / (sizeof(unsigned) << 3) +
						   sizeof(unsigned),
					   GFP_KERNEL);
	if (model->dirty == NULL)
		goto out_free;

	return model;

	// Cleanup on error

out_free:
	kfree(model);
out:
	return NULL;
}

/**
 * Free PCM model data
 */
void pcm_model_free(struct pcm_model *model)
{
	unsigned total_reads;
	unsigned total_writes;
	unsigned cached_reads;
	unsigned cached_writes;

	// Compute some statistics

	total_reads  = model->stat_reads_uncached + model->stat_reads_cached;
	total_writes = model->stat_writes_uncached + model->stat_writes_cached;

	cached_reads  = 0;
	cached_writes = 0;

	if (total_reads > 0) {
		cached_reads = (10000 * model->stat_reads_cached) / total_reads;
	}

	if (total_writes > 0) {
		cached_writes =
			(10000 * model->stat_writes_cached) / total_writes;
	}

	// Print the statistics

	printk("\n");
	printk("  PCMSIM Statistics  \n");
	printk("---------------------\n");
	printk("\n");
	printk("Reads         : %6d (%2d.%02d%% cached)\n", total_reads,
	       cached_reads / 100, cached_reads % 100);
	printk("Writes        : %6d (%2d.%02d%% cached)\n", total_writes,
	       cached_writes / 100, cached_writes % 100);
	printk("\n");

	// Free the data structures

	kfree(model->dirty);
	kfree(model);
}

/**
 * Perform a PCM read access
 */
void pcm_read(struct pcm_model *model, void *dest, const void *src,
	      size_t length, sector_t sector)
{
	unsigned T, before, after;
	unsigned sectors;
	unsigned t;

	sectors = length >> SECTOR_SHIFT;
	WARN_ON(sectors > PCMSIM_MEM_SECTORS);

	// Perform the operation
	before = _rdtsc();
	memory_copy(
		dest, src,
		length); // This does mfence, so we do not need pipeline flush
	after = _rdtsc();
	T     = after - before;

	//  Add latency to model
	model->budget += pcm_latency_delta[PCM_READ][sectors];
	// Speaks for itself TODO: change the 0 to a macro
	model->stat_reads_uncached++;

	/* Only useful if caching is handled
	// Clear the dirty bit
	model->dirty[sector >> (UNSIGNED_SHIFT + 3)] &= ~(1 << (sector & 0x1f));
        */

	// Stall
	t = _rdtsc();
	model->budget -= (int)(t - after);
	while (model->budget >= (int)overhead_get_ticks) {
		T = _rdtsc();
		model->budget -= (int)(T - t);
		t = T;
	}
}

/**
 * Perform a PCM write access
 */
void pcm_write(struct pcm_model *model, void *dest, const void *src,
	       size_t length, sector_t sector)
{
	unsigned T, before, after;
	unsigned sectors;
	unsigned t;
	int      cached;

	sectors = length >> SECTOR_SHIFT;
	WARN_ON(sectors > PCMSIM_MEM_SECTORS);

	// Perform the operation
	before = _rdtsc();
	memory_copy(
		dest, src,
		length); // This does mfence, so we do not need pipeline flush
	after = _rdtsc();
	T     = after - before;

	// Add latency to model
	model->budget += pcm_latency_delta[PCM_WRITE][sectors];
	// Speaks for itself TODO: change the 0 to a macro
	model->stat_writes_uncached++;

	/* Only useful if caching is handled
        // Get the dirty bit
	model->dirty = (model->dirty[sector >> (UNSIGNED_SHIFT + 3)] &
			(1 << (sector & 0x1f))) != 0;
	// Set the dirty bit
	model->dirty[sector >> (UNSIGNED_SHIFT + 3)] |= 1 << (sector & 0x1f);
        */

	// Stall
	t = _rdtsc(); // get_ticks ?
	model->budget -= (int)(t - after);
	while (model->budget >= (int)overhead_get_ticks) {
		T = _rdtsc();
		model->budget -= (int)(T - t);
		t = T;
	}
}
