/*
 * memory.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/highmem.h>
#include <linux/gfp.h>
#include <linux/timex.h>
#include <linux/vmalloc.h>

#include <asm/uaccess.h>
#include <asm/string.h>

#define __PCMSIM_MEM_NO_EXTERN
#include "memory.h"
#include "util.h"
#include "config.h"

// there is no tsc.h interface on ARM: https://marc.info/?l=linux-arm-kernel&m=118970523409140&w=2
#ifdef __arm__

unsigned cpu_khz = PCMSIM_CPU_KHZ; //TODO: check if it's relevant !

#endif

/**
 * The threshold (the number of ticks), below which we can assume
 * no L2 misses in a result of time_read()
 */
unsigned memory_time_l2_threshold = 0;

/**
 * The average overhead of memory_read() per number of sectors
 * * Used in pcm.c
 */
unsigned memory_overhead_read[2 /* 0 = uncached, 1 = cached */]
			     [PCMSIM_MEM_SECTORS + 1];

/**
 * Memory bus speed
 */
unsigned memory_bus_mhz;

/**
 * Memory bus scaling factor
 */
unsigned memory_bus_scale;

/**
 * Logical memory row width (bytes per row-to-row advance)
 */
unsigned memory_row_width = 128;

/**
 * Flush the pipeline
 */
static void __inline__ flush(void)
{
#ifdef __arm__

	asm volatile("ISB");

#elif __i386__

	asm("pushl %eax\n\t"
	    "pushl %ebx\n\t"
	    "pushl %ecx\n\t"
	    "pushl %edx\n\t"
	    "xorl %eax, %eax\n\t"
	    "cpuid\n\t"
	    "popl %edx\n\t"
	    "popl %ecx\n\t"
	    "popl %ebx\n\t"
	    "popl %eax\n\t");

#elif __amd64__

	asm("pushq %rax\n\t"
	    "pushq %rbx\n\t"
	    "pushq %rcx\n\t"
	    "pushq %rdx\n\t"
	    "xorq %rax, %rax\n\t"
	    "cpuid\n\t"
	    "popq %rdx\n\t"
	    "popq %rcx\n\t"
	    "popq %rbx\n\t"
	    "popq %rax\n\t");

#endif
}

static void __inline__ write_back_flush_internal_caches(void)
{
#if defined(__i386__) || defined(__amd64__)
	asm volatile("wbinvd");
#elif __arm__
	asm volatile("Mov R0, #0");
	asm("MCR p15, 0, R0, c7, c14, 2");
#endif
}

static void __inline__ memory_barrier(void)
{
#if defined(__i386__) || defined(__amd64__)
	asm volatile("mfence");
#elif __arm__
	asm volatile("ISB");
#endif
}

/**
 * Measure the maximum time it takes to read a double-word
 * from each cache line of the given buffer. Return the number
 * of ticks, including the overhead of measurement.
 */
static unsigned noinline memory_time_read(const void *buffer, size_t size)
{
	unsigned t;

	//TODO: fix this -> mettre la meme boucle que dans copy et read

#ifdef __arm__

	unsigned d, f;
	char     tab[64] = { 0 }; // taille d'une ligne de cache
	//l'ARMv7 ne supporte pas le préchargement

	//initialisation du timer
	d = get_ticks();
	// Take the time measurement every 64 bytes (a typical length
	// of Pentium's L2 cache line)
	//boucle de lecture du tableau

	asm volatile(" mov R0, #0\n\t"
		     " mov R1, %0\n\t"
		     " boucle: CMP R0, #64\n\t"
		     " BEQ fin\n\t"
		     " ADD R0, R0, #1\n\t"
		     " LDRSB R2, [R1]\n\t"
		     " ADD R1, R1, #1\n\t"
		     " B boucle\n\t"
		     " fin:\n\t"
		     :
		     : "r"(&tab[0]));

	f = get_ticks();

#elif __i386__

	// Take the time measurement every 64 bytes (a typical length
	// of Pentium's L2 cache line)

	// Overhead of cpuid: 400 ticks on Pentium 4, 3 GHz

	asm("pushl %%eax\n\t"
	    "pushl %%esi\n\t"
	    "pushl %%ebx\n\t"
	    "pushl %%ecx\n\t"
	    "pushl %%edx\n\t"
	    "pushl %%ebp\n\t"

	    "cld\n\t"
	    "cli\n\t"

#ifdef PCMSIM_MEM_DISABLE_PREFETCH

	    // Disable prefetch

	    "pushl %%eax\n\t"
	    "pushl %%ecx\n\t"
	    "movl $0x1a0, %%ecx\n\t"
	    "rdmsr\n\t"
	    "movl $0x40200, %%ecx\n\t"
	    "notl %%ecx\n\t"
	    "andl %%ecx, %%eax\n\t"
	    "movl $0x1a0, %%ecx\n\t"
	    "wrmsr\n\t"
	    "popl %%ecx\n\t"
	    "popl %%eax\n\t"

#endif

	    // Initialize the timer

	    "xorl %%eax, %%eax\n\t"
	    "movl %%eax, %%ebp\n\t"
	    "lfence\n\t"
	    "rdtsc\n\t"
	    "pushl %%edx\n\t"
	    "pushl %%eax\n\t"

	    // Main loop

	    "l:\n\t"

	    "lodsl\n\t"
	    "addl $60, %%esi\n\t"

	    // Flush the pipeline

	    "lfence\n\t"

	    // Get the time

	    "rdtsc\n\t"
	    "movl (%%esp), %%ebx\n\t"
	    "movl -4(%%esp),  %%ecx\n\t"
	    "movl %%eax, (%%esp)\n\t"
	    "movl %%edx, -4(%%esp)\n\t"

	    // Keep the maximum interval (in the %ebp register)

	    "subl %%ebx, %%eax\n\t"
	    "sbbl %%ecx, %%edx\n\t"
	    "cmpl %%ebp, %%eax\n\t"
	    "cmovgl %%eax, %%ebp\n\t"

	    // Repeat as long as %edi > 0

	    "decl %%edi\n\t"
	    "jnz l\n\t"

	    "popl %%ebx\n\t"
	    "popl %%ebx\n\t"

#ifdef PCMSIM_MEM_DISABLE_PREFETCH

	    // Enable prefetch

	    "pushl %%eax\n\t"
	    "pushl %%ecx\n\t"
	    "movl $0x1a0, %%ecx\n\t"
	    "rdmsr\n\t"
	    "movl $0x40200, %%ecx\n\t"
	    "orl %%ecx, %%eax\n\t"
	    "movl $0x1a0, %%ecx\n\t"
	    "wrmsr\n\t"
	    "popl %%ecx\n\t"
	    "popl %%eax\n\t"

#endif

	    // Clean-up

	    "sti\n\t"

	    "movl %%ebp, %%edi\n\t"
	    "popl %%ebp\n\t"
	    "popl %%edx\n\t"
	    "popl %%ecx\n\t"
	    "popl %%ebx\n\t"
	    "popl %%esi\n\t"
	    "popl %%eax\n\t"

	    : "=D"(t)
	    : "S"(buffer), "D"(size / 64));

#elif __amd64__

	// Take the time measurement every 64 bytes (a typical length
	// of Pentium's L2 cache line)

	// What this code does is it loads 8bytes into rax every 64 bytes
	// then rbp stores the greatest rtsdc and is finally returned in rax

	// Overhead of cpuid: 200 ticks on Core 2 Duo, 2 GHz

	asm("pushq %%rdi\n\t"
	    "pushq %%rsi\n\t"
	    "pushq %%rbx\n\t"
	    "pushq %%rcx\n\t"
	    "pushq %%rdx\n\t"
	    "pushq %%rbp\n\t"

	    "cld\n\t"

	    // Initialize the timer

	    "xorq %%rax, %%rax\n\t"
	    "movq %%rax, %%rbp\n\t"
	    "lfence\n\t"
	    "rdtsc\n\t"
	    "pushq %%rdx\n\t"
	    "pushq %%rax\n\t"

	    // Main loop

	    "l:\n\t"

	    "lodsq\n\t"
	    "addq $56, %%rsi\n\t" // hypothesis: 56 'cause reg is 8 byte thus 64

	    // Flush the pipeline

	    "lfence\n\t"

	    // Get the time

	    "rdtsc\n\t"
	    "movq (%%rsp), %%rbx\n\t"
	    "movq -8(%%rsp),  %%rcx\n\t"
	    "movq %%rax, (%%rsp)\n\t"
	    "movq %%rdx, -8(%%rsp)\n\t"

	    // Keep the maximum interval (in the %rbp register)

	    "subq %%rbx, %%rax\n\t"
	    "sbbq %%rcx, %%rdx\n\t"
	    "cmpq %%rbp, %%rax\n\t"
	    "cmovgq %%rax, %%rbp\n\t"

	    // Repeat as long as %rdi > 0

	    "decq %%rdi\n\t"
	    "jnz l\n\t"

	    "popq %%rbx\n\t"
	    "popq %%rbx\n\t"

	    // Clean-up

	    "movq %%rbp, %%rax\n\t"
	    "popq %%rbp\n\t"
	    "popq %%rdx\n\t"
	    "popq %%rcx\n\t"
	    "popq %%rbx\n\t"
	    "popq %%rsi\n\t"
	    "popq %%rdi\n\t"

	    : "=a"(t)
	    : "S"(buffer), "D"(size / 64));

#endif

	flush();

#ifdef __arm__
	t = f - d;
#endif

	return t;
}

/**
 * Read the contents of a buffer
 */
void memory_read(const void *buffer, size_t size)
{
#ifdef __arm__

	// taken from /arch/arm/boot/compress/string.c of linux kernel 5.1.9
	int		       i = 0;
	unsigned char *	s = (unsigned char *)buffer;
	volatile unsigned char x0, x1, x2, x3, x4, x5, x6, x7;

	for (i = size >> 3; i > 0; i--) {
		x0 = *s++;
		x1 = *s++;
		x2 = *s++;
		x3 = *s++;
		x4 = *s++;
		x5 = *s++;
		x6 = *s++;
		x7 = *s++;
	}

	if (size & 1 << 2) {
		x0 = *s++;
		x1 = *s++;
		x2 = *s++;
		x3 = *s++;
	}

	if (size & 1 << 1) {
		x0 = *s++;
		x1 = *s++;
	}

	if (size & 1)
		x0 = *s++;

		//printk("size %d, adresse buffer %px", size, buffer);

		// The argument should be loaded like this anyway, but removing
		// them breaks the noinline
		// TODO: find why !
		//asm volatile("MOV r0, %0\n\t" : : "r"(buffer));
		//asm volatile("MOV r1, %0\n\t" : : "r"(size));
		/*asm volatile("WordRead: LDR r3, [r0], #4\n\t"
		     "SUBS r1, r1, #4\n\t"
		     "BGE WordRead");*/
		/*asm volatile("mov R2, #0\n\t"
		     " boucle_lec: CMP  R2 , R1\n\t"
		     " BEQ end_lec\n\t"
		     " LDRSB  R3, [R0]\n\t"
		     " ADD R0, R0, #1\n\t"
		     " ADD R2, R2, #1\n\t"
		     " B boucle_lec\n\t"
		     "end_lec:\n\t");*/

		//TODO: restore flush() after testing
		//flush();

#elif __i386__

	// taken from /arch/x86/boot/compress/string.c of linux kernel 5.1.9

	/*int d0, d1;
	asm volatile("rep ; lodsl\n\t"
		     "movl %4,%%ecx\n\t"
		     "rep ; lodsb\n\t"
		     : "=&c"(d0), "=&S"(d1)
		     : "0"(size >> 2), "g"(size & 3), "1"(buffer)
		     : "memory");*/

	asm("pushl %%eax\n\t"
	    "pushl %%esi\n\t"
	    "pushl %%ecx\n\t"

	    "cld\n\t"
	    "rep lodsl\n\t"
	    "lfence\n\t"

	    "popl %%ecx\n\t"
	    "popl %%esi\n\t"
	    "popl %%eax\n\t"

	    :
	    : "S"(buffer), "c"(size >> 2));

#elif __amd64__

	// same as mfence for lfence (see the comment in the function below)
	// taken from /arch/x86/boot/compress/string.c of linux kernel 5.1.9

	/*long d0, d1;
	asm volatile("rep ; lodsq\n\t"
		     "movq %4,%%rcx\n\t"
		     "rep ; lodsb\n\t"
		     : "=&c"(d0), "=&S"(d1)
		     : "0"(size >> 3), "g"(size & 7), "1"(buffer)
		     : "memory");*/

	asm("pushq %%rax\n\t"
	    "pushq %%rsi\n\t"
	    "pushq %%rcx\n\t"

	    "cld\n\t"
	    "rep lodsq\n\t"
	    "lfence\n\t"

	    "popq %%rcx\n\t"
	    "popq %%rsi\n\t"
	    "popq %%rax\n\t"

	    :
	    : "S"(buffer), "c"(size >> 3));

#endif
}

/**
 * Copy a memory buffer
 */
void memory_copy(void *dest, const void *buffer, size_t size)
{
#ifdef __arm__

	/*printk("size %d, adresse buffer %px, adresse dest %px\n", size, buffer,
	       dest); // https://lore.kernel.org/patchwork/patch/935610/ */
	// http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.faqs/ka13544.html

	memcpy(dest, buffer, size);
	//TODO: restore flush() after testing
	//flush();

#elif __i386__

	asm("pushl %%eax\n\t"
	    "pushl %%esi\n\t"
	    "pushl %%edi\n\t"
	    "pushl %%ecx\n\t"

	    "cld\n\t"
	    "rep movsl\n\t"
	    "mfence\n\t"

	    "popl %%ecx\n\t"
	    "popl %%edi\n\t"
	    "popl %%esi\n\t"
	    "popl %%eax\n\t"

	    :
	    : "S"(buffer), "D"(dest), "c"(size >> 2));

#elif __amd64__
	// mfence shouldn't be necessary because of Intel Memory Model
	// except when dealing with multi-threaded code, which is not the case here
	// https://preshing.com/20120515/memory-reordering-caught-in-the-act/

	// kernel memcpy is normally the same as the asm below
	// https://elixir.bootlin.com/linux/latest/source/arch/x86/boot/compressed/string.c#L28
	asm("pushq %%rax\n\t"
	    "pushq %%rsi\n\t"
	    "pushq %%rdi\n\t"
	    "pushq %%rcx\n\t"

	    "cld\n\t"
	    "rep movsq\n\t"
	    "mfence\n\t"

	    "popq %%rcx\n\t"
	    "popq %%rdi\n\t"
	    "popq %%rsi\n\t"
	    "popq %%rax\n\t"

	    :
	    : "S"(buffer), "D"(dest), "c"(size >> 3));
	//TODO: understand why ">> 3" in this case ! (dividing by 8)
	// could be because we are not copying byte by byte but rather
	// that we move 8 bytes by 8 bytes thus the size has to be divided
	// by 8, this makes sense for 32 bit code too ! as >> 2 is a division
	// by 4, 32 / 4 = 8 bits, I think it's the right explaination
	// "S" is special for si register
	// "D" is special for di register
	// "c" is special for c register
	// the three of them are used in rep movsq
	/*
        ctrl + f : x86 on that page:
        https://gcc.gnu.org/onlinedocs/gcc/Machine-Constraints.html#Machine-Constraints
        for rep movsq:
        https://c9x.me/x86/html/file_module_x86_id_279.html
        on ASM in GCC:
        https://gcc.gnu.org/onlinedocs/gcc/Extended-Asm.html
        */

#endif
}

/**
 * Calibrate the timer to determine whether there was an L2 cache miss or not
 */
void memory_calibrate(void)
{
	unsigned  count     = 0;
	unsigned  max_count = MMC_MAX_COUNT;
	void *    buffers[MMC_MAX_COUNT];
	void *    write_buffer;
	unsigned *dirty_buffer;

	unsigned no_misses       = 0;
	unsigned l2_misses       = 0;
	unsigned no_misses_total = 0;
	unsigned l2_misses_total = 0;

	unsigned data_no_misses[MMC_MAX_COUNT];
	unsigned data_l2_misses[MMC_MAX_COUNT];

	unsigned u, s, t, n;
	int      i, ok;

	// Initialize
	write_buffer = vmalloc(16 * PCMSIM_MEM_SECTORS * 1024);
	BUG_ON(write_buffer == NULL); // 131,072 Bytes or 131KB

	dirty_buffer = (unsigned *)vmalloc(sizeof(unsigned) * 4 * 1024 * 1024);
	BUG_ON(dirty_buffer == NULL); // 16MB

	for (u = 0; u < max_count; u++) {
		buffers[u] = vmalloc(16 * PCMSIM_MEM_SECTORS *
				     1024); //each 131,072 KB, so 13MB total
		if (buffers[u] != NULL)
			count++;
	}

	// Measure the latencies for cached and uncached reads
	for (u = 0; u < max_count; u++) {
		if (buffers[u] != NULL) {
			write_back_flush_internal_caches();
			s = memory_time_read(buffers[u], 4096);
			t = memory_time_read(buffers[u], 4096);

			if (s > 2 * 2000 || t > 2 * 2000) {
				count--;
				vfree(buffers[u]);
				buffers[u] = NULL;
				continue;
			}

			l2_misses += s;
			no_misses += t;
		}
	}

	WARN_ON(count == 0);
	if (count == 0)
		count++;

	printk("count = %d et l2_misses = %d\n", count, l2_misses);

	if (l2_misses <= no_misses + count * 4) { // TODO: Why ? how ?
		printk(KERN_WARNING
		       "Could not measure the memory access times\n");
		no_misses = l2_misses / 2; //TODO: Is this accurate
	}

	memory_time_l2_threshold =
		(no_misses + (l2_misses - no_misses) / 4) / count;

	/*
         * IMPORTANT
         * The loop below is the most important thing to us
         * in this function, as it gives us the memory timing we
         * need in order to add the PCM delay later.
         * 
         */

	//
	// Measure the total cost of memory_read()
	//
	for (i = 0; i <= PCMSIM_MEM_SECTORS; i++) {
		for (n = 1; n <= PCMSIM_MEM_SECTORS; n++) {
			l2_misses_total = 0;
			no_misses_total = 0;
			for (u = 0; u < max_count; u++) {
				if (buffers[u] != NULL) {
					write_back_flush_internal_caches();
					memory_barrier();

					s = _rdtsc();
					memory_read(buffers[u], n << 9);
					s = _rdtsc() - s;

					memory_barrier();

					t = _rdtsc();
					memory_read(buffers[u], n << 9);
					t = _rdtsc() - t;

					s = s <= overhead_get_ticks ?
						    0 :
						    s - overhead_get_ticks;
					t = t <= overhead_get_ticks ?
						    0 :
						    t - overhead_get_ticks;

					l2_misses_total += s;
					no_misses_total += t;

					data_l2_misses[u] = s;
					data_no_misses[u] = t;
				}
			}

			//memory_overhead_read INIT 1
			memory_overhead_read[PCMSIM_MEM_UNCACHED][n] =
				l2_misses_total / count;
			//memory_overhead_read INIT 2
			memory_overhead_read[PCMSIM_MEM_CACHED][n] =
				no_misses_total / count;
		}

		// Sanity check

		ok = 1;
		for (n = 1; n <= PCMSIM_MEM_SECTORS; n++) {
			if (memory_overhead_read[PCMSIM_MEM_UNCACHED][n] == 0)
				ok = 0;
			if (memory_overhead_read[PCMSIM_MEM_CACHED][n] == 0)
				ok = 0;
		}
		for (n = 2; n <= PCMSIM_MEM_SECTORS; n++) {
			if (memory_overhead_read[PCMSIM_MEM_UNCACHED][n - 1] >=
			    memory_overhead_read[PCMSIM_MEM_UNCACHED][n])
				ok = 0;
			if (memory_overhead_read[PCMSIM_MEM_CACHED][n - 1] >=
			    memory_overhead_read[PCMSIM_MEM_CACHED][n])
				ok = 0;
		}
		if (ok)
			break;
	}

	//
	// Cleanup
	//
	for (u = 0; u < max_count; u++) {
		if (buffers[u] != NULL)
			vfree(buffers[u]);
	}
	if (write_buffer != NULL)
		vfree(write_buffer);
	if (dirty_buffer != NULL)
		vfree(dirty_buffer);

	//
	// Print a report
	//
	printk("\n");
	printk("  PCMSIM Calibration Report  \n");
	printk("-----------------------------\n");
	printk("\n");
	printk("Num. of trials: %4d trials\n", count);
	printk("get_ticks     : %4d cycles\n", overhead_get_ticks);
	printk("\n");
	printk("Cached reads  : %4d cycles\n", no_misses / count);
	printk("Uncached reads: %4d cycles\n", l2_misses / count);
	printk("\n");
	printk("memory_time_l2_threshold : %4d cycles\n",
	       memory_time_l2_threshold);
	printk("\n");

	printk("Memory Access\n");
	printk("                 rU        rC\n");
	for (n = 1; n <= PCMSIM_MEM_SECTORS; n++) {
		printk("%4d sector%s %8d%8d\n", n, n == 1 ? " " : "s",
		       memory_overhead_read[PCMSIM_MEM_UNCACHED][n],
		       memory_overhead_read[PCMSIM_MEM_CACHED][n]);
	}
	printk("\n");
}