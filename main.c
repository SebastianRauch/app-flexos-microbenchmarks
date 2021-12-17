/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2021, Hugo Lefeuvre <hugo.lefeuvre@manchester.ac.uk>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <flexos/isolation.h>
#include <flexos/microbenchmarks/isolated.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

#include <uk/alloc.h>
#include <uk/sched.h>
#include <flexos/impl/main_annotation.h>

static inline int cmp_int(const void *x, const void *y) 
{
    uint64_t a = *((uint64_t*) x);
    uint64_t b = *((uint64_t*) y);
    if (a < b) {
        return -1;
    } else if (a > b) {
        return 1;
    } else {
        return 0;
    }
}

struct statistics {
	uint64_t num_measurements;
	uint64_t min;
	uint64_t max;
	uint64_t median;
	double average;
	double variance;
};

void do_statistics(uint64_t *measurements, uint64_t n, struct statistics *out_stats) {
	qsort(measurements, n, sizeof(uint64_t), &cmp_int);
	uint64_t min = (uint64_t) -1;
	uint64_t max = 0;
	uint64_t sum = 0;
	for (size_t i = 0; i < n; ++i) {
		size_t x = measurements[i];
		sum += x;
		if (x < min) {
			min = x;
		}
		if (x > max) {
			max = x;
		}
	}
	double avg = ((double) sum) / n;
	double var = 0;
	for (size_t i = 0; i < n; ++i) {
		size_t x = measurements[i];
		var += (x - avg) * (x - avg);
	}	
	var /= (n - 1);
	out_stats->num_measurements = n;
	out_stats->min = min;
	out_stats->max = max;
	if (n % 2) {
		out_stats->mean = measurements[n / 2];
	} else {
		out_stats->mean = (measurements[n / 2] + measurements[n / 2 - 1]) / 2;
	}
	out_stats->average = avg;
	out_stats->variance = var;
}

void print_stats(struct statistics *stats, const char *str) {
    int64_t a, b;
    uint64_t x, y;
    fraction_to_dec(stats->average, 2, &a, &x);
    fraction_to_dec(stats->variance, 2, &b, &y);
    uk_pr_info("%16s min=%4ld, max=%8ld, median=%4ld,\taverage=%4ld.%ld,\tvariance=%8ld.%ld\n",
        str, stats->min, stats->max, stats->mean, a, x, b, y); 
}

/* uk_print apparently can't print floating point numbers, so use this instead 
 * simply print the integers a.x */
void fraction_to_dec(double d, int digits, int64_t *out_a, uint64_t *out_x) {
    double frac = d - ((int64_t) d);
    if (frac < 0) {
	frac = -frac;
    }
    uint64_t res = 0;
    for (int i = 0; i < digits; ++i) {
        frac *= 10;
        res = (res * 10) + ((uint64_t) frac);
        frac -= (uint64_t) frac;
    }
    *out_a = (int64_t) d;
    *out_x = res;
}


// some config checks here...
#if CONFIG_LIBFLEXOS_INTELPKU && !CONFIG_LIBFLEXOS_GATE_INTELPKU_NO_INSTRUMENT
#error "Microbenchmarks should not be executed with gate instrumentation!"
#endif

// bench_start returns a timestamp for use to measure the start of a benchmark
// run.
__attribute__ ((always_inline)) static inline uint64_t bench_start(void)
{
  unsigned  cycles_low, cycles_high;
  asm volatile( "CPUID\n\t" // serialize
                "RDTSC\n\t" // read clock
                "MOV %%edx, %0\n\t"
                "MOV %%eax, %1\n\t"
                : "=r" (cycles_high), "=r" (cycles_low)
                :: "%rax", "%rbx", "%rcx", "%rdx" );
  return ((uint64_t) cycles_high << 32) | cycles_low;
}

// bench_end returns a timestamp for use to measure the end of a benchmark run.
__attribute__ ((always_inline)) static inline uint64_t bench_end(void)
{
  unsigned  cycles_low, cycles_high;
  asm volatile( "RDTSCP\n\t" // read clock + serialize
                "MOV %%edx, %0\n\t"
                "MOV %%eax, %1\n\t"
                "CPUID\n\t" // serialize -- but outside clock region!
                : "=r" (cycles_high), "=r" (cycles_low)
                :: "%rax", "%rbx", "%rcx", "%rdx" );
  return ((uint64_t) cycles_high << 32) | cycles_low;
}


// read TSC without any serialization
static inline __attribute__ ((always_inline)) uint64_t readtsc()
{
  unsigned  cycles_low, cycles_high;
	asm volatile(
//	"cpuid			\n"
	"RDTSC			\n"
        "MOV %%edx, %0		\n"
        "MOV %%eax, %1		\n"
        : "=r" (cycles_high), "=r" (cycles_low)
	:
	: "%rax", "%rbx", "%rcx", "%rdx"
	);
  	return ((uint64_t) cycles_high << 32) | cycles_low;
}

#define SERIALIZE_RDTSC 1

#if SERIALIZE_RDTSC
#define BENCH_START() bench_start()
#define BENCH_END() bench_end()
#else
#define BENCH_START() readtsc()
#define BENCH_END() readtsc()
#endif /* SERIALIZE_RDTSC */

__attribute__ ((noinline)) void fcall_0(void) {
    asm volatile ("");
}

__attribute__ ((noinline)) void fcall_0r(void) {
    asm volatile ("");
    return 42;
}

__attribute__ ((noinline)) void fcall_1(uint64_t arg1) {
    asm volatile ("");
}

__attribute__ ((noinline)) void fcall_1r(uint64_t arg1) {
    asm volatile ("");
    return arg1;
}

__attribute__ ((noinline)) void fcall_2(uint64_t arg1, uint64_t arg2) {
    asm volatile ("");
}

__attribute__ ((noinline)) void fcall_2r(uint64_t arg1, uint64_t arg2) {
    asm volatile ("");
    return arg2;
}

__attribute__ ((noinline)) void fcall_3(uint64_t arg1, uint64_t arg2, uint64_t arg3) {
    asm volatile ("");
}

__attribute__ ((noinline)) void fcall_3r(uint64_t arg1, uint64_t arg2, uint64_t arg3) {
    asm volatile ("");
    return arg3;
}

__attribute__ ((noinline)) void fcall_4(uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4) {
    asm volatile ("");
}

__attribute__ ((noinline)) void fcall_4r(uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4) {
    asm volatile ("");
    return arg4;
}

__attribute__ ((noinline)) void fcall_5(uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5) {
    asm volatile ("");
}

__attribute__ ((noinline)) void fcall_5r(uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5) {
    asm volatile ("");
    return arg5;
}

__attribute__ ((noinline)) void fcall_6(uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5, uint64_t arg6) {
    asm volatile ("");
}

__attribute__ ((noinline)) void fcall_6r(uint64_t arg1, uint64_t arg2, uint64_t arg3, uint64_t arg4, uint64_t arg5, uint64_t arg6) {
    asm volatile ("");
    return arg6;
}

/*
 * make sure function does not get inlined
 */
__attribute__ ((noinline))
void empty_fcall(void) {
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_1B(void) {
    char characters[1];
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_1Bs(void) {
    char characters[1] __attribute__((flexos_whitelist));
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_10B(void) {
    char characters[10];
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_10Bs(void) {
    char characters[10] __attribute__((flexos_whitelist));
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_100B(void) {
    char characters[100];
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_100Bs(void) {
    char characters[100] __attribute__((flexos_whitelist));
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_1000B(void) {
    char characters[1000];
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_1000Bs(void) {
    char characters[1000] __attribute__((flexos_whitelist));
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_1xB(void) {
    char characters[1];
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_1xBs(void) {
    char characters[1] __attribute__((flexos_whitelist));
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_2xB(void) {
    char characters1[1];
    char characters2[1];
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_2xBs(void) {
    char characters1[1] __attribute__((flexos_whitelist));
    char characters2[1] __attribute__((flexos_whitelist));
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_3xB(void) {
    char characters1[1];
    char characters2[1];
    char characters3[1];
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_3xBs(void) {
    char characters1[1] __attribute__((flexos_whitelist));
    char characters2[1] __attribute__((flexos_whitelist));
    char characters3[1] __attribute__((flexos_whitelist));
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_4xB(void) {
    char characters1[1];
    char characters2[1];
    char characters3[1];
    char characters4[1];
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_4xBs(void) {
    char characters1[1] __attribute__((flexos_whitelist));
    char characters2[1] __attribute__((flexos_whitelist));
    char characters3[1] __attribute__((flexos_whitelist));
    char characters4[1] __attribute__((flexos_whitelist));
    /* keep the call from being optimized away */
    asm volatile ("");
}

#define REPS			10000
#define WARMUP_REPS		100

#define SERIAL 0

static inline void RUN_ISOLATED_FCALL(void)
{
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_0);
}

static inline void RUN_FCALL(void)
{
	empty_fcall();
}

int main(int argc, char *argv[])
{
#if CONFIG_LIBFLEXOS_GATE_INTELPKU_PRIVATE_STACKS
    uk_pr_info("Measuring gate latencies with stack isolating gates...\n");
#elif CONFIG_LIBFLEXOS_GATE_INTELPKU_SHARED_STACKS
    uk_pr_info("Measuring gate latencies with shared stacks...\n");
#elif CONFIG_LIBFLEXOS_VMEPT
    uk_pr_info("Measuring gate latencies with VM/EPT RPC gates...\n");
#else
    uk_pr_info("Measuring gate latencies with *UNKNOWN* gates...\n");
#endif

    uint32_t overhead_tsc, overhead_gate, overhead_fcall, t0, t1;

#if SERIAL
    uk_pr_info("> serial\n");
    uk_pr_info("TSC\tgate\tfcall\n");
    for(int i = 0; i < REPS; i++) {
        t0 = bench_start();
        asm volatile("");
        t1 = bench_end();
        overhead_tsc = t1 - t0;

        t0 = bench_start();
	RUN_ISOLATED_FCALL();
        t1 = bench_end();
        overhead_gate = t1 - t0;

        t0 = bench_start();
	RUN_FCALL();
        t1 = bench_end();
        overhead_fcall = t1 - t0;

        uk_pr_info("%" PRId64 "\t%" PRId64 "\t%" PRId64 "\n", overhead_tsc,
					overhead_gate, overhead_fcall);
    }
#else
    uint64_t *results = uk_malloc(uk_alloc_get_default(), REPS * sizeof(uint64_t));
    uk_pr_info("> loop\n");
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        asm volatile("");
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    /* only the measurement itself */
    struct statistics stats_empty;
    do_statistics(results, REPS, &stats_empty);

    /* measurements for different local function calls */
    for (size_t i = 0; i < WARMUP_REPS; ++i) {
        fcall_0();
    }

    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        fcall_0();
	t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_fcall_0;
    do_statistics(results, REPS, &stats_fcall_0);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
        fcall_0();
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        fcall_0r();
	t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_fcall_0r;
    do_statistics(results, REPS, &stats_fcall_0r);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
        fcall_1(1);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        fcall_1(1);
	t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_fcall_1;
    do_statistics(results, REPS, &stats_fcall_1);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
        fcall_1r(1);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        fcall_1r(1);
	t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_fcall_1r;
    do_statistics(results, REPS, &stats_fcall_1r);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
        fcall_2(1, 2);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        fcall_2(1, 2);
	t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_fcall_2;
    do_statistics(results, REPS, &stats_fcall_2);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
        fcall_2r(1, 2);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        fcall_2r(1, 2);
	t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_fcall_2r;
    do_statistics(results, REPS, &stats_fcall_2r);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
        fcall_3(1, 2, 3);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        fcall_3(1, 2, 3);
	t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_fcall_3;
    do_statistics(results, REPS, &stats_fcall_3);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
        fcall_3r(1, 2, 3);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        fcall_3r(1, 2, 3);
	t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_fcall_3r;
    do_statistics(results, REPS, &stats_fcall_3r);
   
    for (size_t i = 0; i < WARMUP_REPS; ++i) {
        fcall_4(1, 2, 3, 4);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        fcall_4(1, 2, 3, 4);
	t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_fcall_4;
    do_statistics(results, REPS, &stats_fcall_4);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
        fcall_4r(1, 2, 3, 4);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        fcall_4r(1, 2, 3, 4);
	t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_fcall_4r;
    do_statistics(results, REPS, &stats_fcall_4r);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
        fcall_5(1, 2, 3, 4, 5);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        fcall_5(1, 2, 3, 4, 5);
	t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_fcall_5;
    do_statistics(results, REPS, &stats_fcall_5);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
        fcall_5r(1, 2, 3, 4, 5);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        fcall_5r(1, 2, 3, 4, 5);
	t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_fcall_5r;
    do_statistics(results, REPS, &stats_fcall_5r);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
        fcall_6(1, 2, 3, 4, 5, 6);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        fcall_6(1, 2, 3, 4, 5, 6);
	t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_fcall_6;
    do_statistics(results, REPS, &stats_fcall_6);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
        fcall_6r(1, 2, 3, 4, 5, 6);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
        fcall_6r(1, 2, 3, 4, 5, 6);
	t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_fcall_6r;
    do_statistics(results, REPS, &stats_fcall_6r);



   
    /* measurements for different remote function calls  */
    uint64_t retval;
    for (size_t i = 0; i < WARMUP_REPS; ++i) {
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_0);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_0);
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_remotecall_0;
    do_statistics(results, REPS, &stats_remotecall_0);


    for (size_t i = 0; i < WARMUP_REPS; ++i) {
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_0r);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_0r);
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_remotecall_0r;
    do_statistics(results, REPS, &stats_remotecall_0r);


    for (size_t i = 0; i < WARMUP_REPS; ++i) {
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_1, 1);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_1, 1);
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_remotecall_1;
    do_statistics(results, REPS, &stats_remotecall_1);


    for (size_t i = 0; i < WARMUP_REPS; ++i) {
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_1r, 1);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_1r, 1);
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_remotecall_1r;
    do_statistics(results, REPS, &stats_remotecall_1r);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_2, 1, 2);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_2, 1, 2);
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_remotecall_2;
    do_statistics(results, REPS, &stats_remotecall_2);


    for (size_t i = 0; i < WARMUP_REPS; ++i) {
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_2r, 1, 2);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_2r, 1, 2);
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_remotecall_2r;
    do_statistics(results, REPS, &stats_remotecall_2r);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_3, 1, 2, 3);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_3, 1, 2, 3);
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_remotecall_3;
    do_statistics(results, REPS, &stats_remotecall_3);


    for (size_t i = 0; i < WARMUP_REPS; ++i) {
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_3r, 1, 2, 3);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_3r, 1, 2, 3);
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_remotecall_3r;
    do_statistics(results, REPS, &stats_remotecall_3r);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_4, 1, 2, 3, 4);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_4, 1, 2, 3, 4);
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_remotecall_4;
    do_statistics(results, REPS, &stats_remotecall_4);


    for (size_t i = 0; i < WARMUP_REPS; ++i) {
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_4r, 1, 2, 3, 4);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_4r, 1, 2, 3, 4);
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_remotecall_4r;
    do_statistics(results, REPS, &stats_remotecall_4r);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_5, 1, 2, 3, 4, 5);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_5, 1, 2, 3, 4, 5);
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_remotecall_5;
    do_statistics(results, REPS, &stats_remotecall_5);


    for (size_t i = 0; i < WARMUP_REPS; ++i) {
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_5r, 1, 2, 3, 4, 5);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_5r, 1, 2, 3, 4, 5);
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_remotecall_5r;
    do_statistics(results, REPS, &stats_remotecall_5r);

    for (size_t i = 0; i < WARMUP_REPS; ++i) {
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_6, 1, 2, 3, 4, 5, 6);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_6, 1, 2, 3, 4, 5, 6);
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_remotecall_6;
    do_statistics(results, REPS, &stats_remotecall_6);


    for (size_t i = 0; i < WARMUP_REPS; ++i) {
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_6r, 1, 2, 3, 4, 5, 6);
    }
    for(int i = 0; i < REPS; i++) {
        t0 = BENCH_START();
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_6r, 1, 2, 3, 4, 5, 6);
        t1 = BENCH_END();
	results[i] = t1 - t0;
    }
    struct statistics stats_remotecall_6r;
    do_statistics(results, REPS, &stats_remotecall_6r);


    print_stats(&stats_empty,    "empty:");

    /* results for local calls */
    print_stats(&stats_fcall_0,  "fcall_0:");
    print_stats(&stats_fcall_0r, "fcall_0r:");

    print_stats(&stats_fcall_1,  "fcall_1:");
    print_stats(&stats_fcall_1r, "fcall_1r:");

    print_stats(&stats_fcall_2,  "fcall_2:");
    print_stats(&stats_fcall_2r, "fcall_2r:");

    print_stats(&stats_fcall_3,  "fcall_3:");
    print_stats(&stats_fcall_3r, "fcall_3r:");

    print_stats(&stats_fcall_4,  "fcall_4:");
    print_stats(&stats_fcall_4r, "fcall_4r:");

    print_stats(&stats_fcall_5,  "fcall_5:");
    print_stats(&stats_fcall_5r, "fcall_5r:");

    print_stats(&stats_fcall_6, "fcall_6:");
    print_stats(&stats_fcall_6r, "fcall_6r:");

    /* results for remote calls */
    print_stats(&stats_remotecall_0, "remotecall_0:");
    print_stats(&stats_remotecall_0r, "remotecall_0r:");

    print_stats(&stats_remotecall_1, "remotecall_1:");
    print_stats(&stats_remotecall_1r, "remotecall_1r:");

    print_stats(&stats_remotecall_2, "remotecall_2:");
    print_stats(&stats_remotecall_2r, "remotecall_2r:");

    print_stats(&stats_remotecall_3, "remotecall_3:");
    print_stats(&stats_remotecall_3r, "remotecall_3r:");

    print_stats(&stats_remotecall_4, "remotecall_4:");
    print_stats(&stats_remotecall_4r, "remotecall_4r:");

    print_stats(&stats_remotecall_5, "remotecall_5:");
    print_stats(&stats_remotecall_5r, "remotecall_5r:");

    print_stats(&stats_remotecall_6, "remotecall_6:");
    print_stats(&stats_remotecall_6r, "remotecall_6r:");

#endif

#if CONFIG_LIBFLEXOS_GATE_INTELPKU_PRIVATE_STACKS

    uk_pr_info("Now measuring data sharing latencies (no domain transitions)\n");
    uk_pr_info("> serial\n");

#if CONFIG_LIBFLEXOS_ENABLE_DSS
#define HEADER()					\
do {							\
    uk_pr_info("data shadow stack\n");			\
    uk_pr_info("TSC\tDSS\tstack\n");			\
} while(0)
#else
#define HEADER()					\
do {							\
    uk_pr_info("stack-to-heap converted\n");		\
    uk_pr_info("TSC\ts2h\tstack\n");			\
} while(0)
#endif

#define BENCH_SIZE(SIZE)				\
do {							\
    uk_pr_info(STRINGIFY(SIZE) "B stack v.s. "		\
		STRINGIFY(SIZE) "B ");			\
    HEADER();						\
							\
    for(int i = 0; i < REPS; i++) {			\
        t0 = bench_start();				\
        asm volatile("");				\
        t1 = bench_end();				\
        overhead_tsc = t1 - t0;				\
							\
        t0 = bench_start();				\
	empty_fcall_ ## SIZE ## Bs();			\
        t1 = bench_end();				\
        overhead_gate = t1 - t0;			\
							\
        t0 = bench_start();				\
	empty_fcall_ ## SIZE ## B();			\
        t1 = bench_end();				\
        overhead_fcall = t1 - t0;			\
							\
        uk_pr_info("%" PRId64 "\t%" PRId64 "\t%"		\
		   PRId64 "\n", overhead_tsc,		\
		   overhead_gate, overhead_fcall);	\
    }							\
} while(0)

#define BENCH_NB(NB)					\
do {							\
    uk_pr_info(STRINGIFY(NB) "x1B stack v.s. "		\
		STRINGIFY(NB) "x1B ");			\
    HEADER();						\
							\
    for(int i = 0; i < REPS; i++) {			\
        t0 = bench_start();				\
        asm volatile("");				\
        t1 = bench_end();				\
        overhead_tsc = t1 - t0;				\
							\
        t0 = bench_start();				\
	empty_fcall_ ## NB ## xBs();			\
        t1 = bench_end();				\
        overhead_gate = t1 - t0;			\
							\
        t0 = bench_start();				\
	empty_fcall_ ## NB ## xB();			\
        t1 = bench_end();				\
        overhead_fcall = t1 - t0;			\
							\
        uk_pr_info("%" PRId64 "\t%" PRId64 "\t%"		\
		   PRId64 "\n", overhead_tsc,		\
		   overhead_gate, overhead_fcall);	\
    }							\
} while(0)

    BENCH_SIZE(1);
    BENCH_SIZE(10);
    BENCH_SIZE(100);
    BENCH_SIZE(1000);

    BENCH_NB(1);
    BENCH_NB(2);
    BENCH_NB(3);
    BENCH_NB(4);

#else
    uk_pr_info("Not measuring data sharing latencies as we're configured with a shared stack.\n");
#endif /* CONFIG_LIBFLEXOS_GATE_INTELPKU_PRIVATE_STACKS */

    return 0;
}
