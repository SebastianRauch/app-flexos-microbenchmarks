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

#if CONFIG_LIBFLEXOS_VMEPT
#include <flexos/impl/main_annotation.h>
#endif

// to easily change print method
#define PRINT printf

/* if this is set store results in an array to be able to compute the median
 * otherwise only compute the average */
#define COMPUTE_MEDIAN 1

#if COMPUTE_MEDIAN
	#define BEGIN_MICROBENCHMARKS() \
		uint64_t retval, t0, t1;	\
		uint64_t *results = uk_malloc(uk_alloc_get_default(), REPS * sizeof(uint64_t));
#else
	#define BEGIN_MICROBENCHMARKS() \
		uint64_t min, max, sum, t0, t1, diff, retval;
#endif

#if COMPUTE_MEDIAN
	#define FINALIZE_MICROBENCHMARKS() \
		uk_free(uk_alloc_get_default(), results);
#else
	#define FINALIZE_MICROBENCHMARKS()
#endif

#if COMPUTE_MEDIAN
	#define BENCHMARK(stmt, warmup_runs, measurement_runs, out_stats_ptr) 		\
		for(int i = 0; i < warmup_runs; i++) {									\
			stmt;																\
		}																		\
		for(int i = 0; i < measurement_runs; i++) {								\
			t0 = BENCH_START();													\
			stmt;																\
			t1 = BENCH_END();													\
			results[i] = t1 - t0;												\
		}																		\
		do_statistics(results, warmup_runs, out_stats_ptr);
#else
	#define BENCHMARK(stmt, warmup_runs, measurement_runs, out_stats_ptr)		\
		min = (uint64_t) -1;													\
		max = 0;																\
		sum = 0;																\
		for(int i = 0; i < warmup_runs; i++) {									\
			stmt;																\
		}																		\
		for(int i = 0; i < measurement_runs; i++) {								\
			t0 = BENCH_START();													\
			stmt;																\
			t1 = BENCH_END();													\
			diff = t1 - t0;														\
			if (diff < min) min = diff;											\
			if (diff > max) max = diff;											\
			sum += diff;														\
		}																		\
		(out_stats_ptr)->num_measurements = measurement_runs;					\
		(out_stats_ptr)->min = min;												\
		(out_stats_ptr)->max = max;												\
		(out_stats_ptr)->average = sum / (measurement_runs);
#endif

#define GATECALL_0 \
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_0);

#define GATECALL_0R \
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_0r);

#define GATECALL_1 \
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_1, 1);

#define GATECALL_1R \
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_1r, 1);

#define GATECALL_2 \
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_2, 1, 2);

#define GATECALL_2R \
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_2r, 1, 2);

#define GATECALL_3 \
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_3, 1, 2, 3);

#define GATECALL_3R \
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_3r, 1, 2, 3);

#define GATECALL_4 \
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_4, 1, 2, 3, 4);

#define GATECALL_4R \
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_4r, 1, 2, 3, 4);

#define GATECALL_5 \
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_5, 1, 2, 3, 4, 5);

#define GATECALL_5R \
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_5r, 1, 2, 3, 4, 5);

#define GATECALL_6 \
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_fcall_6, 1, 2, 3, 4, 5, 6);

#define GATECALL_6R \
	flexos_gate_r(libflexosmicrobenchmarks, retval, flexos_microbenchmarks_fcall_6r, 1, 2, 3, 4, 5, 6);

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
#if COMPUTE_MEDIAN
	uint64_t median;
	double average;
	double sdev;
#else
	uint64_t average;
#endif
};

#if COMPUTE_MEDIAN
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
		out_stats->median = measurements[n / 2];
	} else {
		out_stats->median = (measurements[n / 2] + measurements[n / 2 - 1]) / 2;
	}
	out_stats->average = avg;
	out_stats->sdev = sqrt(var);
}
#endif

void print_stats(struct statistics *stats, const char *str) {

#if COMPUTE_MEDIAN
    int64_t a, b;
    uint64_t x, y;
    fraction_to_dec(stats->average, 2, &a, &x);
    fraction_to_dec(stats->sdev, 2, &b, &y);
	// fomat: description min max median average sdev
	PRINT("%16s %4ld \t %8ld \t %4ld \t %4ld.%ld \t %8ld.%ld\n",
        str, stats->min, stats->max, stats->median, a, x, b, y);
#else
	// fomat: description min max average
	PRINT("%16s %4ld \t %8ld \t %4ld\n",
        str, stats->min, stats->max, stats->average);
#endif
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

	BEGIN_MICROBENCHMARKS()

#if CONFIG_LIBFLEXOS_GATE_INTELPKU_PRIVATE_STACKS
    PRINT("Measuring gate latencies with stack isolating gates...\n");
#elif CONFIG_LIBFLEXOS_GATE_INTELPKU_SHARED_STACKS
    PRINT("Measuring gate latencies with shared stacks...\n");
#elif CONFIG_LIBFLEXOS_VMEPT
    PRINT("Measuring gate latencies with VM/EPT RPC gates...\n");
#else
    PRINT("Measuring gate latencies with *UNKNOWN* gates...\n");
#endif


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

    //uk_pr_info("> loop\n");
    /* only the measurement itself */
	struct statistics rdtsc_overhead;
	BENCHMARK(asm volatile(""), WARMUP_REPS, REPS, &rdtsc_overhead)

    /* measurements for different local function calls */
	struct statistics stats_fcall_0;
	BENCHMARK(fcall_0(), WARMUP_REPS, REPS, &stats_fcall_0)

	struct statistics stats_fcall_0r;
	BENCHMARK(fcall_0r(), WARMUP_REPS, REPS, &stats_fcall_0r)

	struct statistics stats_fcall_1;
	BENCHMARK(fcall_1(1), WARMUP_REPS, REPS, &stats_fcall_1)

	struct statistics stats_fcall_1r;
	BENCHMARK(fcall_1r(1), WARMUP_REPS, REPS, &stats_fcall_1r)

	struct statistics stats_fcall_2;
	BENCHMARK(fcall_2(1, 2), WARMUP_REPS, REPS, &stats_fcall_2)

	struct statistics stats_fcall_2r;
	BENCHMARK(fcall_2r(1, 2), WARMUP_REPS, REPS, &stats_fcall_2r)

	struct statistics stats_fcall_3;
	BENCHMARK(fcall_3(1, 2, 3), WARMUP_REPS, REPS, &stats_fcall_3)

	struct statistics stats_fcall_3r;
	BENCHMARK(fcall_3r(1, 2, 3), WARMUP_REPS, REPS, &stats_fcall_3r)

	struct statistics stats_fcall_4;
	BENCHMARK(fcall_4(1, 2, 3, 4), WARMUP_REPS, REPS, &stats_fcall_4)

	struct statistics stats_fcall_4r;
	BENCHMARK(fcall_4r(1, 2, 3, 4), WARMUP_REPS, REPS, &stats_fcall_4r)

	struct statistics stats_fcall_5;
	BENCHMARK(fcall_5(1, 2, 3, 4, 5), WARMUP_REPS, REPS, &stats_fcall_5)

	struct statistics stats_fcall_5r;
	BENCHMARK(fcall_5r(1, 2, 3, 4, 5), WARMUP_REPS, REPS, &stats_fcall_5r)

	struct statistics stats_fcall_6;
	BENCHMARK(fcall_6(1, 2, 3, 4, 5, 6), WARMUP_REPS, REPS, &stats_fcall_6)

	struct statistics stats_fcall_6r;
	BENCHMARK(fcall_6r(1, 2, 3, 4, 5, 6), WARMUP_REPS, REPS, &stats_fcall_6r)

	/* measurements for different remote function calls  */
	struct statistics stats_remotecall_0;
	BENCHMARK(GATECALL_0, WARMUP_REPS, REPS, &stats_remotecall_0)

	struct statistics stats_remotecall_0r;
	BENCHMARK(GATECALL_0R, WARMUP_REPS, REPS, &stats_remotecall_0r)

	struct statistics stats_remotecall_1;
	BENCHMARK(GATECALL_1, WARMUP_REPS, REPS, &stats_remotecall_1)

	struct statistics stats_remotecall_1r;
	BENCHMARK(GATECALL_1R, WARMUP_REPS, REPS, &stats_remotecall_1r)

	struct statistics stats_remotecall_2;
	BENCHMARK(GATECALL_2, WARMUP_REPS, REPS, &stats_remotecall_2)

	struct statistics stats_remotecall_2r;
	BENCHMARK(GATECALL_2R, WARMUP_REPS, REPS, &stats_remotecall_2r)

	struct statistics stats_remotecall_3;
	BENCHMARK(GATECALL_3, WARMUP_REPS, REPS, &stats_remotecall_3)

	struct statistics stats_remotecall_3r;
	BENCHMARK(GATECALL_3R, WARMUP_REPS, REPS, &stats_remotecall_3r)

	struct statistics stats_remotecall_4;
	BENCHMARK(GATECALL_4, WARMUP_REPS, REPS, &stats_remotecall_4)

	struct statistics stats_remotecall_4r;
	BENCHMARK(GATECALL_4R, WARMUP_REPS, REPS, &stats_remotecall_4r)

	struct statistics stats_remotecall_5;
	BENCHMARK(GATECALL_5, WARMUP_REPS, REPS, &stats_remotecall_5)

	struct statistics stats_remotecall_5r;
	BENCHMARK(GATECALL_5R, WARMUP_REPS, REPS, &stats_remotecall_5r)

	struct statistics stats_remotecall_6;
	BENCHMARK(GATECALL_6, WARMUP_REPS, REPS, &stats_remotecall_6)

	struct statistics stats_remotecall_6r;
	BENCHMARK(GATECALL_6R, WARMUP_REPS, REPS, &stats_remotecall_6r)


#if COMPUTE_MEDIAN
	printf("%16s %4s \t %8s \t %8s %8s \t %8s\n", "name", "min", "max", "median", "average", "sdev");
#else
	printf("%16s %4s \t %8s \t %16s\n", "name", "min", "max", "median/avg");
#endif
	print_stats(&rdtsc_overhead, "rdtsc_overhead");

	/* results for local calls */
	print_stats(&stats_fcall_0,  "fcall_0");
	print_stats(&stats_fcall_0r, "fcall_0r");

	print_stats(&stats_fcall_1,  "fcall_1");
	print_stats(&stats_fcall_1r, "fcall_1r");

	print_stats(&stats_fcall_2,  "fcall_2");
	print_stats(&stats_fcall_2r, "fcall_2r");

	print_stats(&stats_fcall_3,  "fcall_3");
	print_stats(&stats_fcall_3r, "fcall_3r");

	print_stats(&stats_fcall_4,  "fcall_4");
	print_stats(&stats_fcall_4r, "fcall_4r");

	print_stats(&stats_fcall_5,  "fcall_5");
	print_stats(&stats_fcall_5r, "fcall_5r");

	print_stats(&stats_fcall_6, "fcall_6");
	print_stats(&stats_fcall_6r, "fcall_6r");

    /* results for remote calls */
	print_stats(&stats_remotecall_0, "remotecall_0");
	print_stats(&stats_remotecall_0r, "remotecall_0r");

	print_stats(&stats_remotecall_1, "remotecall_1");
	print_stats(&stats_remotecall_1r, "remotecall_1r");

	print_stats(&stats_remotecall_2, "remotecall_2");
	print_stats(&stats_remotecall_2r, "remotecall_2r");

	print_stats(&stats_remotecall_3, "remotecall_3");
	print_stats(&stats_remotecall_3r, "remotecall_3r");

	print_stats(&stats_remotecall_4, "remotecall_4");
	print_stats(&stats_remotecall_4r, "remotecall_4r");

	print_stats(&stats_remotecall_5, "remotecall_5");
	print_stats(&stats_remotecall_5r, "remotecall_5r");

	print_stats(&stats_remotecall_6, "remotecall_6");
	print_stats(&stats_remotecall_6r, "remotecall_6r");

	FINALIZE_MICROBENCHMARKS()
#endif

    return 0;
}
