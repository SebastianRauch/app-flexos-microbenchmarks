#include <flexos/isolation.h>
#include <flexos/microbenchmarks/isolated.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

#include <uk/alloc.h>
#include <uk/sched.h>

#include <flexos/impl/main_annotation.h>


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

#define BENCH_START_SER() bench_start()
#define BENCH_END_SER() bench_end()

#define BENCH_START_NO_SER() readtsc()
#define BENCH_END_NO_SER() readtsc()

#define REPS 10000
#define WARMUP_REPS 1000

static inline int cmp_int(const void *x, const void *y) {
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
	double sdev;
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
		out_stats->median = measurements[n / 2];
	} else {
		out_stats->median = (measurements[n / 2] + measurements[n / 2 - 1]) / 2;
	}
	out_stats->average = avg;
	out_stats->sdev = sqrt(var);
}


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

void print_stats(struct statistics *stats, const char *str) {
    int64_t a, b;
    uint64_t x, y;
    fraction_to_dec(stats->average, 2, &a, &x);
    fraction_to_dec(stats->sdev, 2, &b, &y);
	// fomat: description min max median average sdev
	printf("%16s %4ld \t %8ld \t %4ld \t %4ld.%ld \t %8ld.%ld\n",
        str, stats->min, stats->max, stats->median, a, x, b, y);
}

int main(int argc, char *argv[]) {
	volatile int *stop = (volatile int *) uk_malloc(flexos_shared_alloc, sizeof(int));
	*stop = 0;
	volatile int *state = (volatile int *) (&comm);
	*state = STATE_IDLE;


	uint64_t *results = uk_malloc(uk_alloc_get_default(), REPS * sizeof(uint64_t));

	flexos_gate(libflexosmicrobenchmarks, start_comm_test, stop);

	uint64_t t0, t1, diff;
	uint64_t sum = 0, min = (uint64_t) -1, max = 0;
	for (int i = 0; i < WARMUP_REPS; ++i) {
		*state = STATE_IDLE;
		t0 = BENCH_START_SER();
		*state = STATE_SENT;
		while (*state != STATE_RET) {
			asm volatile("pause" ::: "memory");
		}
		t1 = BENCH_END_SER();
		results[i] = t1 - t0;
	}
	for (int i = 0; i < REPS; ++i) {
		*state = STATE_IDLE;
		t0 = BENCH_START_SER();
		*state = STATE_SENT;
		while (*state != STATE_RET) {
			asm volatile("pause" ::: "memory");
		}
		t1 = BENCH_END_SER();
		results[i] = t1 - t0;
		/*	
		diff = t1 - t0;
		if (diff < min) min = diff;
		if (diff > max) max = diff;
		sum += diff;
		*/
	}
	
	struct statistics stats_ser;
	do_statistics(results, REPS, &stats_ser);
	
	printf("# with serialization:\n");
	print_stats(&stats_ser, "");
	//uk_pr_info("%ld \t %ld \t %ld\n", min, max, sum / runs);
	
	sum = 0;
	min = (uint64_t) -1;
	max = 0;

	for (int i = 0; i < WARMUP_REPS; ++i) {
		*state = STATE_IDLE;
		t0 = BENCH_START_NO_SER();
		*state = STATE_SENT;
		while (*state != STATE_RET) {
			asm volatile("pause" ::: "memory");
		}
		t1 = BENCH_END_NO_SER();
		results[i] = t1 - t0;
	}
	for (int i = 0; i < REPS; ++i) {
		*state = STATE_IDLE;
		t0 = BENCH_START_NO_SER();
		*state = STATE_SENT;
		while (*state != STATE_RET) {
			asm volatile("pause" ::: "memory");
		} 
		t1 = BENCH_END_NO_SER();
		results[i] = t1 - t0;
		/*	
		diff = t1 - t0;
		if (diff < min) min = diff;
		if (diff > max) max = diff;
		sum += diff;
		*/
	}

	struct statistics stats_no_ser;
	do_statistics(results, REPS, &stats_no_ser);

	printf("# without serialization:\n");
	print_stats(&stats_no_ser, "");
	//uk_pr_info("%ld \t %ld \t %ld\n", min, max, sum / runs);

	*stop = 1;
	uk_free(uk_alloc_get_default(), results);
	return 0;
}
