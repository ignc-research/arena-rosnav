#ifndef F_MATH_H
#define F_MATH_H

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>

//often used math-functions
#ifdef __cplusplus
extern "C" {
#endif


//returning maximum/minimum of two given values
int f_imax(int a, int b);
int f_imin(int a, int b);
float f_fmax(float a, float b);
float f_fmin(float a, float b);

//limiting given value to upper and lower bound
int f_ilimit(int a, int max, int min);
float f_flimit(float a, float max, float min);

//round to next integer
float f_round(float a);

//signum: returns 1 if a is positive, -1 if a is negative or else 0
float f_fsign(float a);
int f_isign(int a);

//converting radiants to degrees and vice versa
float f_rad(float deg);
float f_deg(float rad);

int f_equals(float a, float b, float epsilon);

//interpolating quadratic from 0 to 1
float f_quadricInterpolate(float t);

// random value between 0 and 1
double f_random();

// returns a random value in [min, max]
int f_irandomRange(int min, int max);
float f_frandomRange(float min, float max);

// selects a random index in [0, num_buckets[ according to frequency
// if sum != NULL total the value of @sum will be used as sum of all frequencies, otherwise it is calculated in this function
int f_randomBuckets(const float * bucket_freq, int num_buckets, const float *sum);

// selection sort on array
void f_selectionSort(float * array, int size);

#ifdef __cplusplus
}
#endif
#endif
