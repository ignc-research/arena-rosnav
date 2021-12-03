#include "f_math.h"

int f_equals(float a, float b, float epsilon)
{
	return (fabs(a-b) <= epsilon);
}

int f_imax(int a, int b)
{
	if(a > b)
		return a;

	return b;
}

int f_imin(int a, int b)
{
	if(a < b)
		return a;

	return b;
}

float f_fmax(float a, float b)
{
	if(a > b)
		return a;

	return b;
}

float f_fmin(float a, float b)
{
	if(a < b)
		return a;

	return b;
}


int f_ilimit(int a, int max, int min)
{
	if(a < min)
		return min;

	if(a > max)
		return max;

	return a;
}

float f_flimit(float a, float max, float min)
{
	if(a < min)
		return min;

	if(a > max)
		return max;

	return a;
}

float f_round(float a)
{
	if(a >= 0)
		return (float)((int)(a+0.5f));
	
	return (float)((int)(a-0.5f));
}

float f_fsign(float a)
{
	if(a > 0.f)
		return 1.f;

	if(a < 0.f)
		return -1.f;

	return 0.f;
}

int f_isign(int a)
{
	if(a > 0)
		return 1;

	if(a < 0)
		return -1;

	return 0;
}

float f_rad(float deg)
{
	return static_cast<float>(M_PI)*(deg/180.f);
}

float f_deg(float rad)
{
	return 180.f*(rad/static_cast<float>(M_PI));
}

float f_quadricInterpolate(float t)
{
	if(t <= 0.5f)
		return 2*t*t;

	float t_ = t-1;
	return 1 - 2*t_*t_;
}

double f_random()
{
	return rand()/(double)RAND_MAX;
}

int f_irandomRange(int min, int max)
{
	return rand()%(max-min + 1) + min;
}

float f_frandomRange(float min, float max)
{
	return f_random()*(max-min) + min;
}

int f_randomBuckets(const float * bucket_freq, int num_buckets, const float *sum)
{
	float s = 0.f;
	int i;
	if(sum == NULL){// sum not given -> calculate
		for(i = 0; i < num_buckets; i++){
			s += bucket_freq[i];
		}
	}
	else{ // sum given use given value
		s = *sum;
	}
	float r = f_random()*s;
	float offset = 0.f;

	for(i = 0; i < num_buckets-1; i++){
		if(offset + bucket_freq[i] > r){
			return i;
		}
		offset += bucket_freq[i];
	}
	return i;
}

void f_selectionSort(float * array, int size)
{
	for(int i = 0; i < size-1; i++){
		int min = i;
		for(int j = i+1; j < size; j++){
			if(array[j] < array[min])
				min = j;
		}

		float t = array[i];
		array[i]= array[min];
		array[min] = t;
	}
}
