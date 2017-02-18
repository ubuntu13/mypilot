#include "filter.h"

static float samples[FILTER_SIZE] = {};
static uint8_t sample_index = 0;

void FilterWithBuffer_reset(void)
{
	// clear samples buffer
	for( int8_t i=0; i<FILTER_SIZE; i++ )
	{
		samples[i] = 0;
	}

	// reset index back to beginning of the array
	sample_index = 0;
}

// apply - take in a new raw sample, and return the filtered results
void FilterWithBuffer_apply(float sample)
{
    // add sample to array
    samples[sample_index++] = sample;

    // wrap index if necessary
    if( sample_index >= FILTER_SIZE )
        sample_index = 0;
}

float DerivativeFilter_slope(void)
{
    float result = 0;

    // use f() to make the code match the maths a bit better. Note
    // that unlike an average filter, we care about the order of the elements
#define f(i) samples[ ( ( (sample_index-1) +i+1 ) + 3*FILTER_SIZE/2 ) % FILTER_SIZE ]
#define t 0.02f

    // N in the paper is FILTER_SIZE
    switch (FILTER_SIZE) {
    case 5:
        result = 2*2*(f(1) - f(-1)) / (2 * t)
                 + 4*1*(f(2) - f(-2)) / (4 * t);
        result /= 8;
        break;
    case 7:
        result = 2*5*(f(1) - f(-1)) / (2 * t)
                 + 4*4*(f(2) - f(-2)) / (4 * t)
                 + 6*1*(f(3) - f(-3)) / (6 * t);
        result /= 32;
        break;
    case 9:
        result = 2*14*(f(1) - f(-1)) / (2 * t)
                 + 4*14*(f(2) - f(-2)) / (4 * t)
                 + 6* 6*(f(3) - f(-3)) / (6 * t)
                 + 8* 1*(f(4) - f(-4)) / (8 * t);
        result /= 128;
        break;
    case 11:
        result =  2*42*(f(1) - f(-1)) / (2 * t)
                 +  4*48*(f(2) - f(-2)) / (4 * t)
                 +  6*27*(f(3) - f(-3)) / (6 * t)
                 +  8* 8*(f(4) - f(-4)) / (8 * t)
                 + 10* 1*(f(5) - f(-5)) / (10 * t);
        result /= 512;
        break;
    default:
        result = 0;
        break;
    }

    // cope with numerical errors
    if (isnan(result) || isinf(result)) {
        result = 0;
    }

    return result;
}




