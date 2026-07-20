#ifndef __Waveform_classification_H__
#define __Waveform_classification_H__

#include "FFT.h"
#include "math.h"
#include "HMI.h"
#include <stdint.h>
#include <stdlib.h>

typedef enum
{
    WAVE_SINE = 0,
    WAVE_TRIANGLE
} WaveType;

typedef struct
{
    uint32_t bin;
    float freq;
    float amp;
    float phase;
    WaveType type;
} SignalInfo;


extern SignalInfo A;
extern SignalInfo B;

void Waveform_SetAdcFrame(const uint16_t *adc_buf);
void wavetypedetect(const float *fft_mag, float fs, SignalInfo *A, SignalInfo *B);
void test(void);

#endif
