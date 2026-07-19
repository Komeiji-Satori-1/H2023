#include "arm_math.h"
#include "arm_const_structs.h"
#include "math.h"
#include <stdio.h>
#include "main.h"
#include "usart.h"
#include "HMI.h"
#include "AD9833.h"

typedef struct
{
    float real;
    float imag;
    float mag;
    float phase;
} FFT_SingleFreqResult_t;

typedef struct
{
    float real;
    float imag;
    float mag;
    float phase;
} FFT_TransferResult_t;

void FFT_SingleFreqDFT_U16(const uint16_t *adc_buf,
                           uint16_t len,
                           float fs,
                           float target_freq,
                           FFT_SingleFreqResult_t *result);

void FFT_CalcTransfer(const FFT_SingleFreqResult_t *input,
                      const FFT_SingleFreqResult_t *output,
                      FFT_TransferResult_t *h);

void FFT_Process(uint16_t *ADC_Buffer, float *FFT_Ampl);
void IFFT_Process(void);
void window(void);
void ADC_FFT_Get_Wave_Mes(uint32_t Row,float Fs,float *VPP,float *Freq,int correctNum);
void Find_BaseIndex(void);
void Process_FFT_mag(float *FFT_mag, float *FFT_mag_max, uint32_t *FFT_mag_max_index,float *FFT_Ampl);
void showdata(float *buffer, uint16_t n);
void FFT_SetSampling(float sampling_freq);
float Calculate_DC_Value(uint16_t *ADC_Buffer);
