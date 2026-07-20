#include "Waveform_classification.h"

#define PEAK_GUARD 5U
#define HARMONIC_SEARCH_HALF_WIDTH 2U
#define TRI_3RD_RATIO_THRESHOLD 0.08f
#define TRI_5TH_RATIO_THRESHOLD 0.02f
#define FFT_HALF_BIN (FFT_LEN / 2U)
#define FFT_LAST_BIN (FFT_HALF_BIN - 1U)
#define MAG_EPSILON 1.0e-12f

extern uint16_t ADC_C[ADC_LEN];
extern float fs;
extern float FFT_mag[FFT_LEN];
extern float FFT_Input[FFT_LEN * 2];

SignalInfo A;
SignalInfo B;
static const uint16_t *s_adc_frame = ADC_C;

void Waveform_SetAdcFrame(const uint16_t *adc_buf)
{
    if (adc_buf != 0)
    {
        s_adc_frame = adc_buf;
    }
    else
    {
        s_adc_frame = ADC_C;
    }
}

static uint32_t clamp_bin(int32_t bin)
{
    if (bin < 2)
    {
        return 2U;
    }

    if ((uint32_t)bin > FFT_LAST_BIN)
    {
        return FFT_LAST_BIN;
    }

    return (uint32_t)bin;
}

static uint32_t find_peak_in_window(const float *mag, uint32_t start_bin, uint32_t end_bin, float *peak_mag)
{
    uint32_t peak_bin;
    float max_mag;

    start_bin = clamp_bin((int32_t)start_bin);
    end_bin = clamp_bin((int32_t)end_bin);
    if (start_bin > end_bin)
    {
        uint32_t tmp = start_bin;
        start_bin = end_bin;
        end_bin = tmp;
    }

    peak_bin = start_bin;
    max_mag = mag[start_bin];
    for (uint32_t i = start_bin + 1U; i <= end_bin; i++)
    {
        if (mag[i] > max_mag)
        {
            max_mag = mag[i];
            peak_bin = i;
        }
    }

    if (peak_mag != 0)
    {
        *peak_mag = max_mag;
    }

    return peak_bin;
}

static float harmonic_ratio_near(const float *mag,
                                 uint32_t base_bin,
                                 uint32_t harmonic_bin,
                                 uint32_t half_width,
                                 uint32_t *peak_bin)
{
    uint32_t start_bin;
    uint32_t end_bin;
    float harmonic_mag = 0.0f;
    float base_mag;

    if ((mag == 0) || (base_bin > FFT_LAST_BIN))
    {
        if (peak_bin != 0)
        {
            *peak_bin = 0U;
        }
        return 0.0f;
    }

    if (harmonic_bin > FFT_LAST_BIN)
    {
        if (peak_bin != 0)
        {
            *peak_bin = 0U;
        }
        return 0.0f;
    }

    start_bin = (harmonic_bin > half_width) ? (harmonic_bin - half_width) : 2U;
    end_bin = harmonic_bin + half_width;
    if (end_bin > FFT_LAST_BIN)
    {
        end_bin = FFT_LAST_BIN;
    }

    if (start_bin > end_bin)
    {
        if (peak_bin != 0)
        {
            *peak_bin = 0U;
        }
        return 0.0f;
    }

    if (peak_bin != 0)
    {
        *peak_bin = find_peak_in_window(mag, start_bin, end_bin, &harmonic_mag);
    }
    else
    {
        (void)find_peak_in_window(mag, start_bin, end_bin, &harmonic_mag);
    }

    base_mag = mag[base_bin];
    if (base_mag <= MAG_EPSILON)
    {
        return 0.0f;
    }

    return harmonic_mag / base_mag;
}

void wavetypedetect(const float *fft_mag, float fs, SignalInfo *A, SignalInfo *B)
{
    float uc_amp;
    float max1 = 0.0f;
    float max2 = 0.0f;
    uint32_t index1 = 0U;
    uint32_t index2 = 0U;
    float a_3_ratio;
    float a_5_ratio;
    float b_3_ratio;
    float b_5_ratio;

    FFT_Process((uint16_t *)s_adc_frame, &uc_amp);

    if ((fft_mag == 0) || (A == 0) || (B == 0))
    {
        return;
    }

    for (uint32_t i = 2U; i < FFT_HALF_BIN - 1U; i++)
    {
        if ((fft_mag[i] > fft_mag[i - 1U]) && (fft_mag[i] > fft_mag[i + 1U]))
        {
            if (fft_mag[i] > max1)
            {
                max1 = fft_mag[i];
                index1 = (uint16_t)i;
            }
        }
    }

    for (uint32_t i = 2U; i < FFT_HALF_BIN - 1U; i++)
    {
        uint32_t delta = (i > index1) ? (i - index1) : (index1 - i);

        if (delta <= PEAK_GUARD)
        {
            continue;
        }

        if ((fft_mag[i] > fft_mag[i - 1U]) && (fft_mag[i] > fft_mag[i + 1U]))
        {
            if (fft_mag[i] > max2)
            {
                max2 = fft_mag[i];
                index2 = (uint16_t)i;
            }
        }
    }

    if (index1 < index2)
    {
        A->bin = index1;
        B->bin = index2;
    }
    else
    {
        A->bin = index2;
        B->bin = index1;
    }

    ADC_FFT_Get_Wave_Mes(A->bin, fs, &A->amp, &A->freq, 2);
    ADC_FFT_Get_Wave_Mes(B->bin, fs, &B->amp, &B->freq, 2);

    a_3_ratio = harmonic_ratio_near(fft_mag, A->bin, A->bin * 3U, HARMONIC_SEARCH_HALF_WIDTH, 0);
    a_5_ratio = harmonic_ratio_near(fft_mag, A->bin, A->bin * 5U, HARMONIC_SEARCH_HALF_WIDTH, 0);
    b_3_ratio = harmonic_ratio_near(fft_mag, B->bin, B->bin * 3U, HARMONIC_SEARCH_HALF_WIDTH, 0);
    b_5_ratio = harmonic_ratio_near(fft_mag, B->bin, B->bin * 5U, HARMONIC_SEARCH_HALF_WIDTH, 0);

    A->type = ((a_3_ratio >= TRI_3RD_RATIO_THRESHOLD) || (a_5_ratio >= TRI_5TH_RATIO_THRESHOLD))
              ? WAVE_TRIANGLE
              : WAVE_SINE;
    HMI_send_string("t5", (A->type == WAVE_TRIANGLE) ? "Triangle" : "Sine");
    B->type = ((b_3_ratio >= TRI_3RD_RATIO_THRESHOLD) || (b_5_ratio >= TRI_5TH_RATIO_THRESHOLD))
              ? WAVE_TRIANGLE
              : WAVE_SINE;
    HMI_send_string("t6", (B->type == WAVE_TRIANGLE) ? "Triangle" : "Sine");
}

void test(void)
{
    wavetypedetect(FFT_mag, fs, &A, &B);
}
