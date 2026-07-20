#include "Waveform_classification.h"

#define PEAK_GUARD 5U
#define HARMONIC_SEARCH_HALF_WIDTH 2U
#define TRI_3RD_RATIO_THRESHOLD 0.08f
#define TRI_5TH_RATIO_THRESHOLD 0.02f
#define BIN_MATCH_TOLERANCE HARMONIC_SEARCH_HALF_WIDTH
#define FFT_HALF_BIN (FFT_LEN / 2U)
#define FFT_LAST_BIN (FFT_HALF_BIN - 1U)
#define MAG_EPSILON 1.0e-12f

extern uint16_t ADC_C[ADC_LEN];
extern float fs;
extern float FFT_mag[FFT_LEN];

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

static uint8_t bins_near(uint32_t left_bin, uint32_t right_bin, uint32_t tolerance)
{
    uint32_t diff;

    if (left_bin > right_bin)
    {
        diff = left_bin - right_bin;
    }
    else
    {
        diff = right_bin - left_bin;
    }

    return (diff <= tolerance) ? 1U : 0U;
}

static uint8_t harmonic_bin_is_valid(uint32_t harmonic_bin)
{
    return ((harmonic_bin >= 2U) && (harmonic_bin <= FFT_LAST_BIN)) ? 1U : 0U;
}

static WaveType classify_by_ratio(float ratio, float threshold)
{
    return (ratio > threshold) ? WAVE_TRIANGLE : WAVE_SINE;
}

static WaveType classify_b_by_3rd(float b3_ratio)
{
    return classify_by_ratio(b3_ratio, TRI_3RD_RATIO_THRESHOLD);
}

static WaveType classify_b_by_5th_or_fallback(uint32_t b5_bin, float b5_ratio, float b3_ratio)
{
    if (harmonic_bin_is_valid(b5_bin) != 0U)
    {
        return classify_by_ratio(b5_ratio, TRI_5TH_RATIO_THRESHOLD);
    }

    return classify_b_by_3rd(b3_ratio);
}

void wavetypedetect(const float *fft_mag, float sampling_fs, SignalInfo *A, SignalInfo *B)
{
    float uc_amp;
    float max1 = 0.0f;
    float max2 = 0.0f;
    uint32_t index1 = 0U;
    uint32_t index2 = 0U;
    uint32_t a3_bin;
    uint32_t a5_bin;
    uint32_t b3_bin;
    uint32_t b5_bin;
    float a3_ratio;
    float a5_ratio;
    float b3_ratio;
    float b5_ratio;
    uint8_t a3_overlaps_b_base;
    uint8_t a5_overlaps_b3;

    if ((fft_mag == 0) || (A == 0) || (B == 0))
    {
        return;
    }

    FFT_Process((uint16_t *)s_adc_frame, &uc_amp);

    for (uint32_t i = 2U; i < FFT_HALF_BIN - 1U; i++)
    {
        if ((fft_mag[i] > fft_mag[i - 1U]) && (fft_mag[i] > fft_mag[i + 1U]))
        {
            if (fft_mag[i] > max1)
            {
                max1 = fft_mag[i];
                index1 = i;
            }
        }
    }
    for (uint32_t i = 2U; i < FFT_HALF_BIN - 1U; i++)
    {
        if (bins_near(i, index1, PEAK_GUARD) != 0U)
        {
            continue;
        }

        if ((fft_mag[i] > fft_mag[i - 1U]) && (fft_mag[i] > fft_mag[i + 1U]))
        {
            if (fft_mag[i] > max2)
            {
                max2 = fft_mag[i];
                index2 = i;
            }
        }
    }

    if ((index1 == 0U) || (index2 == 0U))
    {
        return;
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

    ADC_FFT_Get_Wave_Mes(A->bin, sampling_fs, &A->amp, &A->freq, 2);
    ADC_FFT_Get_Wave_Mes(B->bin, sampling_fs, &B->amp, &B->freq, 2);

    a3_bin = A->bin * 3U;
    a5_bin = A->bin * 5U;
    b3_bin = B->bin * 3U;
    b5_bin = B->bin * 5U;

    a3_ratio = harmonic_ratio_near(fft_mag, A->bin, a3_bin, HARMONIC_SEARCH_HALF_WIDTH, 0);
    a5_ratio = harmonic_ratio_near(fft_mag, A->bin, a5_bin, HARMONIC_SEARCH_HALF_WIDTH, 0);
    b3_ratio = harmonic_ratio_near(fft_mag, B->bin, b3_bin, HARMONIC_SEARCH_HALF_WIDTH, 0);
    b5_ratio = harmonic_ratio_near(fft_mag, B->bin, b5_bin, HARMONIC_SEARCH_HALF_WIDTH, 0);

    a3_overlaps_b_base = bins_near(a3_bin, B->bin, BIN_MATCH_TOLERANCE);
    a5_overlaps_b3 = bins_near(a5_bin, b3_bin, BIN_MATCH_TOLERANCE);

    if (a3_overlaps_b_base != 0U)
    {
        A->type = classify_by_ratio(a5_ratio, TRI_5TH_RATIO_THRESHOLD);
        B->type = classify_b_by_3rd(b3_ratio);
    }
    else if (a3_ratio > TRI_3RD_RATIO_THRESHOLD)
    {
        A->type = WAVE_TRIANGLE;

        if (a5_overlaps_b3 != 0U)
        {
            B->type = classify_b_by_5th_or_fallback(b5_bin, b5_ratio, b3_ratio);
        }
        else
        {
            B->type = classify_b_by_3rd(b3_ratio);
        }
    }
    else
    {
        A->type = WAVE_SINE;
        B->type = classify_b_by_3rd(b3_ratio);
    }

    HMI_send_string("t5", (A->type == WAVE_TRIANGLE) ? "Triangle" : "Sine");
    HMI_send_string("t6", (B->type == WAVE_TRIANGLE) ? "Triangle" : "Sine");
}

void test(void)
{
    wavetypedetect(FFT_mag, fs, &A, &B);
}
