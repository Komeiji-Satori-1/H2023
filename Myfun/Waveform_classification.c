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

void wavetypedetect(float *FFT_mag, float fs, SignalInfo *A, SignalInfo *B)
{
    float uc_amp;
    FFT_Process(ADC_UC, &uc_amp);

    // 找两个峰 最大和次大
    float max1 = 0.0f, max2 = 0.0f;
    uint16_t index1 = 0, index2 = 0;
    for (uint16_t i = 2; i < FFT_LEN / 2 - 1; i++)
    {
        if (FFT_mag[i] > FFT_mag[i - 1] && FFT_mag[i] > FFT_mag[i + 1])
        {
            if (FFT_mag[i] > max1)
            {
                max1 = FFT_mag[i];
                index1 = i;
            }
        }
    }
    for (uint16_t i = 2; i < FFT_LEN / 2 - 1; i++)
    {
        if (abs((int)i - (int)index1) <= PEAK_GUARD)
        {
            continue;
        }

        if (FFT_mag[i] > FFT_mag[i - 1] && FFT_mag[i] > FFT_mag[i + 1])
        {
            if (FFT_mag[i] > max2)
            {
                max2 = FFT_mag[i];
                index2 = i;
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

    // 波形判断
    uint16_t a_3_index = A->bin * 3;
    uint16_t b_3_index = B->bin * 3;
    uint16_t a_5_index = A->bin * 5;
    uint16_t b_5_index = B->bin * 5;

    //float a_3_ratio = FFT_mag[a_3_index] / FFT_mag[A->bin];
    //if (a_3_ratio > 0.08f)
    //{
    //    A->type = WAVE_TRIANGLE;
    //    B->type = WAVE_SINE;
    //}
    //else
    //{
    //    if (B->bin * 3 < FFT_LEN / 2 && FFT_mag[b_3_index] / FFT_mag[B->bin] > 0.08f)
    //    {
    //        A->type = WAVE_SINE;
    //        B->type = WAVE_TRIANGLE;
    //    }
    //    else
    //    {
    //        A->type = WAVE_SINE;
    //        B->type = WAVE_SINE;
    //    }
    //}

    
    // 注：以下是我写的部分，根据题意，只有一个三角波因此判断A为三角波后B必定为正弦。而判断A为正弦后B的类型判断没写
    if (FFT_mag[a_3_index] > 0)
    {
        if (a_3_index == B->bin)
        {
            if (FFT_mag[a_5_index] > 0.0f)
            {
                if (FFT_mag[a_5_index] / FFT_mag[A->bin] > 0.02f)
                {
                    A->type = WAVE_TRIANGLE;
                    B->type = WAVE_SINE;
                }
                else
                {
                    A->type = WAVE_SINE;
                    // 缺少B的判断
                }
            }
            else
            {
                A->type = WAVE_SINE;
                // 缺少B的判断
            }
        }
        else
        {
            A->type = WAVE_TRIANGLE;
            B->type = WAVE_SINE;
        }
    }
    //此分支应当为A三次谐波幅值为零、五次谐波幅值不为零的情况，但是前面没加return所以有bug
    else if (FFT_mag[a_5_index] > 0)
    {
        if (a_5_index == B->bin)
        {
            if (FFT_mag[a_3_index] > 0.0f)
            {
                //理论上在三次谐波为零情况不会进入这个分支。为了可读性高增加了这个判断。
                if (FFT_mag[a_3_index] / FFT_mag[A->bin] > 0.08f)
                {
                    A->type = WAVE_TRIANGLE;
                    B->type = WAVE_SINE;
                }
                else
                {
                    A->type = WAVE_SINE;
                    // 缺少B的判断
                }
            }
            else
            {
                A->type = WAVE_SINE;
                // 缺少B的判断
            }
        }
        else if (a_5_index == B->bin * 3)
        {
            //理论上在三次谐波为零情况不会进入这个分支。为了可读性高增加了这个判断。
            if (FFT_mag[a_3_index] > 0.0f)
            {
                if (FFT_mag[a_3_index] / FFT_mag[A->bin] > 0.08f)
                {
                    A->type = WAVE_TRIANGLE;
                    B->type = WAVE_SINE;
                }
                else
                {
                    A->type = WAVE_SINE;
                    // 缺少B的判断
                }
            }
            else
            {
                A->type = WAVE_SINE;
                // 缺少B的判断
            }
        }
    }
    // A的三次谐波无幅值且五次谐波无幅值才会进入else但是前面没加return所以有bug
    else
    {
        //等你补充
    }
}


void test()
{
    wavetypedetect(FFT_mag, fs, &A, &B);
}
