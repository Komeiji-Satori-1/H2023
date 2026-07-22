#include "ad9833_fll.h"
#include "AD9833.h"
#include <math.h>
#include <string.h>

#define AD9833_FLL_CHANNELS 2U
#define AD9833_FLL_PI 3.14159265358979323846f
#define AD9833_FLL_TWO_PI (2.0f * AD9833_FLL_PI)
#define AD9833_FLL_MIN_FREQ_HZ 1.0f
#define AD9833_FLL_MAX_FREQ_HZ 1000000.0f
#define AD9833_FLL_LOCK_MIN_BIN 2U
#define AD9833_FLL_LOCK_MIN_MAG 1.0f
#define AD9833_FLL_PID_KP 3.2f
#define AD9833_FLL_PID_KI 0.06f
#define AD9833_FLL_PID_KD 0.0f
#define AD9833_FLL_PID_I_MAX 2.5f
#define AD9833_FLL_PID_OUT_MAX 500.0f
#define AD9833_FLL_FREQ_OFFSET_A 0.0f
#define AD9833_FLL_FREQ_OFFSET_B 0.0f

typedef struct
{
    float base_freq_hz;
    float output_freq_hz;
    float freq_corr_hz;
    float freq_offset_hz;
    uint16_t wave_type;
    uint8_t chip;
    pid_struct_t pid;
    Ad9833FllDebug_t debug;
} Ad9833FllChannel_t;

static Ad9833FllChannel_t s_fll[AD9833_FLL_CHANNELS];
static uint8_t s_ready = 0;
static uint8_t s_running = 0;
static uint16_t s_phase_deg = 0;

static float Ad9833Fll_LimitFloat(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

static float Ad9833Fll_WrapPi(float phase_rad)
{
    while (phase_rad > AD9833_FLL_PI)
    {
        phase_rad -= AD9833_FLL_TWO_PI;
    }

    while (phase_rad < -AD9833_FLL_PI)
    {
        phase_rad += AD9833_FLL_TWO_PI;
    }

    return phase_rad;
}

static uint32_t Ad9833Fll_FindPeakBin(const float *mag, uint16_t len)
{
    uint32_t peak_bin = AD9833_FLL_LOCK_MIN_BIN;
    float peak_mag = 0.0f;
    uint32_t i;

    if (mag == 0U)
    {
        return 0U;
    }

    for (i = AD9833_FLL_LOCK_MIN_BIN; i < (uint32_t)(len / 2U); i++)
    {
        if (mag[i] > peak_mag)
        {
            peak_mag = mag[i];
            peak_bin = i;
        }
    }

    return peak_bin;
}

static void Ad9833Fll_TopTwoBins(const float *mag, uint16_t len, uint32_t *bin0, uint32_t *bin1)
{
    uint32_t i;
    uint32_t first = 0U;
    uint32_t second = 0U;
    float first_mag = 0.0f;
    float second_mag = 0.0f;

    if ((mag == 0U) || (bin0 == 0U) || (bin1 == 0U))
    {
        return;
    }

    for (i = AD9833_FLL_LOCK_MIN_BIN; i < (uint32_t)(len / 2U); i++)
    {
        if (mag[i] > first_mag)
        {
            second_mag = first_mag;
            second = first;
            first_mag = mag[i];
            first = i;
        }
        else if (mag[i] > second_mag)
        {
            second_mag = mag[i];
            second = i;
        }
    }

    if (first > second)
    {
        uint32_t temp = first;
        first = second;
        second = temp;
    }

    *bin0 = first;
    *bin1 = second;
}

static void Ad9833Fll_SingleBinDft(const uint16_t *adc_buf,
                                  uint16_t len,
                                  uint32_t bin,
                                  FFT_SingleFreqResult_t *result)
{
    uint16_t i;
    float dc = 0.0f;
    float real = 0.0f;
    float imag = 0.0f;
    float sample;
    float angle_step;
    float angle;
    float cos_step;
    float sin_step;
    float cos_val;
    float sin_val;

    if ((adc_buf == 0U) || (result == 0U) || (len == 0U) || (bin >= (uint32_t)(len / 2U)))
    {
        return;
    }

    for (i = 0; i < len; i++)
    {
        dc += (float)adc_buf[i];
    }
    dc /= (float)len;

    angle_step = (2.0f * AD9833_FLL_PI * (float)bin) / (float)len;
    cos_step = cosf(angle_step);
    sin_step = sinf(angle_step);
    cos_val = 1.0f;
    sin_val = 0.0f;

    for (i = 0; i < len; i++)
    {
        sample = (float)adc_buf[i] - dc;
        real += sample * cos_val;
        imag -= sample * sin_val;

        angle = cos_val * cos_step - sin_val * sin_step;
        sin_val = sin_val * cos_step + cos_val * sin_step;
        cos_val = angle;
    }

    result->real = real;
    result->imag = imag;
    result->mag = 2.0f * sqrtf(real * real + imag * imag) / (float)len;
    result->phase = atan2f(real, imag);
}

static void Ad9833Fll_AnalyseFrame(const uint16_t *frame,
                                   uint16_t len,
                                   FFT_SingleFreqResult_t *first,
                                   FFT_SingleFreqResult_t *second)
{
    float mag_buf[AD9833_FLL_LOCK_MIN_BIN + 256U];
    uint32_t i;
    uint32_t bin0;
    uint32_t bin1;
    FFT_SingleFreqResult_t temp;

    if ((frame == 0U) || (first == 0U) || (second == 0U) || (len == 0U))
    {
        return;
    }

    if (len > 256U)
    {
        len = 256U;
    }

    for (i = 0; i < (uint32_t)(len / 2U); i++)
    {
        mag_buf[i] = 0.0f;
    }

    for (i = AD9833_FLL_LOCK_MIN_BIN; i < (uint32_t)(len / 2U); i++)
    {
        Ad9833Fll_SingleBinDft(frame, len, i, &temp);
        mag_buf[i] = temp.mag;
    }

    Ad9833Fll_TopTwoBins(mag_buf, len, &bin0, &bin1);
    if ((bin0 == 0U) || (bin1 == 0U))
    {
        memset(first, 0, sizeof(*first));
        memset(second, 0, sizeof(*second));
        return;
    }

    Ad9833Fll_SingleBinDft(frame, len, bin0, first);
    Ad9833Fll_SingleBinDft(frame, len, bin1, second);
}

static void Ad9833Fll_AnalysePeakFrame(const uint16_t *frame,
                                       uint16_t len,
                                       FFT_SingleFreqResult_t *peak)
{
    float mag_buf[AD9833_FLL_LOCK_MIN_BIN + 256U];
    uint32_t i;
    uint32_t peak_bin;
    FFT_SingleFreqResult_t temp;

    if ((frame == 0U) || (peak == 0U) || (len == 0U))
    {
        return;
    }

    if (len > 256U)
    {
        len = 256U;
    }

    for (i = 0; i < (uint32_t)(len / 2U); i++)
    {
        mag_buf[i] = 0.0f;
    }

    for (i = AD9833_FLL_LOCK_MIN_BIN; i < (uint32_t)(len / 2U); i++)
    {
        Ad9833Fll_SingleBinDft(frame, len, i, &temp);
        mag_buf[i] = temp.mag;
    }

    peak_bin = Ad9833Fll_FindPeakBin(mag_buf, len);
    if (peak_bin == 0U)
    {
        memset(peak, 0, sizeof(*peak));
        return;
    }

    Ad9833Fll_SingleBinDft(frame, len, peak_bin, peak);
}

static uint16_t Ad9833Fll_WaveTypeToAd9833(WaveType type)
{
    if (type == WAVE_TRIANGLE)
    {
        return ad9833_Triangle;
    }

    return ad9833_Sine;
}

static float Ad9833Fll_ValidFreq(float freq_hz)
{
    if (freq_hz < AD9833_FLL_MIN_FREQ_HZ)
    {
        return AD9833_FLL_MIN_FREQ_HZ;
    }

    if (freq_hz > AD9833_FLL_MAX_FREQ_HZ)
    {
        return AD9833_FLL_MAX_FREQ_HZ;
    }

    return freq_hz;
}

static void Ad9833Fll_ConfigChannel(uint8_t index,
                                    const SignalInfo *sig,
                                    uint8_t chip)
{
    if ((index >= AD9833_FLL_CHANNELS) || (sig == 0))
    {
        return;
    }

    memset(&s_fll[index], 0, sizeof(s_fll[index]));
    s_fll[index].base_freq_hz = Ad9833Fll_ValidFreq(sig->freq);
    s_fll[index].output_freq_hz = s_fll[index].base_freq_hz;
    s_fll[index].freq_corr_hz = 0.0f;
    s_fll[index].freq_offset_hz = (index == 0U) ? AD9833_FLL_FREQ_OFFSET_A : AD9833_FLL_FREQ_OFFSET_B;
    s_fll[index].wave_type = Ad9833Fll_WaveTypeToAd9833(sig->type);
    s_fll[index].chip = chip;

    pid_init(&s_fll[index].pid,
             AD9833_FLL_PID_KP,
             AD9833_FLL_PID_KI,
             AD9833_FLL_PID_KD,
             AD9833_FLL_PID_I_MAX,
             AD9833_FLL_PID_OUT_MAX,
             1.0f);
}

static void Ad9833Fll_UpdateChannel(uint8_t index,
                                    const FFT_SingleFreqResult_t *ref,
                                    const FFT_SingleFreqResult_t *fb)
{
    float error;
    float pid_out;
    float next_freq;

    if ((index >= AD9833_FLL_CHANNELS) || (ref == 0) || (fb == 0))
    {
        return;
    }

    if ((ref->mag < AD9833_FLL_LOCK_MIN_MAG) || (fb->mag < AD9833_FLL_LOCK_MIN_MAG))
    {
        return;
    }

    error = Ad9833Fll_WrapPi(fb->phase - ref->phase);
    pid_out = pid_calc(&s_fll[index].pid, 0.0f, error);
    pid_out = Ad9833Fll_LimitFloat(pid_out,
                                   -AD9833_FLL_PID_OUT_MAX,
                                   AD9833_FLL_PID_OUT_MAX);

    s_fll[index].freq_corr_hz = pid_out;
    next_freq = s_fll[index].base_freq_hz - s_fll[index].freq_corr_hz + s_fll[index].freq_offset_hz;
    //next_freq = s_fll[index].base_freq_hz + 500.0f;
    s_fll[index].output_freq_hz = next_freq;

    ad9833_set_freq_ch_live((uint32_t)(next_freq + 0.5f),
                            s_fll[index].wave_type,
                            s_fll[index].chip);

    s_fll[index].debug.base_freq_hz = s_fll[index].base_freq_hz;
    s_fll[index].debug.output_freq_hz = s_fll[index].output_freq_hz;
    s_fll[index].debug.freq_corr_hz = s_fll[index].freq_corr_hz;
    s_fll[index].debug.phase_error_rad = error;
    s_fll[index].debug.ref_phase_rad = ref->phase;
    s_fll[index].debug.fb_phase_rad = fb->phase;
    s_fll[index].debug.ref_mag = ref->mag;
    s_fll[index].debug.fb_mag = fb->mag;
    s_fll[index].debug.wave_type = s_fll[index].wave_type;
    s_fll[index].debug.chip = s_fll[index].chip;
}

void Ad9833Fll_Config(const SignalInfo *sig_a,
                      const SignalInfo *sig_b,
                      uint16_t phase_deg)
{
    if ((sig_a == 0) || (sig_b == 0))
    {
        s_ready = 0;
        s_running = 0;
        return;
    }

    s_phase_deg = phase_deg;
    Ad9833Fll_ConfigChannel(0U, sig_a, AD9833_CH1);
    Ad9833Fll_ConfigChannel(1U, sig_b, AD9833_CH2);

    ad9833_sync_start((uint32_t)(s_fll[0].base_freq_hz + 0.5f),
                      s_fll[0].wave_type,
                      (uint32_t)(s_fll[1].base_freq_hz + 0.5f),
                      s_fll[1].wave_type);
    ad9833_write_phase(0.0f, (float)s_phase_deg);

    s_ready = 1;
    s_running = 0;
}

void Ad9833Fll_Start(void)
{
    if (!s_ready)
    {
        return;
    }

    pid_reset(&s_fll[0].pid);
    pid_reset(&s_fll[1].pid);
    s_running = 1;
}

void Ad9833Fll_Stop(void)
{
    s_running = 0;
}

void Ad9833Fll_UpdatePhase(uint16_t phase_deg)
{
    s_phase_deg = phase_deg;
    ad9833_write_phase(0.0f, (float)s_phase_deg);
}

void Ad9833Fll_Task(const uint16_t *c_frame,
                    const uint16_t *a_feedback_frame,
                    const uint16_t *b_feedback_frame,
                    uint16_t len,
                    float fs_hz)
{
    FFT_SingleFreqResult_t ref_a;
    FFT_SingleFreqResult_t ref_b;
    FFT_SingleFreqResult_t fb_a;
    FFT_SingleFreqResult_t fb_b;
    uint16_t use_len;

    if ((!s_ready) || (!s_running) ||
        (c_frame == 0) || (a_feedback_frame == 0) || (b_feedback_frame == 0))
    {
        return;
    }

    (void)fs_hz;
    use_len = (len > 256U) ? 256U : len;
    Ad9833Fll_AnalyseFrame(c_frame, use_len, &ref_a, &ref_b);
    Ad9833Fll_AnalysePeakFrame(a_feedback_frame, use_len, &fb_a);
    Ad9833Fll_AnalysePeakFrame(b_feedback_frame, use_len, &fb_b);

    Ad9833Fll_UpdateChannel(0U, &ref_a, &fb_a);
    Ad9833Fll_UpdateChannel(1U, &ref_b, &fb_b);
}

uint8_t Ad9833Fll_IsRunning(void)
{
    return s_running;
}

const Ad9833FllDebug_t *Ad9833Fll_GetDebug(uint8_t ch)
{
    if (ch >= AD9833_FLL_CHANNELS)
    {
        return 0;
    }

    return &s_fll[ch].debug;
}
