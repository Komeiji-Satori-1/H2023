#include "ad9833_fll.h"
#include "AD9833.h"
#include <math.h>
#include <string.h>

#define AD9833_FLL_CHANNELS 2U
#define AD9833_FLL_PI 3.14159265358979323846f
#define AD9833_FLL_TWO_PI (2.0f * AD9833_FLL_PI)
#define AD9833_FLL_MIN_FREQ_HZ 1.0f
#define AD9833_FLL_MAX_FREQ_HZ 1000000.0f
#define AD9833_FLL_MIN_MAG 1.0f
#define AD9833_FLL_PID_KP 12.0f
#define AD9833_FLL_PID_KI 0.08f
#define AD9833_FLL_PID_KD 0.0f
#define AD9833_FLL_MAX_CORR_HZ 200.0f

typedef struct
{
    float base_freq_hz;
    float output_freq_hz;
    float freq_corr_hz;
    uint16_t wave_type;
    uint8_t chip;
    PidController_t pid;
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
    s_fll[index].wave_type = Ad9833Fll_WaveTypeToAd9833(sig->type);
    s_fll[index].chip = chip;

    Pid_Init(&s_fll[index].pid,
             AD9833_FLL_PID_KP,
             AD9833_FLL_PID_KI,
             AD9833_FLL_PID_KD,
             -AD9833_FLL_MAX_CORR_HZ,
             AD9833_FLL_MAX_CORR_HZ);
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

    if ((ref->mag < AD9833_FLL_MIN_MAG) || (fb->mag < AD9833_FLL_MIN_MAG))
    {
        return;
    }

    error = Ad9833Fll_WrapPi(fb->phase - ref->phase);
    pid_out = Pid_Calc(&s_fll[index].pid, error);
    pid_out = Ad9833Fll_LimitFloat(pid_out,
                                   -AD9833_FLL_MAX_CORR_HZ,
                                   AD9833_FLL_MAX_CORR_HZ);

    s_fll[index].freq_corr_hz = pid_out;
    next_freq = s_fll[index].base_freq_hz - s_fll[index].freq_corr_hz;
    next_freq = Ad9833Fll_ValidFreq(next_freq);
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

    Pid_Reset(&s_fll[0].pid);
    Pid_Reset(&s_fll[1].pid);
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

    if ((!s_ready) || (!s_running) ||
        (c_frame == 0) || (a_feedback_frame == 0) || (b_feedback_frame == 0))
    {
        return;
    }

    FFT_SingleFreqDFT_U16(c_frame, len, fs_hz, s_fll[0].base_freq_hz, &ref_a);
    FFT_SingleFreqDFT_U16(c_frame, len, fs_hz, s_fll[1].base_freq_hz, &ref_b);
    FFT_SingleFreqDFT_U16(a_feedback_frame, len, fs_hz, s_fll[0].base_freq_hz, &fb_a);
    FFT_SingleFreqDFT_U16(b_feedback_frame, len, fs_hz, s_fll[1].base_freq_hz, &fb_b);

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
