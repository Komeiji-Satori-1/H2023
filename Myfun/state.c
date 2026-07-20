#include "state.h"
#include "calculate.h"
#include "command.h"
#include "FFT.h"
#include "Waveform_classification.h"
#include "AD9833.h"
#include "ad9833_fll.h"
#include "dac_nco.h"
#include <string.h>

#define HMI_CMD_A1 0xA1
#define HMI_CMD_A2 0xA2
#define HMI_CMD_A3 0xA3

#define STATE_DEFAULT_FREQ 1000
#define STATE_DEFAULT_VOUT 1.0f
#define STATE_PHASE_MAX_DEG 180U
#define STATE_FRAME_LEN 1024U
#define STATE_ADC_WAIT_TIMEOUT_MS 100U
#define STATE_ADC_FS_HZ 1025641.0f

extern uint16_t ADC_C[ADC_LEN];
extern volatile int32_t adc_ready_offset;
extern volatile uint64_t adc_ready_sample_start;
extern SignalInfo A;
extern SignalInfo B;

__weak uint16_t ADC_AD9833_A[ADC_LEN] = {0};
__weak uint16_t ADC_AD9833_B[ADC_LEN] = {0};
__weak volatile int32_t adc_fll_ready_offset = -1;
__weak volatile uint64_t adc_fll_ready_sample_start = 0;

typedef enum
{
    STATE_IDLE = 0,
    STATE_CHECK_HMI,
    STATE_SEPARATE_START,
    STATE_AD9833_CONFIG,
    STATE_FREQ_LOCK,
    STATE_CALC_PHASE,
} State_t;

static State_t state = STATE_IDLE;
static uint8_t separate_request = 0;
static uint8_t separate_active = 0;
static uint8_t separate_config_ready = 0;
static uint8_t phase_ready = 0;
static uint16_t phase_deg = 0;
static uint16_t adc_frame[STATE_FRAME_LEN];
static uint16_t adc_a_frame[STATE_FRAME_LEN];
static uint16_t adc_b_frame[STATE_FRAME_LEN];
static uint64_t adc_frame_sample_start = 0;

uint16_t freq = STATE_DEFAULT_FREQ;
float vout = STATE_DEFAULT_VOUT;

static uint16_t State_LimitPhase(uint32_t value)
{
    if (value > STATE_PHASE_MAX_DEG)
    {
        return STATE_PHASE_MAX_DEG;
    }

    return (uint16_t)value;
}

static uint8_t State_TakeReadyAdcFrame(uint16_t *dst,
                                       uint16_t len,
                                       uint64_t *sample_start,
                                       uint32_t timeout_ms)
{
    uint32_t tick_start;
    int32_t ready_offset;
    uint64_t ready_sample_start;

    if ((dst == NULL) || (sample_start == NULL) ||
        (len != STATE_FRAME_LEN) || (ADC_LEN < (STATE_FRAME_LEN * 2U)))
    {
        return 0;
    }

    tick_start = HAL_GetTick();

    do
    {
        __disable_irq();
        ready_offset = adc_ready_offset;
        ready_sample_start = adc_ready_sample_start;
        adc_ready_offset = -1;
        __enable_irq();

        if ((ready_offset == 0) || (ready_offset == (int32_t)STATE_FRAME_LEN))
        {
            memcpy(dst, &ADC_C[ready_offset], len * sizeof(uint16_t));
            *sample_start = ready_sample_start;
            return 1;
        }
    } while ((timeout_ms != 0U) &&
             ((HAL_GetTick() - tick_start) < timeout_ms));

    return 0;
}

static uint8_t State_CopyStableAdcFrame(uint16_t *dst, uint16_t len)
{
    return State_TakeReadyAdcFrame(dst,
                                   len,
                                   &adc_frame_sample_start,
                                   STATE_ADC_WAIT_TIMEOUT_MS);
}

static uint8_t State_TakeReadyFllFrame(uint16_t *c_dst,
                                       uint16_t *a_dst,
                                       uint16_t *b_dst,
                                       uint16_t len)
{
    int32_t ready_offset;

    if ((c_dst == NULL) || (a_dst == NULL) || (b_dst == NULL) ||
        (len != STATE_FRAME_LEN) || (ADC_LEN < (STATE_FRAME_LEN * 2U)))
    {
        return 0;
    }

    __disable_irq();
    ready_offset = adc_fll_ready_offset;
    adc_fll_ready_offset = -1;
    __enable_irq();

    if ((ready_offset != 0) && (ready_offset != (int32_t)STATE_FRAME_LEN))
    {
        return 0;
    }

    memcpy(c_dst, &ADC_C[ready_offset], len * sizeof(uint16_t));
    memcpy(a_dst, &ADC_AD9833_A[ready_offset], len * sizeof(uint16_t));
    memcpy(b_dst, &ADC_AD9833_B[ready_offset], len * sizeof(uint16_t));

    return 1;
}

static void State_MeasureDftInfo(uint16_t *frame)
{
    FFT_SingleFreqResult_t dft_a;
    FFT_SingleFreqResult_t dft_b;

    FFT_SingleFreqDFT_U16(frame, STATE_FRAME_LEN, STATE_ADC_FS_HZ, A.freq, &dft_a);
    FFT_SingleFreqDFT_U16(frame, STATE_FRAME_LEN, STATE_ADC_FS_HZ, B.freq, &dft_b);

    A.amp = dft_a.mag;
    A.phase = dft_a.phase;
    B.amp = dft_b.mag;
    B.phase = dft_b.phase;
}

static void State_StartSeparate(void)
{
    separate_request = 0;
    separate_active = 0;
    separate_config_ready = 0;
    phase_ready = 0;

    DacNco_Stop();
    Ad9833Fll_Stop();

    if (!State_CopyStableAdcFrame(adc_frame, STATE_FRAME_LEN))
    {
        return;
    }

    FFT_SetSampling(STATE_ADC_FS_HZ);
    Waveform_SetAdcFrame(adc_frame);
    test();
    State_MeasureDftInfo(adc_frame);

    if ((A.freq <= 0.0f) || (B.freq <= 0.0f))
    {
        return;
    }

    separate_config_ready = 1;
}

static void State_ConfigAd9833(void)
{
    Ad9833Fll_Config(&A, &B, phase_deg);
    Ad9833Fll_Start();

    separate_config_ready = 0;
    separate_active = 1;
}

static void State_FreqLock(void)
{
    if (!State_TakeReadyFllFrame(adc_frame,
                                 adc_a_frame,
                                 adc_b_frame,
                                 STATE_FRAME_LEN))
    {
        return;
    }

    Ad9833Fll_Task(adc_frame,
                   adc_a_frame,
                   adc_b_frame,
                   STATE_FRAME_LEN,
                   STATE_ADC_FS_HZ);
}

static void State_UpdatePhase(void)
{
    phase_ready = 0;
    Ad9833Fll_UpdatePhase(phase_deg);
}

static void State_HandleHmiData(uint8_t head, uint32_t value)
{
    switch (head)
    {
    case HMI_CMD_A1:
        separate_request = 1;
        break;

    case HMI_CMD_A2:
    case HMI_CMD_A3:
        phase_deg = State_LimitPhase(value);
        phase_ready = 1;
        break;

    default:
        break;
    }
}

void State_Init(void)
{
    state = STATE_CHECK_HMI;

    ad9833_init();

    vout = STATE_DEFAULT_VOUT;
    freq = STATE_DEFAULT_FREQ;

    separate_request = 0;
    separate_active = 0;
    separate_config_ready = 0;
    phase_ready = 0;
    phase_deg = 0;
}

void State_Proc(void)
{
    Usart_Rx_Proc();

    switch (state)
    {
    case STATE_IDLE:
        state = STATE_CHECK_HMI;
        break;

    case STATE_CHECK_HMI:
        if (hmi_a1_update_flag)
        {
            hmi_a1_update_flag = 0;
            State_HandleHmiData(HMI_CMD_A1, hmi_a1_value);
        }

        if (hmi_a2_update_flag)
        {
            hmi_a2_update_flag = 0;
            State_HandleHmiData(HMI_CMD_A2, hmi_a2_value);
        }

        if (hmi_a3_update_flag)
        {
            hmi_a3_update_flag = 0;
            State_HandleHmiData(HMI_CMD_A3, hmi_a3_value);
        }

        if (separate_request)
        {
            state = STATE_SEPARATE_START;
        }
        else if (separate_config_ready)
        {
            state = STATE_AD9833_CONFIG;
        }
        else if (separate_active && phase_ready)
        {
            state = STATE_CALC_PHASE;
        }
        else if (separate_active)
        {
            state = STATE_FREQ_LOCK;
        }
        else
        {
            phase_ready = 0;
        }

        break;

    case STATE_CALC_PHASE:
        if (separate_active && phase_ready)
        {
            State_UpdatePhase();
        }
        else
        {
            phase_ready = 0;
        }

        state = STATE_CHECK_HMI;
        break;

    case STATE_SEPARATE_START:
        State_StartSeparate();
        state = STATE_CHECK_HMI;
        break;

    case STATE_AD9833_CONFIG:
        if (separate_config_ready)
        {
            State_ConfigAd9833();
        }

        state = STATE_CHECK_HMI;
        break;

    case STATE_FREQ_LOCK:
        if (separate_request)
        {
            state = STATE_SEPARATE_START;
        }
        else
        {
            State_FreqLock();
            state = STATE_CHECK_HMI;
        }
        break;

    default:
        state = STATE_CHECK_HMI;
        break;
    }
}
