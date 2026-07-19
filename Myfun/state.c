#include "state.h"
#include "calculate.h"
#include "command.h"
#include "iir.h"
#include "realtime_filter.h"

#define HMI_CMD_A1 0xA1
#define HMI_CMD_A2 0xA2
#define HMI_CMD_A3 0xA3
#define HMI_CMD_A4 0xA4
#define HMI_CMD_A5 0xA5
#define HMI_CMD_A6 0xA6
#define HMI_CMD_A7 0xA7

#define STATE_DEFAULT_VOUT 1.0f
#define STATE_DEFAULT_FREQ 1000U

extern uint16_t ADC1_IN[ADC_LEN ];
extern uint16_t ADC2_OUT[ADC_LEN ] ;

typedef enum
{
    STATE_IDLE = 0,
    STATE_CHECK_HMI,
    STATE_CALC_VIN,
    STATE_CALC_LEARN,
    STATE_CALC_IIR,
} State_t;

static State_t state = STATE_IDLE;

static float vout = 0.0f;
static uint32_t freq = 0;
static float vin = 0.0f;

static uint8_t vout_ready = 0;
static uint8_t freq_ready = 0;

static uint8_t need_calculate = 0;

static void State_ProcessRealtimeFilter(void)
{
    uint8_t process_offset_valid = 0U;
    uint32_t process_offset = 0U;

    if (RealtimeFilter_IsRunning() == 0U)
    {
        return;
    }

    __disable_irq();
    if ((g_adc_mode_ctrl.iir_adc_half_ready != 0U) &&
        (g_adc_mode_ctrl.iir_dac_half_ready != 0U))
    {
        g_adc_mode_ctrl.iir_adc_half_ready = 0U;
        g_adc_mode_ctrl.iir_dac_half_ready = 0U;
        process_offset = 0U;
        process_offset_valid = 1U;
    }
    else if ((g_adc_mode_ctrl.iir_adc_full_ready != 0U) &&
             (g_adc_mode_ctrl.iir_dac_full_ready != 0U))
    {
        g_adc_mode_ctrl.iir_adc_full_ready = 0U;
        g_adc_mode_ctrl.iir_dac_full_ready = 0U;
        process_offset = ADC_LEN / 2U;
        process_offset_valid = 1U;
    }
    __enable_irq();

    if (process_offset_valid != 0U)
    {
        RealtimeFilter_ProcessHalf(process_offset);
    }
}

static void State_HandleHmiData(uint8_t head, uint32_t value)
{
    switch (head)
    {
    case HMI_CMD_A1:
        freq = (uint32_t)value;
        freq_ready = 1;
        break;

    case HMI_CMD_A2:
        vout = (float)value / 100.0f;
        vout_ready = 1;
        break;

    case HMI_CMD_A3:
        freq = (uint32_t)value;
        freq_ready = 1;
        break;

    case HMI_CMD_A4:
        vout = (float)value / 100.0f;
        vout_ready = 1;
        break;
        
    case HMI_CMD_A7:
        vout = (float)value / 100.0f;
        vout_ready = 1;
        break;

    default:
        break;
    }

    if (vout_ready && freq_ready)
    {
        need_calculate = 1;
    }
}

void State_Init(void)
{
    state = STATE_CHECK_HMI;

    vout = STATE_DEFAULT_VOUT;
    freq = STATE_DEFAULT_FREQ;

    vin = calculate_vin(vout, freq);

    vout_ready = 1;
    freq_ready = 1;
    need_calculate = 0;
}

void State_Proc(void)
{
    State_ProcessRealtimeFilter();

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

        if (hmi_a4_update_flag)
        {
            hmi_a4_update_flag = 0;
            State_HandleHmiData(HMI_CMD_A4, hmi_a4_value);
        }

        if (hmi_a5_update_flag)
        {
            hmi_a5_update_flag = 0;
            RealtimeFilter_Stop();
            App_ADC_SetMode(ADC_MODE_LEARN);
            App_ADC_ResetFlags();
            calculate_learn_start();
            state = STATE_CALC_LEARN;
        }

        if (hmi_a6_update_flag)
        {
            hmi_a6_update_flag = 0;
            state = STATE_CALC_IIR;
        }

        if (hmi_a7_update_flag)
        {
            hmi_a7_update_flag = 0;
            State_HandleHmiData(HMI_CMD_A7, hmi_a7_value);
        }

        if (need_calculate)
        {
            state = STATE_CALC_VIN;
        }
        break;

    case STATE_CALC_VIN:
        need_calculate = 0;

        vin = calculate_vin(vout, freq);
        calculate_set_ad9833_amp_by_vin(vin);
        AD9833_WaveSeting(freq, 0, SIN_WAVE, 0);

        state = STATE_CHECK_HMI;
        break;

    case STATE_CALC_LEARN:
        need_calculate = 0;

        calculate_learn_proc();

        if (get_learn_done() == 1)
        {
            printf("Learn finished, calculating IIR coefficients...\n");
            state = STATE_CHECK_HMI;
        }
        break;

    case STATE_CALC_IIR:
        need_calculate = 0;
        (void)RealtimeFilter_Start();
        state = STATE_CHECK_HMI;
        break;

    default:
        state = STATE_CHECK_HMI;
        break;
    }
}
