#include "state.h"
#include "calculate.h"
#include "command.h"

#define HMI_CMD_A1 0xA1
#define HMI_CMD_A2 0xA2
#define HMI_CMD_A3 0xA3

#define STATE_DEFAULT_FREQ 1000
#define STATE_DEFAULT_VOUT 1.0f
#define STATE_PHASE_MAX_DEG 180U

extern uint16_t ADC_C[ADC_LEN];

typedef enum
{
    STATE_IDLE = 0,
    STATE_CHECK_HMI,
    STATE_CALC_DES,
    STATE_CALC_PHASE,
} State_t;

static State_t state = STATE_IDLE;

static uint8_t separate_request = 0;
static uint8_t separate_active = 0;
static uint8_t phase_ready = 0;
static uint16_t phase_deg = 0;

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

static void State_StartSeparate(void)
{
    separate_request = 0;
    separate_active = 1;
    phase_ready = 0;
    (void)phase_deg;

    /*
     * TODO:
     * 1. Copy a stable ADC frame from the DMA buffer.
     * 2. Run FFT/DFT to identify A and B.
     * 3. Generate DAC_A/DAC_B output buffers by NCO with phase_deg.
     * 4. Start DAC DMA output.
     */
}

static void State_UpdatePhase(void)
{
    phase_ready = 0;
    (void)phase_deg;

    /*
     * TODO:
     * Use phase_deg to rebuild or shift the B' DAC buffer.
     * A' is the reference phase, B' should output phase_deg degrees ahead.
     */
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

    vout = STATE_DEFAULT_VOUT;
    freq = STATE_DEFAULT_FREQ;

    separate_request = 0;
    separate_active = 0;
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
            state = STATE_CALC_DES;
        }
        else if (separate_active && phase_ready)
        {
            state = STATE_CALC_PHASE;
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

    case STATE_CALC_DES:
        if (separate_request)
        {
            State_StartSeparate();
        }

        state = STATE_CHECK_HMI;
        break;

    default:
        state = STATE_CHECK_HMI;
        break;
    }
}
