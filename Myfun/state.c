#include "state.h"
#include "calculate.h"
#include "command.h"


#define HMI_CMD_A1 0xA1
#define HMI_CMD_A2 0xA2
#define HMI_CMD_A3 0xA3

#define STATE_DEFAULT_FREQ 1000
#define STATE_DEFAULT_VOUT 1.0f

extern uint16_t ADC_C[ADC_LEN ];


typedef enum
{
    STATE_IDLE = 0,
    STATE_CHECK_HMI,
    STATE_CALC_DES,
    STATE_CALC_PHASE,
} State_t;

static State_t state = STATE_IDLE;

static uint8_t phase_ready = 0;
uint16_t freq = STATE_DEFAULT_FREQ;
float vout = STATE_DEFAULT_VOUT;



static void State_HandleHmiData(uint8_t head, uint32_t value)
{
    switch (head)
    {
    case HMI_CMD_A1:

        break;

    case HMI_CMD_A2:

        break;

    case HMI_CMD_A3:

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


}

void State_Proc(void)
{


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
       
        }




        break;

    case STATE_CALC_PHASE:
      



        state = STATE_CHECK_HMI;
        break;

    case STATE_CALC_DES:
      


        break;


    default:
        state = STATE_CHECK_HMI;
        break;
    }
}
