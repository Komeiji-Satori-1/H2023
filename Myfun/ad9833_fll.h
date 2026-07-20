#ifndef __AD9833_FLL_H__
#define __AD9833_FLL_H__

#include "FFT.h"
#include "Waveform_classification.h"
#include "pid.h"

typedef struct
{
    float base_freq_hz;
    float output_freq_hz;
    float freq_corr_hz;
    float phase_error_rad;
    float ref_phase_rad;
    float fb_phase_rad;
    float ref_mag;
    float fb_mag;
    uint16_t wave_type;
    uint8_t chip;
} Ad9833FllDebug_t;

void Ad9833Fll_Config(const SignalInfo *sig_a,
                      const SignalInfo *sig_b,
                      uint16_t phase_deg);
void Ad9833Fll_Start(void);
void Ad9833Fll_Stop(void);
void Ad9833Fll_UpdatePhase(uint16_t phase_deg);
void Ad9833Fll_Task(const uint16_t *c_frame,
                    const uint16_t *a_feedback_frame,
                    const uint16_t *b_feedback_frame,
                    uint16_t len,
                    float fs_hz);
uint8_t Ad9833Fll_IsRunning(void);
const Ad9833FllDebug_t *Ad9833Fll_GetDebug(uint8_t ch);

#endif /* __AD9833_FLL_H__ */
