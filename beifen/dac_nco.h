#ifndef __DAC_NCO_H__
#define __DAC_NCO_H__

#include "main.h"
#include "Waveform_classification.h"

void DacNco_Config(const SignalInfo *A,
                   const SignalInfo *B,
                   uint16_t phase_deg);
void DacNco_Start(void);
void DacNco_StartAtSample(uint64_t start_sample);
void DacNco_Stop(void);
void DacNco_UpdatePhase(uint16_t phase_deg);
void DacNco_UpdateComponent(uint8_t ch, const SignalInfo *sig);
void DacNco_SetReferenceSample(uint64_t sample_ref);
void DacNco_PllUpdate(uint8_t ch, float measured_phase_rad, uint64_t frame_start_sample);
void DacNco_Task(void);

#endif /* __DAC_NCO_H__ */
