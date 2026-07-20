# 2026-07-19 DAC NCO implementation note

## Goal
Complete the DAC NCO path with:
- phase_inc calculation
- sine/triangle lookup tables
- half-transfer and full-transfer buffer refill

## Changes
- Added `Myfun/dac_nco.c`
- Added `Myfun/dac_nco.h`
- Connected `DacNco_Config()`, `DacNco_Start()`, and `DacNco_Stop()` in `Myfun/state.c`
- Allowed `Myfun/Waveform_classification.c` to consume an external ADC frame
- Added the new files to `MDK-ARM/23H.uvprojx`

## Result
- DAC output is driven by double-buffer DMA
- `phase_deg` can be updated at runtime
- Waveform classification can reconfigure the NCO

## Note
- Build verification was not run in Keil from this environment

## 2026-07-20 update
- Added ADC frame handoff with `adc_ready_offset` and `adc_ready_sample_start`
- Changed `State_Proc()` flow to initial separation plus continuous tracking
- Added single-frequency DFT phase measurement for the two identified frequencies
- Added NCO PLL state: `phase_ref`, `sample_ref`, `nominal_inc`, `inc_corr`, and integrator
- Moved DAC half-buffer refill work into `DacNco_Task()`, while DAC callbacks only set refill flags
- Fixed FFT window generation length from `ADC_LEN` to `FFT_LEN`
- Current sample rate constant follows the actual TIM8 setting: `240 MHz / 234 = 1025641 Hz`

## 2026-07-20 time-axis fix
- Added `DacNco_StartAtSample(start_sample)`
- Initial DFT phase is still referenced to `adc_frame_sample_start`
- DAC startup now reads the current `adc_sample_count` and starts the first DAC sample from that predicted NCO phase
- This keeps ADC DFT phase reference and DAC/NCO playback on the same sample-count time axis

## 2026-07-20 amplitude and ADC-now fix
- Added triangle-wave amplitude compensation: DFT fundamental amplitude is multiplied by `pi^2 / 8`
- Added `adc_last_boundary_offset` to record the latest ADC DMA half/full boundary
- DAC startup now estimates current ADC sample count using DMA `NDTR`, not only the coarse 1024-sample callback counter
