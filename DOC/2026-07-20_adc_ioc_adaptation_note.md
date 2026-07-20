# 2026-07-20 ADC IOC adaptation note

## IOC review

Current `23H.ioc` uses three independent ADC peripherals with the same TIM8 trigger:

- `ADC1 / PA0 / ADC_CHANNEL_16`: C reference signal
- `ADC2 / PA2 / ADC_CHANNEL_14`: AD9833_A feedback in code
- `ADC3 / PC3_C / ADC_CHANNEL_1`: AD9833_B feedback in code

All three ADCs use:

- external trigger: `ADC_EXTERNALTRIG_T8_TRGO`
- trigger edge: rising
- one regular conversion
- DMA one-shot / normal mode

This is suitable for phase-based frequency locking, because the three ADCs are triggered by the same timer event instead of being scanned sequentially by one ADC.

## Code adaptation

- `Core/Src/main.c`
  - added strong buffers for `ADC_AD9833_A` and `ADC_AD9833_B`
  - starts ADC1/ADC2/ADC3 DMA before starting TIM8
  - calibrates ADC1/ADC2/ADC3
  - tracks half/full DMA completion with three-channel sync masks
  - publishes `adc_fll_ready_offset` only after all three ADC DMA callbacks reach the same half/full frame
  - rearms all three DMA transfers in the main loop after a complete one-shot frame

- `AD9833/AD9833.h`
  - changed bit-bang GPIO macros to use CubeMX-generated AD9833 pin names

- `AD9833/AD9833.c`
  - changed AD9833 GPIO initialization to PB12/PB13/PB14/PB15 according to the current IOC

## Important note

The current IOC does not use PA1 for an AD9833 feedback signal. It uses PA2 and PC3_C. If PA1 is required electrically, the CubeMX pin assignment must be changed before code can be mapped to PA1.
