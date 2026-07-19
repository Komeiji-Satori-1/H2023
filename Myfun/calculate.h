#ifndef __CALCULATE_H__
#define __CALCULATE_H__

#include <stdint.h>

#include "FFT.h"


float calculate_vin(float vout, uint32_t f);
void calculate_set_ad9833_amp_by_vin(float vin);
uint8_t calculate_ad9833_amp_code(float vin);
void calculate_learn_start(void);
void calculate_learn_proc(void);
uint8_t calculate_learn_is_done(void);
uint8_t get_learn_done(void);

uint8_t calculate_iir_coeff_ready(void);

#endif /* __CALCULATE_H__ */
