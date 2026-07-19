#ifndef __COMMAND_H__
#define __COMMAND_H__

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <string.h>
#include "usart.h"
#include "main.h"

extern volatile uint8_t hmi_a1_update_flag;
extern volatile uint8_t hmi_a2_update_flag;
extern volatile uint8_t hmi_a3_update_flag;
extern volatile uint8_t hmi_a4_update_flag;
extern volatile uint8_t hmi_a5_update_flag;
extern volatile uint8_t hmi_a6_update_flag;
extern volatile uint8_t hmi_a7_update_flag;

extern volatile uint32_t hmi_a1_value;
extern volatile uint32_t hmi_a2_value;
extern volatile uint32_t hmi_a3_value;
extern volatile uint32_t hmi_a4_value;
extern volatile uint32_t hmi_a5_value;
extern volatile uint32_t hmi_a6_value;
extern volatile uint32_t hmi_a7_value;

void Usart_Send_Computer(UART_HandleTypeDef huart, char *msg);
void My_Usart_Init(void);
void Usart_Rx_Proc(void);

#endif