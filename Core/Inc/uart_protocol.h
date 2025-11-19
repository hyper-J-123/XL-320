#ifndef __UART_PROTOCOL_H
#define __UART_PROTOCOL_H

#include "stm32f1xx_hal.h"

#define FRAME_OVERHEAD_LENGTH 5		// 除数据部分的额外开销长度

//extern uint8_t tx_buf[10];

void data_decode(uint8_t* buf, int16_t num);
void data_encode_v2(void* data, uint8_t* buf, uint8_t func, uint16_t size);
//void sensor_data_encode_v2(sensor_data sdata, uint8_t* buf, uint8_t func, uint16_t size);
void data_decode_v2(uint8_t* buf, int16_t num);
void float_data_encode(float data, uint8_t func, uint8_t* buf);
float float_data_decode(uint8_t* data);
void uart_protocol_decode(void);
void protocol_set_sampling_rate(uint8_t freq_id);
void protocol_set_uart_bps(uint8_t bps_id);
uint16_t ID_to_sampling_rate(uint8_t freq_id);
uint8_t sampling_rate_to_ID(uint16_t freq);
uint32_t ID_to_uart_bps(uint8_t bps_id);
uint8_t uart_bps_to_ID(uint32_t bps);
uint8_t range_judgement(uint8_t data, uint8_t min, uint8_t max);

#endif
