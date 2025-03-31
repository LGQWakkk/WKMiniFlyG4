#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "usart.h"

#define DOUBLE_TX_UART_HANDLE huart3

#define RETURN_SUCCESS 0
#define RETURN_FAILURE 1

// 接收状态机状态
#define RX_WAITING_HEAD 0
#define RX_WAITING_TAIL 1
#define RX_DONE 2

#define M1_DATA_FRAME_HEAD 0xD1
#define M1_DATA_FRAME_TAIL 0x51
#define M2_DATA_FRAME_HEAD 0xD2
#define M2_DATA_FRAME_TAIL 0x52
#define ALL_DATA_FRAME_HEAD 0xD3
#define ALL_DATA_FRAME_TAIL 0x53

void serial_init(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t uart_tx_dma(uint8_t *buffer);
void clear_uart_rx_buffer(void);
uint8_t handle_serial_rx(uint8_t data);
void send_wkfly_data(void);
void receive_wkfly_data(void);
