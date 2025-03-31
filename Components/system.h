#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include "main.h"
#include "gpio.h"

void system_init(void);
void system_loop(void);

void usb_log(const char *fmt,...);
uint32_t get_systime_ms(void);

// 使用TIM1作为系统时间基础
#define SYSTEM_TICK_HANDLE htim1

#define LED1_ON()       HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED1_OFF()      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED2_ON()       HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED2_OFF()      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)

void handle_led_state(void);

