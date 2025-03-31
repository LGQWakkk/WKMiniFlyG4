#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "tim.h"

#define MOTOR_TIM_HANDLE htim2
#define M1_TIM_CHANNEL TIM_CHANNEL_1
#define M2_TIM_CHANNEL TIM_CHANNEL_2
#define M3_TIM_CHANNEL TIM_CHANNEL_3
#define M4_TIM_CHANNEL TIM_CHANNEL_4

#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4

// 和大部分航模遥控器通道数值范围一致
#define MOTOR_PWM_ARR_VALUE (2048-1)

// Functions
void motor_init(void);
void motor_set_duty(uint8_t index, float duty);
void motor_all_stop(void);
void motor_set_2048(uint16_t index, uint16_t duty);
void motors_set_2048(uint16_t duty1, uint16_t duty2, uint16_t duty3, uint16_t duty4);
