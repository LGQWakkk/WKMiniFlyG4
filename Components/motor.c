#include "motor.h"

void motor_init(void)
{
    motor_all_stop();
    HAL_TIM_Base_Start(&MOTOR_TIM_HANDLE);
    HAL_TIM_PWM_Start(&MOTOR_TIM_HANDLE, M1_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&MOTOR_TIM_HANDLE, M2_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&MOTOR_TIM_HANDLE, M3_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&MOTOR_TIM_HANDLE, M4_TIM_CHANNEL);
}

// 设置单个电机功率(占空比)
// index: 1-4, 对应四个电机
// duty: 0.0~1.0 功率
void motor_set_duty(uint8_t index, float duty)
{
    if(duty < 0.0){
        duty = 0.0;
    }else if(duty > 1.0){
        duty = 1.0;
    }
    uint16_t _duty = (uint16_t)(duty * MOTOR_PWM_ARR_VALUE);
    
    switch (index)
    {
    case MOTOR1:
        __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M1_TIM_CHANNEL, _duty);
        break;
    case MOTOR2:
        __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M2_TIM_CHANNEL, _duty);
        break;
    case MOTOR3:
        __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M3_TIM_CHANNEL, _duty);
        break;
    case MOTOR4:
        __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M4_TIM_CHANNEL, _duty);
        break;
    default:
        break;
    }
}

// 立即停止所有电机
void motor_all_stop(void)
{
    __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M1_TIM_CHANNEL, 0);
    __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M2_TIM_CHANNEL, 0);
    __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M3_TIM_CHANNEL, 0);
    __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M4_TIM_CHANNEL, 0);
}

// 设置uint16_t 类型电机功率 0-2047
void motor_set_2048(uint16_t index, uint16_t duty)
{
    if(duty > 2047){
        duty = 2047;
    }
    switch (index)
    {
    case MOTOR1:
        __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M1_TIM_CHANNEL, duty);
        break;
    case MOTOR2:
        __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M2_TIM_CHANNEL, duty);
        break;
    case MOTOR3:
        __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M3_TIM_CHANNEL, duty);
        break;
    case MOTOR4:
        __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M4_TIM_CHANNEL, duty);
        break;
    default:
        break;
    }
}

// 同时设置四个电机功率
void motors_set_2048(uint16_t duty1, uint16_t duty2, uint16_t duty3, uint16_t duty4)
{
    if(duty1 > 2047){duty1 = 2047;}
    if(duty2 > 2047){duty2 = 2047;}
    if(duty3 > 2047){duty3 = 2047;}
    if(duty4 > 2047){duty4 = 2047;}
    __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M1_TIM_CHANNEL, duty1);
    __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M2_TIM_CHANNEL, duty2);
    __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M3_TIM_CHANNEL, duty3);
    __HAL_TIM_SetCompare(&MOTOR_TIM_HANDLE, M4_TIM_CHANNEL, duty4);
}
