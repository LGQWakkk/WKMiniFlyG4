#include "wk_double_tx.h"
#include "lsm6dsr_port.h"
#include "system.h"

uint8_t uart_dma_buf[1] = {0};   //DMA单字节接收缓冲区
// RX BUFFER
#define UART_RX_BUFFER_SIZE 32   // 所有数据包长度均为32字节
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart_rx_buffer_ptr = 0;
// TX BUFFER
#define UART_TX_BUFFER_SIZE 32   // 所有数据包长度均为32字节
static uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
static uint8_t is_uart_tx_dma_sending = 0;//UART DMA是否正在发送数据
// RX STATE
static uint8_t uart_rx_state = RX_WAITING_HEAD;

static uint8_t wkfly_data[32] = {0};//机体数据发送
static uint32_t wkfly_data_count = 0;//机体数据包计数

// 解析出的上行控制数据包
uint32_t upload_timestamp = 0;  //上行时间戳
uint32_t upload_count = 0;      //上行数据包索引
uint16_t motor_data[4];         //电机控制数据

// 是否有新的有效数据包收到(用于检测通信失败)
uint8_t wktx_has_new_data = 0;

// 串口发送接收初始化
void serial_init(void)
{
    // 开始DMA接收
    HAL_UART_Receive_DMA(&DOUBLE_TX_UART_HANDLE, uart_dma_buf, 1);
}

// UART串口DMA接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &DOUBLE_TX_UART_HANDLE)
    {
        // 调用单字节接收处理回调函数
        handle_serial_rx(uart_dma_buf[0]);
        uart_dma_buf[0] = 0;
        HAL_UART_Receive_DMA(&DOUBLE_TX_UART_HANDLE, uart_dma_buf, 1); //重新准备接收
    }
}

// UART串口DMA发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &DOUBLE_TX_UART_HANDLE)
    {
        is_uart_tx_dma_sending = 0;//reset
    }
}

// 串口DMA发送数据
// RETURN SUCCESS: DMA空闲 开始发送
// RETURN FAILURE: DMA忙 等待发送完成 本次发送无效
uint8_t uart_tx_dma(uint8_t *buffer)
{
    if(is_uart_tx_dma_sending == 1){
        return RETURN_FAILURE;//DMA忙 等待发送完成 本次发送无效
    }else{
        for(uint8_t i=0; i<32; i++){
            uart_tx_buffer[i] = buffer[i];//Copy
        }
        HAL_UART_Transmit_DMA(&DOUBLE_TX_UART_HANDLE, uart_tx_buffer, 32);
        is_uart_tx_dma_sending = 1;
        return RETURN_SUCCESS;//DMA空闲 开始发送
    }
}

// 清空串口接收缓冲区
void clear_uart_rx_buffer(void)
{
    for(uint8_t i=0; i<UART_RX_BUFFER_SIZE; i++){
        uart_rx_buffer[i] = 0;
    }
    uart_rx_buffer_ptr = 0;
}

// 单字节串口接收回调函数
uint8_t handle_serial_rx(uint8_t data)
{
    switch (uart_rx_state)
    {
    case RX_WAITING_HEAD://等待数据包帧头
        if(data == M1_DATA_FRAME_HEAD || data == M2_DATA_FRAME_HEAD || data == ALL_DATA_FRAME_HEAD)
        {
            clear_uart_rx_buffer();
            uart_rx_buffer[uart_rx_buffer_ptr] = data;
            uart_rx_buffer_ptr++;
            uart_rx_state = RX_WAITING_TAIL;
        }
        break;
    case RX_WAITING_TAIL://等待数据包帧尾
        if((uart_rx_buffer_ptr == 31) && (data == M1_DATA_FRAME_TAIL || data == M2_DATA_FRAME_TAIL || data == ALL_DATA_FRAME_TAIL))
        {
            uart_rx_buffer[uart_rx_buffer_ptr] = data;//帧尾
            uart_rx_buffer_ptr ++;
            // 接收完毕 处理数据包
            if(uart_rx_buffer[0] == M1_DATA_FRAME_HEAD || uart_rx_buffer[0] == M2_DATA_FRAME_HEAD || uart_rx_buffer[0] == ALL_DATA_FRAME_HEAD){
                // 处理数据包 此处表示收到有效数据
                // 解析电机数据
                wktx_has_new_data = 1;
                receive_wkfly_data();
            }else{
                // 错误数据包
            }
            clear_uart_rx_buffer();
            uart_rx_state = RX_WAITING_HEAD;//复位
        }else{//还没有收到帧尾
            if(uart_rx_buffer_ptr >= (UART_RX_BUFFER_SIZE-1)){//本应该接收到帧尾 但是没有 接收错误
                clear_uart_rx_buffer();
                uart_rx_state = RX_WAITING_HEAD;//重新准备接收
            }else{//接收过程中 保存数据到缓冲区
                uart_rx_buffer[uart_rx_buffer_ptr] = data;
                uart_rx_buffer_ptr ++;
            }
        }
    default:
        break;
    }
    return RETURN_SUCCESS;
}

// 发送数据包 下行数据
// 数据格式:
// 帧头帧尾: WKDOUBLETX M1发送帧头帧尾
// uint32_t 系统时间戳 ms
// uint32_t 系统发送数据包计数
// int16_t *3 acc raw
// int16_t *3 gyro raw
void send_wkfly_data(void)
{
    wkfly_data[0] = M1_DATA_FRAME_HEAD;//0xD1
    wkfly_data[31] = M1_DATA_FRAME_TAIL;//0x51
    uint32_t time_ms = get_systime_ms();// 系统时间戳
    memcpy(&wkfly_data[1], &time_ms, 4);
    memcpy(&wkfly_data[5], &wkfly_data_count, 4);
    memcpy(&wkfly_data[9], acc_raw, 6);
    memcpy(&wkfly_data[15], gyro_raw, 6);
    // 发送数据
    uart_tx_dma(wkfly_data);
    wkfly_data_count ++;
}

// 从接收缓冲区中解析电机控制数据包
// 缓冲区: uart_rx_buffer
void receive_wkfly_data(void)
{
    memcpy(&upload_timestamp, &uart_rx_buffer[1], 4);// 解析时间戳
    memcpy(&upload_count, &uart_rx_buffer[5], 4);// 解析数据包计数
    memcpy(motor_data, &uart_rx_buffer[9], 8);// 解析电机控制数据
}
