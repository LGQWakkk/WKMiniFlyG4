#include "system.h"
#include "usbd_cdc_if.h"
#include "lsm6dsr_port.h"
#include "motor.h"
#include "wk_double_tx.h"

extern uint8_t wktx_has_new_data;  //WK DOUBLETX 是否含有新的有效数据
uint8_t motor_enable = 0;          //电机使能(接收数据有效)
extern uint32_t upload_timestamp;  //上行时间戳
extern uint32_t upload_count;      //上行数据包索引
extern uint16_t motor_data[4];     //电机控制数据

#define UPLOAD_LINK_SUCCESS 0  // 上行链路正常
#define UPLOAD_LINK_LOST 1     // 上行链路丢失
#define UPLOAD_LINK_ERROR 2    // 上行链路错误
uint8_t upload_link_state = UPLOAD_LINK_LOST;//上行链路状态

//////////////////////////////////////////////USB CDC LOG//////////////////////////////////////////////
#define USB_CDC_TX_BUFFER_SIZE 64
uint8_t usb_cdc_buffer[16];
// USB CDC LOG
void usb_log(const char *fmt,...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf((char*)usb_cdc_buffer, USB_CDC_TX_BUFFER_SIZE, fmt, args);
    va_end(args);
    CDC_Transmit_FS(usb_cdc_buffer, strlen((char*)usb_cdc_buffer));
}
//////////////////////////////////////////////USB CDC LOG//////////////////////////////////////////////

//////////////////////////////////////////////SYSTEM TICK//////////////////////////////////////////////
uint8_t   tick_update    = 0;        //系统中断标志位
uint32_t  system_time_ms    = 0;        //系统时间戳(ms)
uint8_t   overloop_flag  = 0;        //此值为1表示系统循环超时 无法满足1000Hz处理频率
uint8_t   system_init_flag = 0;      //系统是否已经初始化完毕
uint16_t  tick_count     = 0;        //系统时钟计数器(0-100)

uint32_t get_systime_ms(void)
{
    return system_time_ms;
}

// 1000Hz 系统时基中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&SYSTEM_TICK_HANDLE))
    {
        if(tick_update){
            overloop_flag = 1;//loop没有来得及处理完毕就再次进入中断 发生了overloop
        }
        tick_update = 1;
        system_time_ms ++;
        tick_count ++;// tick for led blink
        if(tick_count>=1000){
            tick_count = 0;
        }
        handle_led_state();//Handle LED state
    }
}
//////////////////////////////////////////////SYSTEM TICK//////////////////////////////////////////////

// 根据模块状态更新LED状态
void handle_led_state(void)
{
    switch(upload_link_state){
        case UPLOAD_LINK_SUCCESS:
            if(tick_count==0 || tick_count==130){
                LED1_ON();
            }
            if(tick_count==30 || tick_count==160){
                LED1_OFF();
            }
            break;
        case UPLOAD_LINK_LOST:
            if(tick_count <= 500){
                LED1_ON();
            }else{
                LED1_OFF();
            }
            break;
        case UPLOAD_LINK_ERROR:
            LED1_OFF();
            break;
        default:
            LED1_OFF();
            break;
    }
}

void system_init(void)
{
    usb_log("20250309 WK MiniFly G4\r\n");
    usb_log("Init IMU LSM6DS3\r\n");
    lsm6ds3_init();
    serial_init();//启动UART DMA接收
    motor_init();
    HAL_TIM_Base_Start_IT(&SYSTEM_TICK_HANDLE);// 启动系统时基
    system_init_flag = 1;//系统初始化完毕
}

uint8_t wkfly_state_tx_count = 0;   //机体状态发送分频器
uint16_t wktx_lost_count = 0;       //通信链路断联计数器
uint16_t wktx_lost_max_count = 500; //通信链路断联最大阈值 500ms

void system_loop(void)
{
    if(tick_update && system_init_flag){
////////////////////////////////////////////////////////////////////////////////////////////
        // 判断通信链路是否正常
        if(wktx_has_new_data){
            wktx_has_new_data = 0;
            wktx_lost_count = 0;
            upload_link_state = UPLOAD_LINK_SUCCESS;//Success
            motor_enable = 1;//允许电机输出
        }else{
            wktx_lost_count ++;
            if(wktx_lost_count >= wktx_lost_max_count){//丢失连接
                wktx_lost_count = 0;
                upload_link_state = UPLOAD_LINK_LOST;//Failed
                motor_enable = 0;//禁止电机输出
            }
        }
        lsm6dsr_update();//IMU更新 1000Hz
        wkfly_state_tx_count ++;
        if(wkfly_state_tx_count >= 5){//200Hz
            wkfly_state_tx_count = 0;
            send_wkfly_data();//发送机体状态数据
            if(motor_enable){// 电机更新 200Hz
                motors_set_2048(motor_data[0], motor_data[1], motor_data[2], motor_data[3]);
            }else{//关闭电机
                motor_all_stop();
            }
        }
////////////////////////////////////////////////////////////////////////////////////////////
    tick_update = 0;//放置在处理末尾以检测overloop
    }
}
