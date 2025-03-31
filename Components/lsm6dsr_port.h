#ifndef LSM6DS3_PORT_H_
#define LSM6DS3_PORT_H_

#include "main.h"
#include "spi.h"
#include "lsm6dsr_reg.h"

extern int16_t acc_raw[3];     //ACC原始数据
extern int16_t gyro_raw[3];    //GYRO原始数据

#define BOOT_TIME 20 //ms
#define SENSOR_BUS hspi1

//底层接口函数实现声明
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);
//顶层初始化函数
uint8_t lsm6ds3_init(void);
void lsm6dsr_update(void);

#endif /* LSM6DS3_PORT_H_ */
