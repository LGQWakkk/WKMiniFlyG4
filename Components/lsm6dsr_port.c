#include "lsm6dsr_port.h"
#include "main.h"
#include "usart.h"
#include "system.h"

// 原始数据
int16_t acc_raw[3];         //ACC原始数据
int16_t gyro_raw[3];        //GYRO原始数据

int16_t temp_raw;           //Temperature原始数据
uint32_t timestamp_raw;     //TimeStamp原始数据
uint32_t frame_index = 0;   //Index IMU数据索引
stmdev_ctx_t dev_ctx;       //设备控制句柄

// 单位转换后数据
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;//IMU温度数据

//底层接口函数实现
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(handle, bufp, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    reg |= 0x80;
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(handle, bufp, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return 0;
}

static void platform_delay(uint32_t ms){HAL_Delay(ms);}

// IMU初始化
uint8_t lsm6ds3_init(void)
{
    uint8_t id = 0;
    uint8_t rst = 0;
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = &SENSOR_BUS;
    platform_delay(BOOT_TIME);
    lsm6dsr_device_id_get(&dev_ctx, &id);
    // 经过实际测试IMU ID为0x6A  是LSM6DS3TR-C 属于LSM6DS3系列
    lsm6dsr_reset_set(&dev_ctx, PROPERTY_ENABLE);
    HAL_Delay(200);

    lsm6dsr_i3c_disable_set(&dev_ctx, LSM6DSR_I3C_DISABLE);   /* Disable I3C interface */
    lsm6dsr_block_data_update_set(&dev_ctx, PROPERTY_ENABLE); /*  Enable Block Data Update */
    // Full Scale
    lsm6dsr_xl_full_scale_set(&dev_ctx, LSM6DSR_2g);          //2G
    lsm6dsr_gy_full_scale_set(&dev_ctx, LSM6DSR_2000dps);     //2000dps 12 OK
    // Data Rate
    lsm6dsr_xl_data_rate_set(&dev_ctx, LSM6DSR_XL_ODR_1666Hz);
    lsm6dsr_gy_data_rate_set(&dev_ctx, LSM6DSR_GY_ODR_1666Hz);
    // Acc Filter
    lsm6dsr_xl_hp_path_on_out_set(&dev_ctx, LSM6DSR_LP_ODR_DIV_100);
    lsm6dsr_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);
    // Gyro Filter
    lsm6dsr_gy_filter_lp1_set(&dev_ctx, PROPERTY_ENABLE);
    lsm6dsr_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSR_ULTRA_LIGHT);
    // Timestamp
    lsm6dsr_timestamp_set(&dev_ctx, PROPERTY_ENABLE);//Enable Timestamp
    lsm6dsr_timestamp_raw_get(&dev_ctx, &timestamp_raw);//Get the Current Timestamp

    return id;//返回识别到的ID
}

// IMU数据更新
void lsm6dsr_update(void)
{   
    lsm6dsr_reg_t reg;
    lsm6dsr_status_reg_get(&dev_ctx, &reg.status_reg);/* Read output only if new value is available */
    //get timestamp
    if (reg.status_reg.xlda || reg.status_reg.gda || reg.status_reg.tda) {
        lsm6dsr_timestamp_raw_get(&dev_ctx, &timestamp_raw);
    }
    if (reg.status_reg.xlda)//加速度计数据
    {
        memset(acc_raw, 0x00, 3 * sizeof(int16_t));
        lsm6dsr_acceleration_raw_get(&dev_ctx, acc_raw);
        acceleration_mg[0] = lsm6dsr_from_fs2g_to_mg(acc_raw[0]);
        acceleration_mg[1] = lsm6dsr_from_fs2g_to_mg(acc_raw[1]);
        acceleration_mg[2] = lsm6dsr_from_fs2g_to_mg(acc_raw[2]);
    }
    if (reg.status_reg.gda)//陀螺仪数据
    {
        memset(gyro_raw, 0x00, 3 * sizeof(int16_t));
        lsm6dsr_angular_rate_raw_get(&dev_ctx, gyro_raw);
        angular_rate_mdps[0] =lsm6dsr_from_fs2000dps_to_mdps(gyro_raw[0]);
        angular_rate_mdps[1] =lsm6dsr_from_fs2000dps_to_mdps(gyro_raw[1]);
        angular_rate_mdps[2] =lsm6dsr_from_fs2000dps_to_mdps(gyro_raw[2]);
    }
    if (reg.status_reg.tda)//温度数据
    {
        memset(&temp_raw, 0x00, sizeof(int16_t));
        lsm6dsr_temperature_raw_get(&dev_ctx, &temp_raw);
        temperature_degC = lsm6dsr_from_lsb_to_celsius(temp_raw);
    }
}
