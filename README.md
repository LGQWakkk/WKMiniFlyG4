## WKMiniFlyG4 微型四旋翼算法开发平台

![](Hardware/pic1.jpg)

### Hardware

MCU: STM32G431CBU6

IMU: LSM6DS3 / LSM6DSR

BARO: BMP280 / SPL06

TP4056 锂电池充放管理

两侧引出UART1和I2C2

顶部引出UART3 用于连接WKDoubleTXG0作为双向通信模块

支持USB VCP串口

使用1S锂电池驱动，尽量使用高倍率低内阻电池，避免大幅度的压降导致通信不稳定。

在通信模块顶部添加电容可明显改善稳定性：

![](Hardware/pic3.jpg)

![](Hardware/pcb1_3d.png)

### Files

Hardware:  在嘉立创打样你所需的全部文件、立创EDA工程、原理图、图片等。

Python: 目前开发的双向链路通信脚本、六面椭球拟合、陀螺仪校准、Mahony姿态解算等demo，后续会不断更新。

其余为STM32CubeMX MDK工程文件。

3161554058@qq.com

