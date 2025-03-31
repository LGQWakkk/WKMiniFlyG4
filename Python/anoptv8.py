# 20250130 Wakkk 匿名助手通信协议 Python实现

import socket
import time
import struct

# 此程序通过UDP协议与匿名助手通信
# 通过修改也可通过串口进行通信

# 匿名助手IP地址和端口
DEFAULT_TARGET_IP = '10.162.17.224'  # 修改为你的IP地址
DEFAULT_TARGET_PORT = 6670
# 本客户端绑定UDP端口
DEFAULT_CLIENT_PORT = 6669

ANOPTV8_MYDEVID		= (0x01)  # 此设备ID
ANOPTV8DEVID_SWJ	= (0xFE)
ANOPTV8DEVID_ALL	= (0xFF)
ANOPTV8_HWVER		= (500)
ANOPTV8_SWVER		= (100)
ANOPTV8_BLVER		= (800)
ANOPTV8_PTVER		= (800)

ANOPTV8_FRAME_HEAD			= 0xAB
ANOPTV8_FRAME_HEADLEN 		= 6
ANOPTV8_FRAME_MAXDATALEN	= 300
ANOPTV8_FRAME_MAXFRAMELEN	= ANOPTV8_FRAME_MAXDATALEN+ANOPTV8_FRAME_HEADLEN+2

# LOG字符串颜色定义
LOG_COLOR_DEFAULT	= 0
LOG_COLOR_RED	  	= 1
LOG_COLOR_GREEN		= 2
LOG_COLOR_BLUE		= 3

# 匿名助手客户端(数据发送端)
class ANO_CLIENT:
    def __init__(self):
        self.target_ip = DEFAULT_TARGET_IP
        self.target_port = DEFAULT_TARGET_PORT
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建UDP套接字
        self.udp_socket.bind(('', DEFAULT_CLIENT_PORT))  # 绑定本地UDP端口
        self.packet_buffer = bytearray()  # 数据包构建缓冲区

    # UDP发送当前数据包
    def send(self):
        self.udp_socket.sendto(self.packet_buffer, (self.target_ip, self.target_port))
        self.packet_buffer.clear()

    # 计算并添加校验和附加校验和到最后两个字节
    def AnoPTv8CalFrameCheck(self):
        sumcheck = 0
        addcheck = 0
        for i in range(len(self.packet_buffer)):  # 此处还没有最后两个字节的校验和
            sumcheck += self.packet_buffer[i]
            addcheck += sumcheck
        # 添加校验和到最后两个字节
        self.packet_buffer.append(sumcheck % 256)
        self.packet_buffer.append(addcheck % 256)

    # 发送字符串数据包
    # daddr: uint8_t 目标设备地址
    # string_color: uint8_t 字符串颜色
    # str: 要发送的字符串
    # 函数返回数据包字节流
    def AnoPTv8SendStr(self, daddr, string_color, str):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0xA0) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _strlen = len(str)
        _data_len = _strlen + 1  # 字符串颜色占1字节
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 数据内容 首先是字符颜色 之后是字符串内容
        self.packet_buffer.append(string_color)
        self.packet_buffer.extend(str.encode('ascii'))
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()
        
    # 发送惯性传感器数据
    # acc_x, acc_y, acc_z: int16_t 加速度计数据
    # gyr_x, gyr_y, gyr_z: int16_t 陀螺仪数据
    # shock_sta: uint8_t 震动状态
    def SendInertialSensorData(self, daddr, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, shock_sta):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x01) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 13  # 加速度计数据 6Bytes + 陀螺仪数据 6Bytes + 震动状态占1字节
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 加速度计数据 6Bytes
        _packed_acc_data = struct.pack('<hhh', acc_x, acc_y, acc_z)  # 小端 int16_t
        self.packet_buffer.extend(_packed_acc_data)
        # 陀螺仪数据 6Bytes
        _packed_gyr_data = struct.pack('<hhh', gyr_x, gyr_y, gyr_z)  # 小端 int16_t
        self.packet_buffer.extend(_packed_gyr_data)
        # 震动状态 1Byte
        self.packet_buffer.append(shock_sta)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()

    # 发送罗盘 温度传感器数据
    # mag_x, mag_y, mag_z: int16_t 磁力计数据
    # tmp: int16_t 传感器温度，放大 10 倍传输， 0.1 摄氏度
    # mag_sta: uint8_t 依次为气压状态、罗盘状态
    def SendMagTempSensorData(self, daddr, mag_x, mag_y, mag_z, tmp, mag_sta):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x02) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 9
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 磁力计数据 6Bytes
        _packed_mag_data = struct.pack('<hhh', mag_x, mag_y, mag_z)  # 小端 int16_t
        self.packet_buffer.extend(_packed_mag_data)
        # 温度数据 
        _packed_tmp_data = struct.pack('<h', tmp)  # 小端 int16_t
        self.packet_buffer.extend(_packed_tmp_data)
        # 状态 1Byte
        _packed_sta_data = struct.pack('<B', mag_sta)  # 小端 uint8_t
        self.packet_buffer.extend(_packed_sta_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()

    # 发送飞控状态 三轴欧拉角
    # rol, pit, yaw: int16_t 三轴欧拉角 放大100倍
    # fusion_sta: uint8_t 融合状态
    def SendEulerAngleData(self, daddr, rol, pit, yaw, fusion_sta):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x03) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 7
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 三轴欧拉角 6Bytes
        _packed_euler_data = struct.pack('<hhh', rol, pit, yaw)  # 小端 int16_t
        self.packet_buffer.extend(_packed_euler_data)
        # 融合状态 1Byte
        _packed_fusion_data = struct.pack('<B', fusion_sta)  # 小端 uint8_t
        self.packet_buffer.extend(_packed_fusion_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()

    # 发送飞控状态 四元数
    # v0, v1, v2, v3: int16_t 四元数 放大10000倍
    # fusion_sta: uint8_t 融合状态
    def SendQuaternionData(self, daddr, v0, v1, v2, v3, fusion_sta):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x04) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 9
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 四元数 8Bytes
        _packed_quaternion_data = struct.pack('<hhhh', v0, v1, v2, v3)  # 小端 int16_t
        self.packet_buffer.extend(_packed_quaternion_data)
        # 融合状态 1Byte
        _packed_fusion_data = struct.pack('<B', fusion_sta)  # 小端 uint8_t
        self.packet_buffer.extend(_packed_fusion_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()

    # 发送高度数据
    # alt_bar: int32_t 气压计高度，单位 cm
    # alt_add: int32_t 附加高度传感高度数据，如超声波、激光测距，单位厘米
    # alt_fu: int32_t 融合后对地高度，单位厘米
    # alt_sta: uint8_t 测距状态
    def SendAltitudeData(self, daddr, alt_bar, alt_add, alt_fu, alt_sta):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x05) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 13
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 气压计高度 4Bytes
        _packed_alt_bar_data = struct.pack('<i', alt_bar)  # 小端 int32_t
        self.packet_buffer.extend(_packed_alt_bar_data)
        # 附加高度传感高度数据 4Bytes
        _packed_alt_add_data = struct.pack('<i', alt_add)  # 小端 int32_t
        self.packet_buffer.extend(_packed_alt_add_data)
        # 融合后对地高度 4Bytes
        _packed_alt_fu_data = struct.pack('<i', alt_fu)  # 小端 int32_t
        self.packet_buffer.extend(_packed_alt_fu_data)
        # 测距状态 1Byte
        _packed_alt_sta_data = struct.pack('<B', alt_sta)  # 小端 uint8_t
        self.packet_buffer.extend(_packed_alt_sta_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()

    # 发送飞控运行模式
    # mode: uint8_t 飞控模式
    # sflag: uint8_t 功能标志 0:锁定 1:解锁 2:已起飞
    # cid cmd0 cmd1: uint8_t 当前飞控执行的指令功能（指示最近的一次，完成后复位为“悬停功能”）对应指令表
    def SendModeData(self, daddr, mode, sflag, cid, cmd0, cmd1):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x06) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 5
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 飞控模式 1Byte
        _packed_mode_data = struct.pack('<B', mode)  # 小端 uint8_t
        self.packet_buffer.extend(_packed_mode_data)
        # 功能标志 1Byte
        _packed_sflag_data = struct.pack('<B', sflag)  # 小端 uint8_t
        self.packet_buffer.extend(_packed_sflag_data)
        # 当前飞控执行的指令功能 3Bytes
        _packed_cid_cmd_data = struct.pack('<BBB', cid, cmd0, cmd1)  # 小端 uint8_t
        self.packet_buffer.extend(_packed_cid_cmd_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()

    # 发送飞行速度数据
    # speed_x, speed_y, speed_z: int16_t 飞行速度 单位cm/s
    def SendSpeedData(self, daddr, speed_x, speed_y, speed_z):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x07) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 6
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 飞行速度 6Bytes
        _packed_speed_data = struct.pack('<hhh', speed_x, speed_y, speed_z)  # 小端 int16_t
        self.packet_buffer.extend(_packed_speed_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()

    # 发送位置数据(相对于原点)
    # pos_x, pos_y, pos_z: int32_t 融合之后位置坐标 相对于起飞点的偏移量 单位cm 放大100倍
    def SendPositionData(self, daddr, pos_x, pos_y, pos_z):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x08) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 12
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 位置坐标 12Bytes
        _packed_pos_data = struct.pack('<iii', pos_x, pos_y, pos_z)  # 小端 int32_t
        self.packet_buffer.extend(_packed_pos_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()

    # 发送风速估计
    # wind_x, wind_y: int16_t 风速估计 单位cm/s
    def SendWindData(self, daddr, wind_x, wind_y):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x09) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 4
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 风速估计 4Bytes
        _packed_wind_data = struct.pack('<hh', wind_x, wind_y)  # 小端 int16_t
        self.packet_buffer.extend(_packed_wind_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()

    # 发送目标姿态数据
    # tar_rol, tar_pit, tar_yaw: int16_t 目标姿态欧拉角 精确到0.01 放大100倍传输
    def SendTargetEulerAngleData(self, daddr, tar_rol, tar_pit, tar_yaw):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x0A) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 6
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 目标姿态欧拉角 6Bytes
        _packed_tar_euler_data = struct.pack('<hhh', tar_rol, tar_pit, tar_yaw)  # 小端 int16_t
        self.packet_buffer.extend(_packed_tar_euler_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()

    # 发送目标速度数据
    # tar_speed_x, tar_speed_y, tar_speed_z: int16_t 目标速度 单位cm/s
    def SendTargetSpeedData(self, daddr, tar_speed_x, tar_speed_y, tar_speed_z):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x0B) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 6
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 目标速度 6Bytes
        _packed_tar_speed_data = struct.pack('<hhh', tar_speed_x, tar_speed_y, tar_speed_z)  # 小端 int16_t
        self.packet_buffer.extend(_packed_tar_speed_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()
    
    # 发送回航信息
    # r_a: int16_t 回航角度 正负180度 放大100倍传输
    # r_d: int16_t 回航距离 单位m 
    def SendReturnHomeData(self, daddr, r_a, r_d):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x0C) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 4
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 回航角度 2Bytes
        _packed_r_a_data = struct.pack('<h', r_a)  # 小端 int16_t
        self.packet_buffer.extend(_packed_r_a_data)
        # 回航距离 2Bytes
        _packed_r_d_data = struct.pack('<h', r_d)  # 小端 int16_t
        self.packet_buffer.extend(_packed_r_d_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()
    
    # 发送电压电流数据
    # voltage: uint16_t 电压 放大100倍传输
    # current: uint16_t 电流 放大100倍传输
    def SendVoltageCurrentData(self, daddr, voltage, current):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x0D) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 4
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 电压 2Bytes
        _packed_voltage_data = struct.pack('<H', voltage)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_voltage_data)
        # 电流 2Bytes
        _packed_current_data = struct.pack('<H', current)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_current_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()

    # 发送外接模块工作状态
    # sta_g_vel: uint8_t 通用速度传感器状态
    # sta_g_pos: uint8_t 通用位置传感器状态
    # sta_gps: GPS传感器状态
    # sta_alt_add: 附加测高传感器状态
    # 传感器工作状态定义: 0:无数据 1:有数据但是不可用 2:正常工作 3:良好(GPS专用)
    def SendModuleStatusData(self, daddr, sta_g_vel, sta_g_pos, sta_gps, sta_alt_add):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x0E) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 4
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 通用速度传感器状态 1Byte
        _packed_sta_g_vel_data = struct.pack('<B', sta_g_vel)  # 小端 uint8_t
        self.packet_buffer.extend(_packed_sta_g_vel_data)
        # 通用位置传感器状态 1Byte
        _packed_sta_g_pos_data = struct.pack('<B', sta_g_pos)  # 小端 uint8_t
        self.packet_buffer.extend(_packed_sta_g_pos_data)
        # GPS传感器状态 1Byte
        _packed_sta_gps_data = struct.pack('<B', sta_gps)  # 小端 uint8_t
        self.packet_buffer.extend(_packed_sta_gps_data)
        # 附加测高传感器状态 1Byte
        _packed_sta_alt_add_data = struct.pack('<B', sta_alt_add)  # 小端 uint8_t
        self.packet_buffer.extend(_packed_sta_alt_add_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()

    # 发送RGB亮度信息输出
    # bri_r, bri_g, bri_b: uint8_t 红绿蓝灯亮度 0~255
    # bri_a: uint8_t 单独LED亮度 有效范围0-20 0最暗 20最亮
    def SendRGBBrightnessData(self, daddr, bri_r, bri_g, bri_b, bri_a):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x0F) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 4
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 红绿蓝灯亮度 3Bytes
        _packed_bri_data = struct.pack('<BBB', bri_r, bri_g, bri_b)  # 小端 uint8_t
        self.packet_buffer.extend(_packed_bri_data)
        # 单独LED亮度 1Byte
        _packed_bri_a_data = struct.pack('<B', bri_a)  # 小端 uint8_t
        self.packet_buffer.extend(_packed_bri_a_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()
    
######################################飞控控制量输出类######################################
    # 发送PWM控制量
    # pwm1-pwm8: uint16_t 8个通道PWM控制量 0-10000 默认4轴 单位0.01%油门
    def SendPWMData(self, daddr, pwm1, pwm2, pwm3, pwm4, pwm5, pwm6, pwm7, pwm8):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x20) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 16
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        # 8个通道PWM控制量 16Bytes
        _packed_pwm_data = struct.pack('<HHHHHHHH', pwm1, pwm2, pwm3, pwm4, pwm5, pwm6, pwm7, pwm8)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_pwm_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()

    # 发送姿态控制量
    # ctrl_rol, ctrl_pit, ctrl_thr, ctrl_yaw: int16_t 
    # ctrl_rol, ctrl_pit, ctrl_yaw 控制量范围-5000 ~ +5000
    # ctrl_thr 油门控制量为0~10000
    # Warning: 这个数据定义和上位机顺序不一致!!! 上位机的ctrl_thr和ctrl_yaw的顺序反了!!!
    def SendAttitudeControlData(self, daddr, ctrl_rol, ctrl_pit, ctrl_thr, ctrl_yaw):
        # 清空缓冲区
        self.packet_buffer.clear()
        self.packet_buffer.append(ANOPTV8_FRAME_HEAD)
        self.packet_buffer.append(ANOPTV8_MYDEVID)
        self.packet_buffer.append(daddr)
        self.packet_buffer.append(0x21) # Frame ID
        # 数据长度 (数据内容部分长度) 2Bytes
        _data_len = 8
        _packed_data_len = struct.pack('<H', _data_len)  # 小端 uint16_t
        self.packet_buffer.extend(_packed_data_len)
        _packed_ctrl_data = struct.pack('<hhhh', ctrl_rol, ctrl_pit, ctrl_thr, ctrl_yaw)  # 小端 int16_t
        self.packet_buffer.extend(_packed_ctrl_data)
        self.AnoPTv8CalFrameCheck()  # 计算帧检验和
        # 发送数据包
        self.send()


# 发送测试 请忽略
if __name__ == '__main__':
    _send_daddr = 0x01
    # 匿名助手客户端
    client = ANO_CLIENT()
    while True:
        # client.AnoPTv8SendStr(_send_daddr, LOG_COLOR_GREEN, _send_str)
        # client.SendInertialSensorData(_send_daddr, 100, 200, 300, 400, 500, 600, 0)
        # client.SendMagTempSensorData(_send_daddr, 100, 200, 300, 400, 0x01)
        # client.SendEulerAngleData(_send_daddr, 100, 200, 300, 0x01)
        # client.SendQuaternionData(_send_daddr, 100, 200, 300, 400, 0x01)
        # client.SendAltitudeData(_send_daddr, 10000, 20000, 30000, 0x01)
        # client.SendSpeedData(_send_daddr, 1, 2, 3)
        # client.SendPositionData(_send_daddr, 100, 200, 300)
        # client.SendWindData(_send_daddr, 100, 200)
        # client.SendTargetEulerAngleData(_send_daddr, 10, 20, 30)
        # client.SendTargetSpeedData(_send_daddr, 1, 2, 3)
        # client.SendReturnHomeData(_send_daddr, 100, 200)
        # client.SendVoltageCurrentData(_send_daddr, 100, 200)
        # client.SendPWMData(_send_daddr, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000)
        # client.SendAttitudeControlData(_send_daddr, 1000, 2000, 3000, 4000)
        time.sleep(0.1)
