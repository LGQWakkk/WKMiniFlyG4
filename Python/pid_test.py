# 20250326 Wakkk PID TEST

import serial
import keyboard
from serial.tools import list_ports
import struct
import numpy as np
from mahony import Mahony
from mixer import Mixer

from pid import PIDController

# IMU校准数据文件
acc_calib_data_file = r'c:\Wakkk\Practice2024\WKMiniFlyG4\Python\acc_calib_data.txt'
gyro_calib_data_file = r'c:\Wakkk\Practice2024\WKMiniFlyG4\Python\gyro_calib_data.txt'

# 读取IMU校准数据
print("Load IMU Calibration Data")
# 读取文件内容到numpy数组
data = np.loadtxt(acc_calib_data_file)
if data.shape[0] == 6:  # OK
    print("Acc Calibration Data Loaded")
    acc_sx, acc_sy, acc_sz, acc_bx, acc_by, acc_bz = data  # 三轴比例和三轴偏置
else:
    print("Acc Calibration Data Load Failed")
data = np.loadtxt(gyro_calib_data_file)
if data.shape[0] == 3:  # OK
    print("Gyro Calibration Data Loaded")
    gyro_bx, gyro_by, gyro_bz = data  # 三轴偏置

# 去除ACC GYRO偏置和比例
def calib_imu_data(acc_raw, gyro_raw):
    acc_x = (acc_raw[0] - acc_bx) / acc_sx
    acc_y = (acc_raw[1] - acc_by) / acc_sy
    acc_z = (acc_raw[2] - acc_bz) / acc_sz
    gyro_x = (gyro_raw[0] - gyro_bx)
    gyro_y = (gyro_raw[1] - gyro_by)
    gyro_z = (gyro_raw[2] - gyro_bz)
    acc = np.array([acc_x, acc_y, acc_z])
    gyro = np.array([gyro_x, gyro_y, gyro_z])
    return acc, gyro

# 限幅函数
def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

# 打包上行数据
def upload_link_data(timestamp, count, m1, m2, m3, m4):
    # 电机输入限幅
    m1 = clamp(m1, 0, 2047)
    m2 = clamp(m2, 0, 2047)
    m3 = clamp(m3, 0, 2047)
    m4 = clamp(m4, 0, 2047)
    # 有效数据共 4+4+8 = 16Bytes
    data_packet = struct.pack('<BIIHHHHxxxxxxxxxxxxxxB', 0xD1, timestamp, count, m1, m2, m3, m4, 0x51)
    return data_packet

# 寻找可用的串口
def find_serial_port():
    available_ports = list_ports.comports()
    if not available_ports:
        print("No available serial ports found")
        return None
    first_port = available_ports[0]
    print(f"Using Serial Port: {first_port.device}")
    try:
        ser = serial.Serial(first_port.device, baudrate=256000, timeout=1)
        print("Serial Port Opened")
        return ser
    except serial.SerialException as e:
        print(f"Error Opening Serial Port: {e}")
        return None

# 解析下行飞行器数据
def parse_imu_data(data):
    if data[0] != 0xD1 or data[31] != 0x51:
        return (0,0,0,0)
    wkfly_timestamp = struct.unpack('<L', data[1:5])[0]
    wkfly_index = struct.unpack('<L', data[5:9])[0]
    acc_raw = struct.unpack('<hhh', data[9:15])
    gyro_raw = struct.unpack('<hhh', data[15:21])
    return (wkfly_timestamp, wkfly_index, acc_raw, gyro_raw)

# IMU原始数据转换以及坐标系转换
# 对于LSM6DS3TR-C ACC GYRO为左手系且ACC GYRO正方向相反
# 这里转换为右手系 x轴为机体右侧 y轴为机体前方 z轴为机体上方
def process_raw_data(acc_raw, gyro_raw):
    _acc = np.array([float(acc_raw[0]), float(acc_raw[1]), float(acc_raw[2])]) * 0.061 / 1000.0
    _gyro = np.array([float(gyro_raw[0]), float(gyro_raw[1]), float(gyro_raw[2])]) * 70.0 / 1000.0
    acc  = np.array([ -_acc[0],   _acc[1],   _acc[2]])
    gyro = np.array([ _gyro[0], -_gyro[1], -_gyro[2]])
    # gyro单位从deg/s转换为rad/s
    gyro = gyro * np.pi / 180.0
    return acc, gyro

# 构建Mahony姿态解算
mah = Mahony()
last_timestamp = None

# 构建电机动力分配器
mixer = Mixer()

# D = 0.1
# P = 1.0
# I = 0.1
pid_controller = PIDController(1.0, 0.0, 0.1, output_limits=(-1.0, 1.0), integral_limits=(-1.0, 1.0))

# 寻找可用串口
serial_port = find_serial_port()
if serial_port:
    data_packet_size = 32
    while True:
        data = serial_port.read(data_packet_size)
        if len(data) == data_packet_size:
            wkfly_timestamp, wkfly_index, acc_raw, gyro_raw = parse_imu_data(data)
            if wkfly_timestamp > 0:  # 有效IMU数据
                if last_timestamp is None:
                    last_timestamp = wkfly_timestamp
                dt = (wkfly_timestamp - last_timestamp) / 1000.0
                last_timestamp = wkfly_timestamp
                # print(f"dt: {dt*1000.0:0.3f}ms")
####################################################################################
                # 在这里编写你的控制逻辑 注意不要阻塞
                acc, gyro = process_raw_data(acc_raw, gyro_raw)
                acc, gyro = calib_imu_data(acc, gyro)  # g, deg/s
                # print(f"Acc: X {acc[0]:+05.2f}, Y {acc[1]:+05.2f} Z {acc[2]:+05.2f} Gyro: X {gyro[0]:+07.2f} Y {gyro[1]:+07.2f} Z {gyro[2]:+07.2f}")

                # Mahony Update
                mah.update(acc, gyro, dt)  # 200Hz
                _quat = mah.quat
                _Rwb = mah.Rwb.T
                _euler = mah.euler_rad

                roll = _euler[0]
                roll_control = pid_controller(roll, 0.0, dt)
                print(f"Roll: {roll:+05.2f} Roll Control: {roll_control:+05.2f}")

                # print(f"Euler: X {_euler[0]:+05.2f}, Y {_euler[1]:+05.2f} Z {_euler[2]:+05.2f}")

                pitch_control = 0
                yaw_control = 0
                thrust_control = 0.0
                motor_control = mixer.update(roll_control, pitch_control, yaw_control, thrust_control)
                # print(f"Motor: M1 {motor_control[0]}, M2 {motor_control[1]}, M3 {motor_control[2]}, M4 {motor_control[3]}")

                data_packet = upload_link_data(wkfly_timestamp, wkfly_index, motor_control[0], motor_control[1], motor_control[2], motor_control[3])
                serial_port.write(data_packet)
####################################################################################
            else:
                print("Invalid Data Received")
                serial_port.reset_input_buffer()
        else:
            print("Invalid Data Received")
            serial_port.reset_input_buffer()
        if keyboard.is_pressed('esc'):
            print("Program Ended")
            serial_port.close()
            break
