# ACC GYRO 校准程序
# 20250322 Wakkk

import serial
import keyboard
from serial.tools import list_ports
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import leastsq

acc_calib_num = 500    # 用于加速度计校准的单面采样次数
gyro_calib_num = 500   # 用于陀螺仪校准的采样次数

# 加速度计校准参数初步估计:(三轴比例和三轴偏置 单位为g)
acc_calib_guess = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0]

# 椭球方程
def ellipsoid_eq(params, x, y, z):
    x_scale, y_scale, z_scale, x_bias, y_bias, z_bias = params  # 三轴比例和三轴偏置
    return ((x - x_bias)/x_scale)**2 + ((y - y_bias)/y_scale)**2 + ((z - z_bias)/z_scale)**2 - 1

# 椭球残差计算
def residuals(params, x, y, z):
    return ellipsoid_eq(params, x, y, z)

# 寻找可用的串口
def find_serial_port():
    available_ports = list_ports.comports()
    if not available_ports:
        print("No available serial ports found")
        return None
    first_port = available_ports[0]
    print(f"Using Serial Port: {first_port.device}")
    try:
        ser = serial.Serial(first_port.device, baudrate=256000, timeout=2)
        print("Serial Port Opened")
        return ser
    except serial.SerialException as e:
        print(f"Error Opening Serial Port: {e}")
        return None

# 解析IMU数据包
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

# 获取面索引
# acc_g: 三轴加速度(g为单位)
# return: 0-5 表示6个方向
def get_primary_axis_index(acc_g):
    abs_x = abs(acc_g[0])
    abs_y = abs(acc_g[1])
    abs_z = abs(acc_g[2])
    if abs_z / 1.5 > abs_x and abs_z / 1.5 > abs_y:
        # Z轴
        return 0 if acc_g[2] > 0 else 1
    elif abs_x / 1.5 > abs_y and abs_x / 1.5 > abs_z:
        # X轴
        return 2 if acc_g[0] > 0 else 3
    elif abs_y / 1.5 > abs_x and abs_y / 1.5 > abs_z:
        # Y轴
        return 4 if acc_g[1] > 0 else 5
    else:
        return -1

acc_calib_tmp = []   # 用于拟合的加速度计校准面数据
acc_calib_count = 0  # 已采集面数
acc_calib_data = []  # 加速度计校准数据
gyro_calib_data = []  # 陀螺仪校准数据
calib_state = 'wait-start'  # 校准状态记录

# 寻找可用串口
serial_port = find_serial_port()
if serial_port:
    data_packet_size = 32
    while True:
        data = serial_port.read(data_packet_size)
        if len(data) == data_packet_size:
            wkfly_timestamp, wkfly_index, acc_raw, gyro_raw = parse_imu_data(data)
            if wkfly_timestamp > 0:  # 有效IMU数据
                acc, gyro = process_raw_data(acc_raw, gyro_raw)  # g, rad/s
                primary_axis_index = get_primary_axis_index(acc)

                if calib_state == 'wait-start':
                    if keyboard.is_pressed('enter'):  # 按下Enter开始校准
                        calib_state = 'gyro-calib'
                        print("Start Gyro Calibration")
                if calib_state == 'gyro-calib':
                    gyro_calib_data.append(gyro)
                    if len(gyro_calib_data) >= gyro_calib_num:  # 采集足够数据进行陀螺仪校准
                        gyro_calib_data = np.array(gyro_calib_data)
                        calib_state = 'gyro-done'
                        print("Gyro Calibration Finished")
                        # 计算均值和方差
                        gyro_mean = np.mean(gyro_calib_data, axis=0)
                        gyro_std = np.std(gyro_calib_data, axis=0)
                        print(f"Gyro Mean: X:{gyro_mean[0]:.3f} Y:{gyro_mean[1]:.3f} Z:{gyro_mean[2]:.3f}")
                        print(f"Gyro Std: {gyro_std}")
                        gyro_calib_data = gyro_mean  # 仅仅保存均值

                if calib_state == 'gyro-done':
                    if keyboard.is_pressed('enter'):  # 按下Enter开始下一轮陀螺仪校准
                        print("Start Acc Calibration")
                        calib_state = 'acc-calib'

                if calib_state == 'acc-calib':
                    acc_calib_tmp.append(acc)
                    if len(acc_calib_tmp) >= acc_calib_num:  # 采集足够数据进行加速度计校准
                        acc_calib_tmp = np.array(acc_calib_tmp)
                        acc_calib_count += 1
                        print(f"Acc Calibration {acc_calib_count} Finished")
                        calib_state = 'acc-done'
                        # 计算均值和方差
                        acc_mean = np.mean(acc_calib_tmp, axis=0)
                        acc_std = np.std(acc_calib_tmp, axis=0)
                        print(f"Acc Mean: X:{acc_mean[0]:.3f} Y:{acc_mean[1]:.3f} Z:{acc_mean[2]:.3f}")
                        print(f"Acc Std: {acc_std}")
                        # 保存单面校准数据
                        acc_calib_data.append(acc_mean)
                        acc_calib_tmp = []
                
                if calib_state == 'acc-done':
                    if keyboard.is_pressed('enter'):  # 按下Enter开始下一轮加速度计校准
                        print("Start Acc Calibration")
                        calib_state = 'acc-calib'

            else:
                print("Invalid Data Received")
                serial_port.reset_input_buffer()
        else:
            print("Invalid Data Received")
            serial_port.reset_input_buffer()
        if keyboard.is_pressed('esc'):
            print("Calib Done")
            serial_port.close()
            # 陀螺仪偏置校准数据
            gyro_calib_data = np.array(gyro_calib_data)
            # 所有面的校准数据
            acc_calib_data = np.array(acc_calib_data)
            # 处理加速度计校准数据拟合
            result, ier = leastsq(residuals, acc_calib_guess, args=(acc_calib_data[:,0], acc_calib_data[:,1], acc_calib_data[:,2]))
            x_scale, y_scale, z_scale, x_bias, y_bias, z_bias = result
            # 输出三轴比例和三轴偏置
            print(f"Acc Calibration Result: Sx = {x_scale}, Sy = {y_scale}, Sz = {z_scale}, Bx = {x_bias}, By = {y_bias}, Bz = {z_bias}")
            # 绘制拟合情况
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
            # 绘制采样面点
            ax.scatter(acc_calib_data[:,0], acc_calib_data[:,1], acc_calib_data[:,2], c='r', marker='.', s=10.0, label='Data Points')
            u = np.linspace(0, 2 * np.pi, 100)
            v = np.linspace(0, np.pi, 100)
            x = x_scale * np.outer(np.cos(u), np.sin(v)) + x_bias
            y = y_scale * np.outer(np.sin(u), np.sin(v)) + y_bias
            z = z_scale * np.outer(np.ones_like(u), np.cos(v)) + z_bias
            ax.plot_surface(x, y, z, color='b', alpha=0.2, label='Fitted Ellipsoid')
            ax.set_xlabel('X Axis')
            ax.set_ylabel('Y Axis')
            ax.set_zlabel('Z Axis')
            ax.legend()
            plt.show()
            # 保存校准参数到txt
            np.savetxt('acc_calib_data.txt', [x_scale, y_scale, z_scale, x_bias, y_bias, z_bias])
            # 保存陀螺仪偏置校准数据到txt
            np.savetxt('gyro_calib_data.txt', gyro_calib_data)
            break
