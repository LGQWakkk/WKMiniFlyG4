# 读取IMU数据
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq

# IMU 数据格式
# timestamp acc_x acc_y acc_z gyro_x gyro_y gyro_z
# 注意惯性数据为原始数据 int16_t
# ACC量程: 2G (scale = 0.061 mg)   GYRO量程: 2000dps(scale = 70.0 mdps)
# 时间戳单位为us

def process_raw_data(raw_data):
    acc = np.array([float(raw_data[0]), float(raw_data[1]), float(raw_data[2])]) * 0.061 / 1000.0
    gyro = np.array([float(raw_data[3]), float(raw_data[4]), float(raw_data[5])]) * 70.0 / 1000.0
    return acc, gyro

# 加载npy文件
imu_data = np.load('imu_data.npy')
# print(f"Data Shape: {imu_data.shape}")  # (2001, 7)
data_len = imu_data.shape[0]
print(f"Data Len: {data_len}")

acc_data_g = []
gyro_data_dps = []
for i in range(data_len):
    acc, gyro = process_raw_data(imu_data[i])
    acc_data_g.append(acc)
    gyro_data_dps.append(gyro)

acc_data_g = np.array(acc_data_g)
gyro_data_dps = np.array(gyro_data_dps)

# Plot
plt.subplot(2, 1, 1)
plt.plot(acc_data_g[:, 0], label='X')
plt.plot(acc_data_g[:, 1], label='Y')
plt.plot(acc_data_g[:, 2], label='Z')
plt.title('Acceleration')
plt.xlabel('Time (ms)')
plt.ylabel('Acceleration (g)')
plt.legend()
plt.subplot(2, 1, 2)
plt.plot(gyro_data_dps[:, 0], label='X')
plt.plot(gyro_data_dps[:, 1], label='Y')
plt.plot(gyro_data_dps[:, 2], label='Z')
plt.title('Angular Velocity')
plt.xlabel('Time (ms)')
plt.ylabel('Angular Velocity (dps)')
plt.legend()
plt.show()

fs = 200.0  # 200Hz

# 对陀螺仪数据进行频谱分析
gyro_data_dps_fft = gyro_data_dps[:,0]  # 修改0 1 2改为不同轴数据
gyro_fft = fft(gyro_data_dps_fft, axis=0)
gyro_freq = fftfreq(gyro_data_dps_fft.shape[0], 1/fs)
gyro_fft_amp = np.abs(gyro_fft)
gyro_fft_phase = np.angle(gyro_fft)
# Plot
plt.subplot(2, 1, 1)
plt.plot(gyro_freq, gyro_fft_amp)
plt.title('Gyro FFT')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Amplitude')
plt.subplot(2, 1, 2)
plt.plot(gyro_freq, gyro_fft_phase)
plt.xlabel('Frequency (Hz)')
plt.ylabel('Phase (rad)')
plt.show()

# 对加速度数据进行频谱分析
acc_data_g_fft = acc_data_g[:,0]  # 修改0 1 2改为不同轴数据
acc_fft = fft(acc_data_g_fft, axis=0)
acc_freq = fftfreq(acc_data_g_fft.shape[0], 1/fs)
acc_fft_amp = np.abs(acc_fft)
acc_fft_phase = np.angle(acc_fft)
# Plot
plt.subplot(2, 1, 1)
plt.plot(acc_freq, acc_fft_amp)
plt.title('Acc FFT')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Amplitude')
plt.subplot(2, 1, 2)
plt.plot(acc_freq, acc_fft_phase)
plt.xlabel('Frequency (Hz)')
plt.ylabel('Phase (rad)')
plt.show()

# 计算陀螺仪各轴平均值和方差
gyro_mean = np.mean(gyro_data_dps, axis=0)
gyro_std = np.std(gyro_data_dps, axis=0)
print(f"Gyro Mean: {gyro_mean}")
print(f"Gyro Std: {gyro_std}")

# 计算加速度各轴平均值和方差
acc_mean = np.mean(acc_data_g, axis=0)
acc_std = np.std(acc_data_g, axis=0)
print(f"Acc Mean: {acc_mean}")
print(f"Acc Std: {acc_std}")
