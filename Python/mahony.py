# Mahony 六轴姿态解算
import numpy as np
from geometry import *

class Mahony:
    def __init__(self):
        self.quat = Quat(1.0, 0.0, 0.0, 0.0)
        self.Rwb = np.eye(3)
        self.Rbw = np.eye(3)
        self.euler_rad = np.zeros(3)
        self.euler_deg = np.zeros(3)

        self.exInt = 0.0
        self.eyInt = 0.0
        self.ezInt = 0.0
        self.Kp = 2.0
        self.Ki = 0.0

    def update(self, acc, gyro, dt):
        wx = gyro[0]
        wy = gyro[1]
        wz = gyro[2]
        ax = acc[0]
        ay = acc[1]
        az = acc[2]
        _q0 = self.quat.w
        _q1 = self.quat.x
        _q2 = self.quat.y
        _q3 = self.quat.z

        # 加速度单位化
        norm = np.sqrt(ax*ax + ay*ay + az*az)
        if norm < 1e-6:
            return 
        ax /= norm
        ay /= norm
        az /= norm

        vx = 2*( _q1*_q3 - _q0*_q2)
        vy = 2*( _q0*_q1 + _q2*_q3)
        vz = _q0*_q0 - _q1*_q1 - _q2*_q2 + _q3*_q3

        # 计算误差
        ex = -(ay*vz - az*vy)
        ey = -(az*vx - ax*vz)
        ez = -(ax*vy - ay*vx)

        # 误差积分
        self.exInt += ex * self.Ki
        self.eyInt += ey * self.Ki
        self.ezInt += ez * self.Ki

        # 修正角速度
        wx += self.Kp * ex + self.exInt
        wy += self.Kp * ey + self.eyInt
        wz += self.Kp * ez + self.ezInt
        
        # 四元数更新
        q0 = _q0 + ( -_q1*wx - _q2*wy - _q3*wz ) * 0.5 * dt 
        q1 = _q1 + (  _q0*wx + _q2*wz - _q3*wy ) * 0.5 * dt 
        q2 = _q2 + (  _q0*wy - _q1*wz + _q3*wx ) * 0.5 * dt 
        q3 = _q3 + (  _q0*wz + _q1*wy - _q2*wx ) * 0.5 * dt 

        # 四元数单位化
        _norm = np.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self.quat.w = q0 / _norm
        self.quat.x = q1 / _norm
        self.quat.y = q2 / _norm
        self.quat.z = q3 / _norm

        # 旋转矩阵更新
        _q0 = self.quat.w
        _q1 = self.quat.x
        _q2 = self.quat.y
        _q3 = self.quat.z
        # 从世界坐标系转换到机体坐标系
        self.Rbw = np.array(
        [
            [_q0*_q0 + _q1*_q1 - _q2*_q2 - _q3*_q3,             2.0*_q1*_q2 + 2.0*_q0*_q3,             2.0*_q1*_q3 - 2.0*_q0*_q2],
            [            2.0*_q1*_q2 - 2.0*_q0*_q3, _q0*_q0 - _q1*_q1 + _q2*_q2 - _q3*_q3,             2.0*_q2*_q3 + 2.0*_q0*_q1],
            [            2.0*_q1*_q3 + 2.0*_q0*_q2,             2.0*_q2*_q3 - 2.0*_q0*_q1, _q0*_q0 - _q1*_q1 - _q2*_q2 + _q3*_q3]
        ]
        )
        self.Rwb = self.Rbw.T  # 机体坐标系转换为世界坐标系
        # 计算欧拉角
        pitch = -np.arcsin(2.0 * (_q1*_q3 - _q0*_q2))
        roll = np.arctan2(2.0 * (_q0*_q1 + _q2*_q3), _q0 *_q0 - _q1*_q1 - _q2*_q2 + _q3*_q3)
        yaw = np.arctan2(2.0 * (_q1*_q2 + _q0*_q3), _q0 * _q0 + _q1*_q1 - _q2*_q2 - _q3*_q3)
        self.euler_rad = np.array([roll, pitch, yaw])
        self.euler_deg = np.degrees(self.euler_rad)

if __name__ == '__main__':
    print("20250322 TEST")
    mahony = Mahony()
    acc = np.array([0.0, 0.0, 9.8])
    gyro = np.array([0.0, 0.0, 1.0])
    dt = 0.01
    for i in range(100):
        mahony.update(acc, gyro, dt)
        print(mahony.quat.w, mahony.quat.x, mahony.quat.y, mahony.quat.z)
