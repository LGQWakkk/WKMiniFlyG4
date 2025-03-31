# 电机动力分配器
import numpy as np

# 电机分布:
# 4 2 
# 3 1

class Mixer:
    def __init__(self):
        self.motor_max = 2047  # 电机最大控制量
        self.dead_zone = 30.0  # 小于此控制量电机不转

    # roll, pitch, yaw: 三轴扭矩输入 归一化输入 [-1, 1]
    # thrust: 推力输入 归一化输入 [0, 1]
    # 输出: 电机控制量0-2047
    def update(self, roll, pitch, yaw, thrust):
        # 输入限幅
        roll = max(min(roll, 1), -1)
        pitch = max(min(pitch, 1), -1)
        yaw = max(min(yaw, 1), -1)
        thrust = max(min(thrust, 1), 0)
        # 电机分配
        motor1 = int(self.motor_max * (-roll - pitch + yaw + thrust))
        motor2 = int(self.motor_max * ( roll - pitch - yaw + thrust))
        motor3 = int(self.motor_max * (-roll + pitch - yaw + thrust))
        motor4 = int(self.motor_max * ( roll + pitch + yaw + thrust))
        # 电机限幅
        motor1 = max(min(motor1, self.motor_max), 0)
        motor2 = max(min(motor2, self.motor_max), 0)
        motor3 = max(min(motor3, self.motor_max), 0)
        motor4 = max(min(motor4, self.motor_max), 0)
        # 死区处理
        if abs(motor1) < self.dead_zone:
            motor1 = 0
        if abs(motor2) < self.dead_zone:
            motor2 = 0
        if abs(motor3) < self.dead_zone:
            motor3 = 0
        if abs(motor4) < self.dead_zone:
            motor4 = 0
        # return motor1, motor2, motor3, motor4
        return np.array([motor1, motor2, motor3, motor4])

if __name__ == '__main__':
    mixer = Mixer()
    print(mixer.update(0.0, 0.0, 0.1, 0.0))
