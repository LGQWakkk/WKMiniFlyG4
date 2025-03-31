# RadioMaster Pocket ELRS Radio
# BLE Joystick Mode

import pygame
import time
import threading
import numpy as np

# Axis3 Throttle -1.0~1.0
# Axis4 Yaw -1.0~1.0
# Axis1 Pitch -1.0~1.0
# Axis0 Roll -1.0~1.0

# Axis Mapping
ThrottleAxis    = 3
RollAxis        = 0
PitchAxis       = 1
YawAxis         = 4

# 摇杆数据更新频率
UPDATE_RATE = 250 # Hz
UPDATE_PERIOD = 1.0 / UPDATE_RATE

class JoystickThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.throttle = 0

    # 线程主函数
    def run(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("failed to connect joystick")
            return
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Connected to {joystick.get_name()}")

        while self.running:
            # Update Joystick
            pygame.event.get()
            self.roll       = joystick.get_axis(RollAxis)
            self.pitch      = joystick.get_axis(PitchAxis)
            self.yaw        = joystick.get_axis(YawAxis)
            self.throttle   = joystick.get_axis(ThrottleAxis)
            # 对于油门转换为归一化输入
            self.throttle = (self.throttle + 1.0) / 2.0
            time.sleep(UPDATE_PERIOD)
        print("Joystick Thread Stopped")
        pygame.quit()

    # 返回摇杆控制量数组
    def get_control(self):
        return np.array([self.roll, self.pitch, self.yaw, self.throttle])
    
    def stop(self):
        self.running = False

# TEST CODE
if __name__ == "__main__":
    joystick_thread = JoystickThread()
    joystick_thread.start()
    while True:
        control = joystick_thread.get_control()
        print(f"Roll: {control[0]}, Pitch: {control[1]}, Yaw: {control[2]}, Throttle: {control[3]}")
        time.sleep(0.01)
    joystick_thread.stop()