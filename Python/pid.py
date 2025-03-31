class PIDController:
    def __init__(self, Kp, Ki, Kd, output_limits=(None, None), integral_limits=(None, None)):
        """
        PID 控制器类 (动态 setpoint 版本)
        
        参数:
            Kp (float): 比例增益
            Ki (float): 积分增益
            Kd (float): 微分增益
            output_limits (tuple): 输出值的上下限 (默认: (None, None))
            integral_limits (tuple): 积分项的上下限 (默认: (None, None))
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        # 输出限幅
        self._min_output, self._max_output = output_limits
        # 积分限幅
        self._min_integral, self._max_integral = integral_limits
        
        self._integral = 0
        self._last_error = 0
        self._last_output = None
    
    def __call__(self, feedback_value, setpoint, dt):
        """
        计算控制输出
        
        参数:
            feedback_value (float): 当前系统的反馈值
            setpoint (float): 当前目标值
            dt (float): 距离上次调用的时间间隔(秒)
            
        返回:
            float: 控制输出
        """
        if dt <= 0:
            if self._last_output is not None:
                return self._last_output
            return 0
        
        # 计算误差
        error = setpoint - feedback_value
        
        # 比例项
        proportional = self.Kp * error
        
        # 积分项 (带积分限幅)
        self._integral += error * dt
        
        # 应用积分限幅
        if self._min_integral is not None and self._integral < self._min_integral:
            self._integral = self._min_integral
        if self._max_integral is not None and self._integral > self._max_integral:
            self._integral = self._max_integral
            
        integral = self.Ki * self._integral
        
        # 微分项
        delta_error = (error - self._last_error) / dt if dt > 0 else 0
        derivative = self.Kd * delta_error
        
        # 计算总输出
        output = proportional + integral + derivative
        
        # 保存当前状态供下次使用
        self._last_error = error
        
        # 限制输出范围
        if self._min_output is not None and output < self._min_output:
            output = self._min_output
            # 抗积分饱和: 当输出达到限幅时，停止积分累积
            self._integral = self._integral - error * dt
        elif self._max_output is not None and output > self._max_output:
            output = self._max_output
            # 抗积分饱和
            self._integral = self._integral - error * dt
        
        self._last_output = output
        return output
    
    def reset(self):
        """重置控制器状态"""
        self._integral = 0
        self._last_error = 0
        self._last_output = None
    
    def update_gains(self, Kp=None, Ki=None, Kd=None):
        """更新PID增益参数"""
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd
    
    def set_integral_limits(self, min_val=None, max_val=None):
        """设置积分限幅"""
        self._min_integral = min_val
        self._max_integral = max_val
