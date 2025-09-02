#增量式pid控制器
#!/usr/bin/env python3
class pid_increase_t:
    def __init__(self, kp, ki, kd,min,max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_last = 0
        self.error_last_last = 0
        self.integral = 0
        self.min = min
        self.max = max
    def set_target(self, target):
        self.target = target
    def update(self, current_value):
        error = self.target - current_value
        # self.integral += error
        output= self.kp * error +  self.ki*self.error_last+self.kd*(error-2*self.error_last+self.error_last_last)
        self.error_last = error
        self.error_last_last = self.error_last
        if output > self.max:
            output = self.max
        elif output < self.min:
            output = self.min
        return output