# Copyright (C) 2017 Electric Movement Inc.
# All Rights Reserved.

# Author: Brandon Kinman


class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 10):
        #TODO
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_windup = max_windup

        #store relavent data
        self.last_timestamp = 0.0
        self.set_point = 0.0
        self.error_sum = 0.0
        self.last_error = 0.0
        self.alpha = 0.2
    def reset(self):
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        

        #store relavent data
        self.last_timestamp = 0.0
        self.set_point = 0.0
        self.error_sum = 0.0
        self.last_error = 0.0
    def setTarget(self, target):
        self.set_point = target

    def setKP(self, kp):
        self.kp = kp

    def setKI(self, ki):
        self.ki = ki

    def setKD(self, kd):
        self.kd = kd

    def setMaxWindup(self, max_windup):
        self.max_windup = int(max_windup)

    def update(self, measured_value, timestamp):
        delta_time = timestamp - self.last_timestamp
        if delta_time == 0:
        	return 0

        error = self.set_point - measured_value

        self.last_timestamp = timestamp

        self.error_sum = error*delta_time

        delta_error = error - self.last_error

        # Update past error
        self.last_error = error

        # Address max windup
        if self.error_sum > self.max_windup:
        	self.error_sum = self.max_windup
        elif self.error_sum < -self.max_windup:
        	self.error_sum = -self.max_windup

        p = self.kp*error
        i = self.ki*self.error_sum

        # incorporating derivative smoothing
        d = self.kd*(self.alpha * delta_error / delta_time + (1-self.alpha))

        u = p + i + d

        print("u is: ", u)

        return -u


