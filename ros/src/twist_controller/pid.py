import rospy
import math

MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM, pid_type=""):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx
        self.first_zero = False
        self.last_int_val = self.last_error = 0.
        self.pid_type = pid_type

    def reset(self):
        self.last_int_val = 0.0
        self.last_error = 0

    def step(self, error, sample_time):
        # Remove the accumulative integral of initial state.
        
        if error * self.last_error < 0 :
            if not self.first_zero:
                self.last_int_val = 0
                self.first_zero = True
            else:
                self.last_int_val /= 10

        integral = self.last_int_val + error * sample_time;
        #squre_integral = integral * abs(integral)
        derivative = (error - self.last_error) / sample_time;

        y = self.kp * error + self.ki * integral - self.kd * derivative;
        y = max(self.min, min(y, self.max))
        self.last_int_val = integral
        self.last_error = error
        if self.pid_type == 'angle':
            rospy.logwarn_throttle(0.4, "y: %.4f, p: %.4f, i: %.4f, d: %.4f" 
                                %(y, self.kp * error, self.ki * integral, self.kd * derivative))
        return y
