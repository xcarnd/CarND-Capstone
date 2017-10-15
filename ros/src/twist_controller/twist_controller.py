#*_coding:utf-8 _*
import math
import rospy
from pid import PID 
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, brake_deadband, decel_limit, accel_limit, wheel_radius,
                    wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.brake_deadband = brake_deadband
        self.decel_limit    = decel_limit
        self.accel_limit    = accel_limit
        self.steer_ratio = steer_ratio
        # Is pid controller need to init.
        self.need_init = True
        # max and min speed, m/s
        min_speed = 0.001       
        max_speed = 120 / 3.6   
        # TODO: find the best pid parameters.
        speed_kp, speed_ki, speed_kd = 1, 0, -0.2
        steer_kp, steer_ki, steer_kd = 1, 0, -0.2
        self.pid_speed = PID(speed_kp, speed_ki, speed_kd, mn=decel_limit, mx=accel_limit)
        self.pid_steer = PID(steer_kp, steer_ki, steer_kd, mn=-22*math.pi/180, mx=22*math.pi/180)

        self.yaw_ctl = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        # For interal debug.
        self.debug_dict = {}

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)  

    def vector_length(self, v):
        return math.sqrt(v.x**2 + v.y**2 + v.z**2)

    def control(self, target_vl, target_va, current_vl, current_va, dbw_enabled):
        """
            target_vl:  target linear velocity.
            target_va:  target angular velocity.
            current_vl: current linear velocity.
            current_va: current angular velocity. current_va is always going to 0.
            dbw_enabled:    dbw state.
            
            Caculate the steering angle and accelerate from target velocity and current velocity.
        """
        if not dbw_enabled:
            self.need_init = True
            return 0, 0, 0

        if self.need_init:
            self.pid_speed.reset()
            self.pid_steer.reset()
            self.need_init = False
            rospy.loginfo("Init pid control.")
            
        # TODO: delta time between current to next waypoint. Neet to check.
        sample_time = 1.0 / 50      

        # Differance between current and target.
        error_velocity_linear = self.vector_length(target_vl) - self.vector_length(current_vl)
        error_velocity_angular = target_va.z

        throttle = self.pid_speed.step(error_velocity_linear, sample_time)
        angular = self.pid_steer.step(error_velocity_angular, sample_time)

        # Translate the wheel angle to steer angle.
        if abs(angular) > 0:
            current_vl_len = self.vector_length(current_vl)
            steering = self.yaw_ctl.get_angle(max(current_vl_len, self.yaw_ctl.min_speed) / angular)
        else:
            steering = 0.0

        # A negative throttle means we need to brake.
        if throttle < 0:
            brake = -throttle
            throttle = 0.11
        else:
            brake = 0

        # self.log("state: ", round(self.vector_length(target_vl)), round(self.vector_length(current_vl)), 
        #     round(throttle, 2), round(brake, 2), round(steering, 2), round(angular, 2))
        return throttle, brake, steering 

    def log(self, key_word, *msg):
        """
            print the log msg with a 1s interal.
        """
        if not self.debug_dict.has_key(key_word):
            self.debug_dict[key_word] = 0

        if self.debug_dict[key_word] == 50:
            rospy.logwarn(str(key_word) + "\n" + str(msg))
            self.debug_dict[key_word] = 0

        self.debug_dict[key_word] +=  1
