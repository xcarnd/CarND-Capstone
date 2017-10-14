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
        # Is pid controller need to init.
        self.need_init = True
        # max and min speed, m/s
        min_speed = 0.001       
        max_speed = 120 / 3.6   
        # TODO: find the best pid parameters.
        speed_kp, speed_ki, speed_kd = 1, 1, 1
        steer_kp, steer_ki, steer_kd = 1, 1, 1
        self.pid_speed = PID(speed_kp, speed_ki, speed_kd, mn=min_speed, mx=max_speed)
        self.pid_steer = PID(steer_kp, steer_ki, steer_kd)

        self.yaw_ctl = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

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
            current_va: current angular velocity.
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
        error_velocity_linear = self.distance(target_vl, current_vl)
        error_velocity_angular = self.distance(target_va, current_va)

        throttle = self.pid_speed.step(error_velocity_linear, sample_time)
        angular_velocity = self.pid_steer.step(error_velocity_angular, sample_time)


        # Translate the wheel angle to steer angle.
        if abs(angular_velocity) > 0:
            current_vl_len = self.vector_length(current_vl)
            steering = self.yaw_ctl.get_angle(max(current_vl_len, self.yaw_ctl.min_speed) / angular_velocity)
        else:
            steering = 0.0

        # A negative throttle means we need to brake.
        if throttle < 0:
            brake = min(-throttle, self.decel_limit)
            throttle = 0
        else:
            throttle = min(throttle, self.accel_limit)
            brake = 0


        return throttle, brake, steering
        #return 1., 0., 0. 
