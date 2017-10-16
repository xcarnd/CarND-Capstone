#*_coding:utf-8 _*
import math
import rospy
from pid import PID 
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, brake_deadband, decel_limit, accel_limit, wheel_radius,
                    wheel_base, steer_ratio, max_lat_accel, max_steer_angle,
                    vehicle_mass, debug_pub):
        self.brake_deadband = brake_deadband
        self.decel_limit    = decel_limit
        self.accel_limit    = accel_limit
        self.wheel_radius = wheel_radius
        self.steer_ratio = steer_ratio
        self.vehicle_mass = vehicle_mass
        # Is pid controller need to init.
        self.need_init = True
        # max and min speed, m/s
        min_speed = 0   
        max_speed = 120 / 3.6   
        # TODO: find the best p i d parameters.
        speed_kp, speed_ki, speed_kd = 1.4, 0.0, 0  # -0.1
        steer_kp, steer_ki, steer_kd = 1.0, 0.0, 0  # -0.2
        max_wheel_angle = max_steer_angle * steer_ratio * math.pi / 180
        self.pid_speed = PID(speed_kp, speed_ki, speed_kd, mn=decel_limit, mx=accel_limit)
        self.pid_angle = PID(steer_kp, steer_ki, steer_kd, mn=-max_wheel_angle, mx=max_wheel_angle)

        self.yaw_ctl = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        # For For debug visualization.
        self.debug_dict = {}
        self.debug_pub = debug_pub

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
            self.pid_angle.reset()
            self.need_init = False
            rospy.loginfo("Init pid control.")

        # TODO: delta time between current to next waypoint. Neet to check.
        sample_time = 1.0 / 50      
        #current_va.z = min(1.5, current_va.z)   #TODO. Sometimes the anglor becomes to 30, something wrong. 
        
        error_velocity_linear = target_vl.x - current_vl.x
        error_velocity_angular = target_va.z - current_va.z

        throttle = self.pid_speed.step(error_velocity_linear, sample_time)
        angular = self.pid_angle.step(error_velocity_angular, sample_time)

        # Translate the wheel angle to steer angle.
        steering = self.yaw_ctl.get_steering(target_vl.x, angular, current_vl.x)

        brake = 0
        if throttle < 0:
            if abs(throttle) > self.brake_deadband:
                brake = abs(throttle) # * self.vehicle_mass * self.wheel_radius
            throttle = 0

        # Keys are the lengend labels of visual graph.
        debug_msg = {   'throttle':     throttle,
                        'brake':        brake, 
                        'wheel':     steering / self.steer_ratio, 
                        'target_vl.x/7':    target_vl.x / 7.0,
                        'current_vl.x/7':   current_vl.x / 7.0,
                        # 'error_velocity_angular':error_velocity_angular,
                        # 'target_va.z':  target_va.z,
                        # 'current_va.z': current_va.z,                        
                    }
        self.log("state: ", debug_msg, need_pub=True)
        return throttle, brake, steering 

    def log(self, key_word, msg_dict, log_hertz=50, need_pub=False, pub_herz=10, decimal=2):
        """
            print the log msg with a 1s interal.
        """
        if not self.debug_dict.has_key(key_word):
            self.debug_dict[key_word] = 0

        keys = msg_dict.keys()
        for key in keys:
            msg = msg_dict.get(key)
            if isinstance(msg, float):
                msg_dict[key] = round(msg, decimal) 
        msgs = msg_dict.values()

        # Log the msg
        if self.debug_dict[key_word] == log_hertz:
            rospy.logwarn("\n" + str(keys) + "\n" + str(msgs))
            self.debug_dict[key_word] = 0

        # Publish the msg for visulization.
        # Make sure the msg data is number.
        if need_pub and self.debug_dict[key_word] == pub_herz:
            self.debug_pub.publish(data=str([keys, msgs]))

        self.debug_dict[key_word] +=  1


        