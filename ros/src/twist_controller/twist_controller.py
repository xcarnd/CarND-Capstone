#*_coding:utf-8 _*
import math
import rospy
from pid import PID 
from yaw_controller import YawController
from geometry_msgs.msg import Point

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, brake_deadband, decel_limit, accel_limit, wheel_radius,
                    wheel_base, steer_ratio, max_lat_accel, max_steer_angle,
                    vehicle_mass, fuel_capacity, debug_pub, control_frequency):
        self.brake_deadband = brake_deadband
        self.decel_limit    = decel_limit
        self.accel_limit    = accel_limit
        self.wheel_radius = wheel_radius
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        # Is pid controller need to init.
        self.pid_need_init = True
        self.min_speed = 0.1   
        max_speed = 120 / 3.6   # unused
        speed_kp, speed_ki, speed_kd = 0.3, 0.08, 0.04
        # Manual pid parameters:
        # steer_kp, steer_ki, steer_kd = 0.56, 0.057, 0.05  
        # automatic pid parameters:
        steer_kp, steer_ki, steer_kd = 0.79673622805, 0.30044720425, 0.13508517176
        max_wheel_angle = max_steer_angle / steer_ratio
        self.pid_speed = PID(speed_kp, speed_ki, speed_kd, mn=decel_limit, mx=accel_limit, pid_type='speed')
        self.pid_angle = PID(steer_kp, steer_ki, steer_kd, mn=-max_wheel_angle, mx=max_wheel_angle, pid_type='angle')

        self.yaw_ctl = YawController(wheel_base, steer_ratio, self.min_speed, max_lat_accel, max_steer_angle)
        # For debug visualization.
        self.debug_pub = debug_pub
        self.control_frequency = control_frequency
        # save the last velocity, for curvature caculation.
        self.last_velocity = 0

    def control(self, target_vl, target_va, current_vl, current_va, current_pos, dbw_enabled):
        """
            target_vl:  target linear velocity.
            target_va:  target angular velocity.
            current_vl: current linear velocity.
            current_va: current angular velocity. current_va is always going to 0.
            current_pos:current pose.position
            dbw_enabled:    dbw state.
            
            Caculate the steering angle and accelerate from target velocity and current velocity.
        """
        if not dbw_enabled:
            self.pid_need_init = True
            return 0, 0, 0

        if self.pid_need_init:
            self.pid_speed.reset()
            self.pid_angle.reset()
            self.pid_need_init = False
            self.last_velocity = current_vl.x
            rospy.loginfo("PID controller ready.")
        
        """ The target point is transported by the pure_pursuit Node.
            I put them into 3 unused position:
            target_va.x, target_vl.y and target_vl.z.  """
        # target_pos = Point()
        # target_pos.x = target_va.x
        # target_pos.y = target_vl.y
        # target_pos.z = target_vl.z
        # d = self.distance(target_pos, current_pos)
        # mid_velocity = 0.5*(target_vl.x + current_vl.x)
        # target_time = d / mid_velocity

        error_velocity_linear = target_vl.x - current_vl.x
        error_velocity_angular = target_va.z - current_va.z
        sample_time = 1.0 / self.control_frequency
        throttle = self.pid_speed.step(error_velocity_linear, sample_time)
        angular = self.pid_angle.step(error_velocity_angular, sample_time)
        
        # Future optimize: you can consider the change of velocity.
        # Translate the wheel angle to steer angle.
        steering = self.yaw_ctl.get_steering(target_vl.x, angular, current_vl.x)

        brake = 0
        if throttle < 0:
            if abs(throttle) > self.brake_deadband:
                brake = abs(throttle) * self.vehicle_mass * self.wheel_radius
            throttle = 0

        # Keys are the lengend labels of visual graph.
        debug_msg = {   
                        # 'throttle':     throttle,
                        # 'brake/mass':        brake/self.vehicle_mass/self.wheel_radius, 
                        # 'target_vl.x/7':    target_vl.x / 7.0,
                        # 'current_vl.x/7':   current_vl.x / 7.0,
                        # 'wheel*10':     steering / self.steer_ratio*10, 
                        # 'wheel':     steering / self.steer_ratio, 
                        'target_va.z':  target_va.z,
                        'current_va.z': current_va.z,  

                        'current_pos_y':current_pos.y,
                        'current_pos_x':current_pos.x,
                        # 'target_pos_x':target_pos.x,
                        # 'target_pos_y':target_pos.y,
                    }
        # self.debug_publish(debug_msg)
        return throttle, brake, steering 

    def distance(self, a, b):
        """
            a and b are positions.
        """
        return math.sqrt( (a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

    def debug_publish(self, msg_dict):
        """
            print the log msg with a 1s interval.
        """
        keys = msg_dict.keys()
        msgs = msg_dict.values()
        self.debug_pub.publish(data=str([keys, msgs]))



        