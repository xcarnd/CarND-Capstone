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
                    vehicle_mass, debug_pub):
        self.brake_deadband = brake_deadband
        self.decel_limit    = decel_limit
        self.accel_limit    = accel_limit
        self.wheel_radius = wheel_radius
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.vehicle_mass = vehicle_mass
        # Is pid controller need to init.
        self.pid_need_init = True
        self.min_speed = 0.1   
        max_speed = 120 / 3.6   # unused
        # TODO: find the best p i d parameters.
        speed_kp, speed_ki, speed_kd = 0.3, 0.04, 0.04
        steer_kp, steer_ki, steer_kd = 0.44, 0.05, 0.01   
        #steer_kp, steer_ki, steer_kd = 0.44, 0.15, 0.10   
        max_wheel_angle = max_steer_angle / steer_ratio
        self.pid_speed = PID(speed_kp, speed_ki, speed_kd, mn=decel_limit, mx=accel_limit, pid_type='speed')
        self.pid_angle = PID(steer_kp, steer_ki, steer_kd, mn=-max_wheel_angle, mx=max_wheel_angle, pid_type='angle')

        self.yaw_ctl = YawController(wheel_base, steer_ratio, self.min_speed, max_lat_accel, max_steer_angle)
        # For debug visualization.
        self.debug_pub = debug_pub

    def control(self, target_vl, target_va, current_vl, current_va, next_waypoint_pos, current_pos, dbw_enabled):
        """
            target_vl:  target linear velocity.
            target_va:  target angular velocity.
            current_vl: current linear velocity.
            current_va: current angular velocity. current_va is always going to 0.
            next_waypoint_pos: the next target(waypoint) position.
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
            rospy.loginfo("Init pid control.")
        
        # The target point is transported by the pure_pursuit Node.
        # I put them into 3 unused position:
        #   target_va.x, target_vl.y and target_vl.z.
        # target_pos = Point()
        # target_pos.x = target_va.x
        # target_pos.y = target_vl.y
        # target_pos.z = target_vl.z
        # d = self.distance(target_pos, current_pos)

        # mid_velocity = 0.5*(target_vl.x + current_vl.x) 
        # latency =0.1
        # delta_d = mid_velocity * latency
        # if d > 1.1*delta_d:
        #     d -= delta_d
        # if mid_velocity == 0: mid_velocity = self.min_speed
        # target_time = d / transition_v

        # It's a circumvention. I got single abnormal angular sometimes .
        if current_va.z > 3:
            current_va.z = 0.1
        elif current_va.z < -3:
            current_va.z = -0.1
        error_velocity_linear = target_vl.x - current_vl.x
        error_velocity_angular = target_va.z - current_va.z
        sample_time = 1.0/50
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
        debug_msg = {   
                        # 'throttle':     throttle,
                        # 'brake':        brake, 
                        # 'target_vl.x/7':    target_vl.x / 7.0,
                        # 'current_vl.x/7':   current_vl.x / 7.0,
                        'wheel':     steering / self.steer_ratio, 
                        'target_va.z':  target_va.z,
                        'current_va.z': current_va.z,  

                        'waypoint_pos_y':next_waypoint_pos.y,
                        'waypoint_pos_x':next_waypoint_pos.x,
                        'current_pos_y':current_pos.y,
                        'current_pos_x':current_pos.x,
                    }
        self.debug_publish(debug_msg)
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



        