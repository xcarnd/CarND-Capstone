#!/usr/bin/env python

import rospy
import math
import signal
from std_msgs.msg import Bool, String
from styx_msgs.msg import Lane, Waypoint
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)

        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        #steer_ratio = 2.67
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.target_velocity = None
        self.current_velocity = None
        self.current_posistion = None
        self.target_position = None
        self.dbw_enabled = False
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)
        self.debug_pub = rospy.Publisher('/dbw_node/debug', String, queue_size=1)

        sub1 = rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        sub2 = rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        sub3 = rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enable_cb)
        # sub4 = rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)
        sub5 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        
        self.controller = Controller(brake_deadband, decel_limit, accel_limit, wheel_radius,
                                        wheel_base, steer_ratio, max_lat_accel, max_steer_angle, 
                                        vehicle_mass, self.debug_pub)
        self.loop()


    def twist_cb(self, msg):
        self.target_velocity = msg

    def velocity_cb(self, msg):
        self.current_velocity = msg
        # Transform degree to radian
        self.current_velocity.twist.angular.z *= math.pi/180

    def dbw_enable_cb(self, msg):
        self.dbw_enabled = msg.data

    def pose_cb(self, msg):
        self.current_posistion = msg.pose.position

    def loop(self):
        rate = rospy.Rate(10) # 20Hz

        while not (self.target_velocity and self.current_velocity and self.current_posistion):
            # rospy.loginfo("Target_velocity or current_velocity missing.")
            rate.sleep()

        while not rospy.is_shutdown():
            throttle, brake, steering = self.controller.control(self.target_velocity.twist.linear,
                                                                self.target_velocity.twist.angular,
                                                                self.current_velocity.twist.linear,
                                                                self.current_velocity.twist.angular,
                                                                self.current_posistion,
                                                                self.dbw_enabled)
            if self.dbw_enabled:
                self.publish(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

def signal_handler(signum, frame):
    exit(signum)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)    
    DBWNode()
