#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightState
from std_msgs.msg import Header
from location_utils.locator import WaypointLocator
import math
import time
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        # all the waypoints
        self.locator = None
        self.all_ref_waypoints = []
        self.current_linear_velocity = 0
        # control the number of final waypoints.
        self.detect_second = 4
        # the `minimum_lookahead_distance_` in 'pure_pursuit_core.h' is 6
        self.minimum_detect_distance = 6 + 1
        self.upcoming_traffic_light_wp_idx = -1
        self.traffic_light_state = -1     
        self.accel_limit = rospy.get_param('accel_limit', 1.)
        self.decel_limit = abs(rospy.get_param('decel_limit', -5))
        self.max_num_final_waypoints = rospy.get_param('max_num_final_waypoints', 100)
        self.final_waypoints = [ Waypoint() for i in range(self.max_num_final_waypoints)]

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/traffic_waypoint', TrafficLightState, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        """ 
            This function subscrible current pose, and publish the final waypoint.
            The waypoint follwer minimum detect distance is min(6m, 2s * current_velocity).
            So, the final waypoint from 0m to max(6m, 3s * current_velocity) is enough.
        """
        num_all_ref_waypoints = len(self.all_ref_waypoints)
        if self.locator is None or num_all_ref_waypoints == 0:
            return

        # Detect distance.
        detect_distance = self.detect_second * self.current_linear_velocity
        if detect_distance < self.minimum_detect_distance:
            detect_distance = self.minimum_detect_distance 
        detect_distance_squre = detect_distance**2

        # Find the reference waypoint ahead first
        _, _, idx_ahead = self.locator.locate_waypoints_around(msg.pose)
        i = 0
        while i < self.max_num_final_waypoints:
            idx = idx_ahead + i
            if idx >= num_all_ref_waypoints:
                idx -= num_all_ref_waypoints
            ref_waypoint = self.all_ref_waypoints[idx]
            wp_position = ref_waypoint.pose.pose.position
            distance_squre = self.pose_distance_squre(msg.pose.position, wp_position)
            if distance_squre > detect_distance_squre:
                if i == 0: return
                break

            waypoint = self.final_waypoints[i]
            waypoint.pose.pose = ref_waypoint.pose.pose
            waypoint.twist.twist.linear.x = ref_waypoint.twist.twist.linear.x
            # waypoint.twist.twist.linear.x = 50*1.609344 / 3.6
            waypoint.twist.twist.angular = ref_waypoint.twist.twist.angular
            i += 1
        
        waypoints = self.final_waypoints[:i]
        waypoints = self.process_traffic_lights(waypoints, idx_ahead, idx_ahead + i-1, msg.pose.position)
        waypoints = self.smooth_start(waypoints, msg.pose.position)

        final_waypoints = Lane()
        final_waypoints.header.frame_id = "/world"
        final_waypoints.waypoints = waypoints

        self.final_waypoints_pub.publish(final_waypoints)

    def waypoints_cb(self, waypoints):
        self.all_ref_waypoints = waypoints.waypoints
        self.locator = WaypointLocator(self.all_ref_waypoints)
        self.base_waypoints_info()

    def base_waypoints_info(self):
        """
            This function will only run once.
            This function is to overview the base waypoints.
        """
        waypoints = self.all_ref_waypoints
        if len(waypoints) == 0:
            rospy.logfatal("No waypoints found.")
            return

        d_sum, _ = self.wp_distance(waypoints, 0, len(waypoints)-1)
        max_interval, height_diff = _[0], _[1]

        start = waypoints[0].pose.pose.position
        fifth = waypoints[4].pose.pose.position      
        end   = waypoints[-1].pose.pose.position

        rospy.loginfo("Number of waypoints: %d, max interval: %.2f, way length: %.2f",
                        len(waypoints), max_interval, d_sum)
        # rospy.loginfo("Start: (%.2f, %.2f), 5th: (%.2f, %.2f), END: (%.2f, %.2f)." \
                        # %(start.x, start.y, fifth.x, fifth.y, end.x, end.y))
        if height_diff == 0:
            rospy.loginfo("All the way is flat.")
        else:
            rospy.loginfo("There is %.2f Height difference.", height_diff)

    def velocity_cb(self, msg):
        self.current_linear_velocity = msg.twist.linear.x

    def traffic_cb(self, msg):
        self.upcoming_traffic_light_wp_idx = msg.wp_index
        self.traffic_light_state = msg.state
                                    
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def wp_distance(self, waypoints, wp1, wp2):
        dist = d_max_inteval = 0
        z_max = z_min = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            d = dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            dist += d

            if d > d_max_inteval: d_max_inteval = d
            z = waypoints[wp1].pose.pose.position.z
            if z > z_max: z_max = z
            if z < z_min: z_min = z              
            wp1 = i

          
        return dist, [d_max_inteval, z_max-z_min]

    def pose_distance_squre(self, a, b):
        return (a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2

    def process_traffic_lights(self, waypoints, start, end, current_position):
        """
            If there is a traffic light in the final waypoints,  this function
            will adjust the waypoints' linear velocity to avoid violation of traffic rules.
            TrafficLight.RED:       stop.
            TrafficLight.YELLOW:    decelerate.
        """
        light_idx = self.upcoming_traffic_light_wp_idx
        light_state = self.traffic_light_state
        if start >= light_idx:
            return waypoints

        stop_wp = self.all_ref_waypoints[light_idx]
        max_brake_time = 5
        max_brake_distance = 0.5 * self.decel_limit * max_brake_time**2
        current_brake_distance = self.current_linear_velocity**2 / (2*self.decel_limit)
        brake_distance = max(max_brake_distance, current_brake_distance)
        for idx in xrange(len(waypoints)):
            if idx + start >= light_idx:
                if light_state == TrafficLight.RED:
                    waypoints[idx].twist.twist.linear.x = 0
            else:
                dist = math.sqrt(self.pose_distance_squre(waypoints[idx].pose.pose.position, stop_wp.pose.pose.position))
                if dist > brake_distance:
                    continue

                if light_state == TrafficLight.RED:
                    vel = 1.3*math.sqrt(dist)
                    if vel < 2: vel = 0
                    waypoints[idx].twist.twist.linear.x = min(vel, waypoints[idx].twist.twist.linear.x)

                elif light_state == TrafficLight.YELLOW:
                    waypoints[idx].twist.twist.linear.x *= 0.8
        # For debug.
        stop_line_distance = math.sqrt(self.pose_distance_squre(current_position, stop_wp.pose.pose.position))
        rospy.loginfo_throttle(0.5, "Light: %d, Target vel: %.1f m/s. Stop line: %.1f m" 
                %(self.traffic_light_state, waypoints[0].twist.twist.linear.x, stop_line_distance))
        return waypoints

    def smooth_start(self, waypoints, current_position):
        """
            If the car is going to accelerate, apply the soft accelerate to avoid aggressive start
            This will not take effect if the current velocity is above 0.7 * max velocity.
        """

        v0 = self.current_linear_velocity
        next_v = waypoints[0].twist.twist.linear.x
        final_v = self.all_ref_waypoints[0].twist.twist.linear.x
        factor = 1.0*v0 / final_v
        # Only the car is going to accelerate. 
        if v0 > next_v or factor > 0.7:
            return waypoints
            
        # limit accel from 1 to 3.8 m/s^2
        soft_accel = self.accel_limit * (1 + 4*factor)
        for i in range(len(waypoints)):
            target_v = waypoints[i].twist.twist.linear.x
            dist = math.sqrt(self.pose_distance_squre(current_position, waypoints[i].pose.pose.position))
            vt =  math.sqrt(2 * soft_accel * dist + v0**2)
            waypoints[i].twist.twist.linear.x = min(vt, target_v)

        return waypoints


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
