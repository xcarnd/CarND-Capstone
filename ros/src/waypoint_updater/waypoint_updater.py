#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from waypoint_locator.srv import *
from std_msgs.msg import Header

import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # all the waypoints
        self.all_ref_waypoints = []

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.locate_waypoints_around = rospy.ServiceProxy('/waypoint_locator/locate_waypoints_around', LocateWaypointsAround)
        rospy.wait_for_service('/waypoint_locator/locate_waypoints_around')
        
        rospy.spin()

    def pose_cb(self, msg):
        # a simple implementation: find the reference waypoint ahead,
        # then publish 200 waypoints from that ahead waypoint
        num_all_ref_waypoints = len(self.all_ref_waypoints)
        if len(self.all_ref_waypoints) == 0:
            return
        req = LocateWaypointsAroundRequest(msg.pose)
        resp = self.locate_waypoints_around(req)

        final_waypoints = Lane()
        final_waypoints.header = Header()
        final_waypoints.header.stamp = rospy.Time.now()
        final_waypoints.header.frame_id = "/world"
        final_waypoints.waypoints = []
            
        idx_ahead = resp.ahead
        base_offset = 1
        i = 0
        while i < 200:
            idx = idx_ahead + i + base_offset
            if idx >= num_all_ref_waypoints:
                idx -= num_all_ref_waypoints
            ref_waypoint = self.all_ref_waypoints[idx]
            waypoint = Waypoint()
            waypoint.pose.header = Header()
            waypoint.pose.header.stamp = rospy.Time.now()
            waypoint.pose.header.frame_id= "/world"
            waypoint.pose.pose.position = ref_waypoint.pose.pose.position
            waypoint.pose.pose.orientation = ref_waypoint.pose.pose.orientation
            waypoint.twist.header = Header()
            waypoint.twist.header.stamp = rospy.Time.now()
            waypoint.twist.header.frame_id = "/world"
            waypoint.twist.twist.linear = ref_waypoint.twist.twist.linear
            waypoint.twist.twist.angular = ref_waypoint.twist.twist.angular

            final_waypoints.waypoints.append(waypoint)
            i += 1
        self.final_waypoints_pub.publish(final_waypoints)
        

    def waypoints_cb(self, waypoints):
        # simple store all the reference waypoints
        all_ref_waypoints = []
        waypoints = waypoints.waypoints
        for waypoint in waypoints:
            all_ref_waypoints.append(waypoint)
        self.all_ref_waypoints = all_ref_waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
