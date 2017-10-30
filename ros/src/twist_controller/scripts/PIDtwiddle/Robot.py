# ------------------------------------------------
# 
# this is the Robot class
# This class is a approximate model of the real code running in ROS.
# I use it to twiddle the pid parameters automatically, so it's a little messy.
# I simulated the WaypointLoder/WaypointUpdater/WaypointFollower/Controllor all in here.
# If you need to reuse, you need modify all the parameters to adapt your model.
import random
import numpy as np
import math
from yaw_controller import YawController

class Robot(object):
    def __init__(self, length=2.8498):
        """
        Creates robot and initializes location/orientation to 0, 0, 0.
        """
        self.x = 0.0
        self.y = 0.0
        self.velocity = 1.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0
        self.target_x = 0
        self.target_y = 0
        self.target_velocity = 11
        self.sample_time = 1
        self.max_steering_angle = 8/14.8
        self.max_radius = 9e10
        self.turn = 0
        self.locator = None
        self.last_target_angular = 0
        self.target_pos = []
        self.yaw_ctl = YawController(self.length, 1, 0.01, 3.0, self.max_steering_angle)
        # cte of current point different from basewaypoints
        self.pose_basewp_dist = 0
        # step numbers the robot ran.
        self.step_count = 0

    def init(self):
        self.update_target()

    def set(self, x, y, orientation):
        """
        Sets a robot coordinate.
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_target(self, target_x=0, target_y=0, target_velocity=11):
        self.target_x = target_x
        self.target_y = target_y
        self.target_velocity = target_velocity        

    def set_sample_time(self, sample_time):
        self.sample_time = sample_time

    def set_noise(self, steering_noise, distance_noise):
        """
        Sets the noise parameters.
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def caculate_cte(self):
        cte = self.last_target_angular - self.turn
        # cte = self.distance([self.target_x, self.target_y], [self.x, self.y])
        return cte

    def distance(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def calcCurvature(self, target):
        delta_x, delta_y = target[0] - self.x, target[1]-self.y
        theta1 = np.arctan(delta_y / delta_x)
        if delta_x < 0: theta1 += math.pi
        theta2 = theta1 - self.orientation
        # while theta2 > math.pi/2: theta2 -= math.pi/2
        l = math.sqrt(delta_x**2 + delta_y**2)
        relate_x, relate_y = l*math.cos(theta2), l*math.sin(theta2)
        # print 'theta2: ', theta2, theta1, self.orientation, delta_y, delta_x, target[0], self.x, target[1], self.y
        numerator = 2.0*relate_y
        denominator = l**2
        if denominator != 0:
            kappa = numerator / denominator
        elif numerator > 0: 
            kappa = 1.0/self.max_radius
        else:
            kappa = -1.0/self.max_radius
        target_steer = kappa * self.velocity
        return target_steer

    def interpolateNextTarget(self, next_wp_idx, start, end, detect_distance):
        # getLinearEquation
        x1, y1, x2, y2 = start[0], start[1], end[0], end[1]
        a, b, c = y2-y1, -1*(x2-x1), -1*(y2-y1)*x1 + (x2-x1)*y1
        # get distance
        d = abs(a*self.x+b*self.y+c) / math.sqrt(a**2+b**2)
        if d > detect_distance:
            print 'd > detect_distance.'
            return end;
        # print start, end, a,b,c,d
        # get normalized vector
        v = [x2-x1, y2-y1]
        #array([[ 0.51449576,  0.85749293]])
        norm_v = v / np.linalg.norm(v)
        x, y = norm_v[0], norm_v[1]
        w1, w2 = [-y, x], [y, -x]

        # get 2 interpolated point
        h1 = [self.x+d*w1[0], self.y+d*w1[1]]
        h2 = [self.x+d*w2[0], self.y+d*w2[1]]

        if abs(a*h1[0]+b*h1[1]+c) < 0.01:
            h = h1
        elif abs(a*h2[0]+b*h2[1]+c) < 0.01:
            h = h2
        else:
            print "h1 and h2 are both errors."
            print "norm_v, w1, w2, h1, h2: ", norm_v, w1, w2, h1, h2
            exit(-1)
        # interpolated a more far point.
        if d == detect_distance:
            print "d == detect_distance"
            return h
        s = math.sqrt(detect_distance**2 - d**2)
        t1 = [h[0]+s*norm_v[0], h[1]+s*norm_v[1]]
        t2 = [h[0]-s*norm_v[0], h[1]-s*norm_v[1]]
        interval = self.distance(end, start)
        if self.distance(t1, end) < interval+0.1:
            return t1
        elif self.distance(t2, end) < interval+0.1:
            print 't2 returned.', t2, t1, start, end, [self.x, self.y]
            return t2
        else:
            print "t1 and t2 are both wrong."
            print 'start, end, a,b,c,d, self',start, end, a,b,c,d,[self.x, self.y]
            print "t1, t2, interval: ",t1, t2, interval
            print "norm_v, h, dist(t1), dist(t2): ", norm_v, h, self.distance(t1, end), self.distance(t2, end)
            exit(-1)

    def waypoint_update(self):
        dist, _, idx_ahead, final_wps = self.locator.locate_waypoints_around([self.x, self.y])
        # if dist > self.pose_basewp_dist:
        self.pose_basewp_dist += dist
        self.step_count += 1
        if len(final_wps) < 5:
            print 'Finish circle. idx_ahead', idx_ahead
            exit(0)
        return final_wps

    def waypoint_follow(self, final_waypoints):
        # Find the fisrt waypoint that meets the condition
        detect_distance = max(6, 2*self.velocity)
        next_wp_idx = -1
        for i in range(len(final_waypoints)):
            if i == len(final_waypoints) - 1:
                print "search waypoint is the last"
                next_wp_idx = i
                break
            if self.distance(final_waypoints[i], [self.x, self.y]) > detect_distance:
                next_wp_idx = i
                break
        # Determine the target position.
        if next_wp_idx == 0 or next_wp_idx == len(final_waypoints)-1:
            # print 'Next waypoint is first or last:', next_wp_idx, final_waypoints[0], final_waypoints[-1],[self.x, self.y]
            target = final_waypoints[next_wp_idx]
        else:
            end = final_waypoints[next_wp_idx]
            start = final_waypoints[next_wp_idx-1]
            target = self.interpolateNextTarget(next_wp_idx, start, end, detect_distance)
        # Get target angular.
        target_angular = self.calcCurvature(target)
        end = final_waypoints[1]
        start = final_waypoints[2]        
        x1, x2, y1, y2 = start[0], start[1], end[0], end[1]
        a, b, c = y2-y1, -1*(x2-x1), -1*(y2-y1)*x1 + (x2-x1)*y1
        # get distance
        d = abs(a*self.x+b*self.y+c) / math.sqrt(a**2+b**2)
        if d < 0.1:
            angular = self.last_target_angular
            pass
        else:
            angular = target_angular

        return target, angular

    def update_target(self):
        final_waypoints = self.waypoint_update()
        target_position, angular = self.waypoint_follow(final_waypoints)

        self.last_target_angular = angular
        self.target_pos.append(target_position)
        self.set_target(target_position[0], target_position[1], 11)

    def move(self, turn, tolerance=0.0001):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        distance = self.velocity * self.sample_time
        if distance < 0.0:
            distance = 0.0

        # Execute motion
        # turn = np.tan(steering2) * distance / self.length
        self.turn = turn
        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance * np.cos(self.orientation)
            self.y += distance * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

        
        self.update_target()

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)
