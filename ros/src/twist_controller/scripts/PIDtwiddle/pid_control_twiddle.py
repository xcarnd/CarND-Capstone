# *_coding:utf-8_*----------------
# This is a PID parameters twiddle function.
# I hard coded the variables for my specific model. If you need to reuse, modify the `make_robot` and `run` functions.
# 使用twiddle函数在一个区间内自动查找最优的参数组合。
# 这种方式可以应用在其他需要组合参数优化的地方。
import numpy as np
import matplotlib.pyplot as plt
import csv
import copy
import time
from Robot import Robot
from locater import WaypointLocator
from pid import PID 

def run(robot, params, n=50):
    x_trajectory = []
    y_trajectory = []
    steer_kp, steer_ki, steer_kd = params[0], params[1], params[2]
    pid_angle = PID(steer_kp, steer_ki, steer_kd, mn=-robot.max_steering_angle, mx=robot.max_steering_angle, pid_type='angle')
    err = 0
    for i in range(2*n):
        cte = robot.caculate_cte()
        angular = pid_angle.step(cte, robot.sample_time)
        turn = robot.yaw_ctl.get_steering(robot.target_velocity, angular, robot.velocity)
        robot.move(turn)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        # 统计50(n)步到100(2n)步的误差，目标是将这期间的误差降低到最小。
        if i >= n:
            err += cte ** 2
    return x_trajectory, y_trajectory, 1.0*err / n

# 局部最优。
# 往上探索，如果减小了，则加大倍率，下次继续
#     如果增大了，则往下探索.
#           如果减小了，则加大倍率，下次
#           如果还是增大，则减小倍率，下次继续
def twiddle(waypoint_locater, tol=0.00001): 
    # p = [0, 0, 0]
    p = [0.7967362280585872, 0.3006914162165476, 0.13508517176729964]
    dp = [1, 1, 1]
    robot = make_robot(waypoint_locater)
    x_trajectory, y_trajectory, best_err = run(robot, p)
    best_cte = robot.pose_basewp_dist / robot.step_count + 15*best_err
    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_cte))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot(waypoint_locater)
            x_trajectory, y_trajectory, err = run(robot, p)
            dist_cte = robot.pose_basewp_dist / robot.step_count+ 15*best_err

            if dist_cte < best_cte:
                best_cte = dist_cte
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot(waypoint_locater)
                x_trajectory, y_trajectory, err = run(robot, p)
                dist_cte = robot.pose_basewp_dist / robot.step_count+ 15*best_err

                if dist_cte < best_cte:
                    best_cte = dist_cte
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p, best_cte

def load_waypoint():
    waypoints = []
    path = '../../../../../data/sim_waypoints.csv'
    with open(path, 'r') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
        for row in spamreader:
            row = ', '.join(row).split(',')
            waypoints.append((float(row[0]),float(row[1])) )
    print 'Waypoint load succ. len: %d' %len(waypoints)
    return waypoints


def make_robot(waypoint_locater):
    """
    Resets the robot back to the initial position and drift.
    You'll want to call this after you call `run`.
    """
    robot = Robot()
    robot.set(909.48, 1128.67, 0)
    # robot.set(1110.28430777, 1181.16314177, 0.173621560468)
    # robot.set_steering_drift(45.0 / 180 * np.pi)
    # robot.set_target(50, 6, 11)
    robot.set_sample_time(1.0/30)
    robot.velocity = 11.0
    robot.locator = waypoint_locater
    robot.init()
    return robot


if __name__ == '__main__':
    waypoints = load_waypoint()
    waypoints = waypoints[:-100]
    waypoint_locater = WaypointLocator(waypoints)
    # _, _, idx_ahead, final_wps = waypoint_locater.locate_waypoints_around([2359.7632380785399, 2790.8523915409241])
    # print "locate: ", idx_ahead, final_wps[0], final_wps[-1]

    params, err = twiddle(waypoint_locater)
    print("Final twiddle error = {}, params: {}".format(err, params))

    robot = make_robot(waypoint_locater)
    # params = [0.56, 0.057, 0.05]
    # params = [0.7964615060547852, 0.30306993392695375, 0.13508517176729942]
    x_trajectory, y_trajectory, err = run(robot, params, n=1000)
    distance_cte = robot.pose_basewp_dist / robot.step_count
    print 'err: ',err, ' distance_cte: ', distance_cte, ' m'
    n = len(x_trajectory)

    plt.plot(x_trajectory, y_trajectory, marker='.', color='g', label='Twiddle PID controller')
    way_x = [touple[0] for touple in robot.target_pos]
    way_y = [touple[1] for touple in robot.target_pos]
    plt.plot(way_x, way_y, marker='.', color='r', )

    # bway_x = [touple[0] for touple in waypoints]
    # bway_y = [touple[1] for touple in waypoints]
    # plt.plot(bway_x[:n], bway_y[:n], marker='.', color='b', ) 

    plt.show()


"""
These functions are failed attempt. Leave them alone.
def twiddle_all(robot, tol=0.03): 
    # p = [0.56, 0.057, 0.05]
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot_origin = copy.deepcopy(robot)

    best_err = run_all(robot, p)
    # best_err = 100
    while sum(dp) > tol:
        for i in range(len(p)):
            p[i] += dp[i]
            robot = copy.deepcopy(robot_origin)
            err = run_all(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = copy.deepcopy(robot_origin)
                err = run_all(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
    return p

def run_all(robot, params, n=60):
    steer_kp, steer_ki, steer_kd = params[0], params[1], params[2]
    pid_angle = PID(steer_kp, steer_ki, steer_kd, mn=-robot.max_steering_angle, mx=robot.max_steering_angle, pid_type='angle')
    err = 0
    for i in range(2*n):
        cte = robot.caculate_cte()
        angular = pid_angle.step(cte, robot.sample_time)
        turn = robot.yaw_ctl.get_steering(robot.target_velocity, angular, robot.velocity)
        robot.move(turn)
        if i >= n:
            err += cte ** 2
    return 1.0*err / n

def run_main(robot, params, n=60):
    x_trajectory = []
    y_trajectory = []
    steer_kp, steer_ki, steer_kd = params[0], params[1], params[2]
    pid_angle = PID(steer_kp, steer_ki, steer_kd, mn=-robot.max_steering_angle, mx=robot.max_steering_angle, pid_type='angle')
    err = 0
    start_time = time.time()
    local_time = time.strftime("%H:%M", time.localtime())
    with open('paras'+local_time+'.csv', 'w') as csvfile:
        spamwriter = csv.writer(csvfile, delimiter='\t')
        header = [('', 'p[0]', 'p[1]', 'p[2]', 'turn', 'target_angular', 'x', 'y', '',)]
        spamwriter.writerow(header)
        for i in range(n):
            cte = robot.caculate_cte()
            angular = pid_angle.step(cte, robot.sample_time)
            turn = robot.yaw_ctl.get_steering(robot.target_velocity, angular, robot.velocity)
            robot.move(turn)
            x_trajectory.append(robot.x)
            y_trajectory.append(robot.y)
            err += cte ** 2
            if i%30 == 0:
                p = twiddle_all(copy.deepcopy(robot))
                pid_angle.kp, pid_angle.ki, pid_angle.kd = p[0], p[1], p[2]
                data = [('', p[0], p[1], p[2], robot.turn, robot.last_target_angular, robot.x, robot.y, '')]
                spamwriter.writerow(data)
                print 'run step: ', i, "err: ", err, 'time: ', int(time.time() - start_time)
    # print 'err ', 1.0*err / n, params
    return x_trajectory, y_trajectory, 1.0*err / n

# def loop_twiddle(waypoint_locater, min_, max_, precision = 0.1):
#     j = k = l = min_
#     p_best = []
#     best_err = float('inf')
#     start_time = time.time()
#     while j < max_:
#         print 'j loop: ', j, 'time: ', int(time.time()-start_time)
#         j += precision
#         k = l = min_
#         while k < max_:
#             k += precision
#             l = min_
#             while l < max_:
#                 l += precision
#                 robot = make_robot(waypoint_locater)
#                 x_trajectory, y_trajectory, err = run(robot, [j,k,l])
#                 new_cte = robot.pose_basewp_dist / robot.step_count + 10*err

#                 if new_cte < best_err:
#                     best_err = new_cte
#                     p_best = [j,k,l]  
#     return p_best, best_err 
"""
