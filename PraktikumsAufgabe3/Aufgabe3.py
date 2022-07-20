import math
from datetime import datetime
import time
from math import *
import numpy as np
from Robot_Simulator_V3 import emptyWorld
from Robot_Simulator_V3 import Robot
import matplotlib.pyplot as plt

global min_diff
min_diff = 0.5

def set_min_def(new):
    global min_diff
    min_diff = new

def curveDrive(robot, v, r, delta_theta):
    if v > robot._maxSpeed:
        v = robot._maxSpeed
    if v < -robot._maxSpeed:
        v = -robot._maxSpeed

    #print(robot._world.getTrueRobotPose())
    # Calculate the length the robot must walk on the circle
    circle_length = r * np.deg2rad(abs(delta_theta))

    # full s with speed v
    steps_full = int(circle_length/v)
    # for last step calculate Velocity ratio # to get the last missing length with reduced Velocity
    steps_rest = circle_length/v - steps_full

    #number of steps to complete length with given Velocity
    steps = steps_full / robot.getTimeStep()

    # for number of steps calculate the angle per step
    if steps_full == 0:
        angle_per_step = pi
    else:
        angle_per_step = delta_theta / steps_full

    for t in range(int(steps)):
        robot.move([v, np.deg2rad(angle_per_step)])

    # complete length with reduced velocity and one last rotation
    if steps_rest != 0:
        robot.move([v * steps_rest, np.deg2rad(angle_per_step)])

def straightDrive(robot, v, l):
    # Bewege Roboter
    # v = m/s
    # l = m

    #print(robot._world.getTrueRobotPose())

    if v > robot._maxSpeed:
        v = robot._maxSpeed
    if v < -robot._maxSpeed:
        v = -robot._maxSpeed

    # full s with speed v
    steps_full = int(l/v)

    # for last step calculate Velocity ratio # to get the last missing length with reduced Velocity
    steps_rest = l/v - steps_full

    #number of steps to complete length with given Velocity
    steps = steps_full / robot.getTimeStep()

    for t in range(int(steps)):
        robot.move([v, 0])
    # complete length with reduced velocity
    robot.move([v*steps_rest, 0])

def followLine(robot, v, p1, p2):
    #myWorld.drawsimplelines(p1[0], p1[1], p2[0], p2[1])

    #time_list = []
    #distance_list = []
    #count = 1

    while True:
        # get robot position
        robot_pose = robot._world.getTrueRobotPose()
        robot_position = np.array([robot_pose[0], robot_pose[1]])

        # condition to break when p2 has been reached
        if min_diff > np.sqrt(np.power(robot_pose[0]- p2[0] ,2) + pow(robot_pose[1]- p2[1], 2)):
            break

        # calculate and normalize direction vector of the line between p1 and p2
        r_original = p2 - p1
        length_r_original = math.sqrt(math.pow(r_original[0], 2) + math.pow(r_original[1], 2))
        r = r_original / length_r_original

        # use system of linear equation to calculate parameter t, to get the nearest point between robot position and line
        t = (robot_position[1] * r[1] - r[0] * p1[0] + robot_position[0] * r[0] - p1[1] * r[1]) / (math.pow(r[0], 2) + math.pow(r[1], 2))
        nearest_point = p1 + t * r

        # shortest vector between robot and line
        n = nearest_point - robot_position
        n_length = math.sqrt(math.pow(n[0], 2) + math.pow(n[1], 2))

        #distance_list.append(n_length)
        #time_list.append(count)
        #count = count + 0.2

        # calculate direction on the line, in which the robot should go
        n_r = robot_position + n + r - robot_position

        # this is the direction where the angle of robot is 0
        default_orientation_vector = np.array([1, 0])

        # calculate the angle of the desired direction
        unit_n_r = n_r / np.linalg.norm(n_r)
        unit_default_orientation_vector = default_orientation_vector / np.linalg.norm(default_orientation_vector)
        dot_product = np.dot(unit_n_r, unit_default_orientation_vector)
        theta_star = np.arccos(dot_product)

        # get theta for pose
        theta = robot_pose[2]

        # calculate angle with which robot should correct its orientation
        # see 3-7 for the idea
        # theta_star should be shown as the outer angle
        # if the normal vector to the line shows downwards, then the outer angle is calculated like 2*pi - theta_star
        # if the normal vector shows upwards, then the outer angle is calculated like 2*pi + theta_star
        if n_r[1] <= 0:
            diff_angle = (2*pi - theta_star - theta + pi) % (2*pi) - pi
        else:
            diff_angle = (2*pi + theta_star - theta + pi) % (2*pi) - pi

        # move the robot with speed and angle
        robot.move([v, diff_angle])

    # plot distance between robot and line on time
    # plt.title('Abweichungsfehler');
    # plt.plot(time_list, distance_list, "-b")
    # plt.xlabel("Time [s]")
    # plt.ylabel("Distance [m]")
    # plt.show()

def followLine_lab(robot, v, p1, p2, d):
    #myWorld.drawsimplelines(p1[0], p1[1], p2[0], p2[1])

    #time_list = []
    #distance_list = []
    #count = 1

    while True:
        # get robot position
        robot_pose = robot._world.getTrueRobotPose()
        robot_position = np.array([robot_pose[0], robot_pose[1]])
        dists = robot.sense()[11:13]
        if dists[0]:
            if dists[0] < d:
                robot.move([0,pi])
                robot.move([0,pi])
                robot.move([0,pi])
                robot.move([0,pi])
                robot.move([0,pi])
                robot.move([0.001,0])
                break
        if dists[1]:
            if dists[1] < d :
                robot.move([0,pi])
                robot.move([0,pi])
                robot.move([0,pi])
                robot.move([0,pi])
                robot.move([0,pi])
                robot.move([0.001,0])
                break

        directions = robot.getSensorDirections()[11:13]

        # condition to break when p2 has been reached
        if min_diff > np.sqrt(np.power(robot_pose[0]- p2[0] ,2) + pow(robot_pose[1]- p2[1], 2)):
            break

        # calculate and normalize direction vector of the line between p1 and p2
        r_original = p2 - p1
        length_r_original = math.sqrt(math.pow(r_original[0], 2) + math.pow(r_original[1], 2))
        r = r_original / length_r_original

        # use system of linear equation to calculate parameter t, to get the nearest point between robot position and line
        t = (robot_position[1] * r[1] - r[0] * p1[0] + robot_position[0] * r[0] - p1[1] * r[1]) / (math.pow(r[0], 2) + math.pow(r[1], 2))
        nearest_point = p1 + t * r

        # shortest vector between robot and line
        n = nearest_point - robot_position
        n_length = math.sqrt(math.pow(n[0], 2) + math.pow(n[1], 2))

        #distance_list.append(n_length)
        #time_list.append(count)
        #count = count + 0.2

        # calculate direction on the line, in which the robot should go
        n_r = robot_position + n + r - robot_position

        # this is the direction where the angle of robot is 0
        default_orientation_vector = np.array([1, 0])

        # calculate the angle of the desired direction
        unit_n_r = n_r / np.linalg.norm(n_r)
        unit_default_orientation_vector = default_orientation_vector / np.linalg.norm(default_orientation_vector)
        dot_product = np.dot(unit_n_r, unit_default_orientation_vector)
        theta_star = np.arccos(dot_product)

        # get theta for pose
        theta = robot_pose[2]

        # calculate angle with which robot should correct its orientation
        # see 3-7 for the idea
        # theta_star should be shown as the outer angle
        # if the normal vector to the line shows downwards, then the outer angle is calculated like 2*pi - theta_star
        # if the normal vector shows upwards, then the outer angle is calculated like 2*pi + theta_star
        if n_r[1] <= 0:
            diff_angle = (2*pi - theta_star - theta + pi) % (2*pi) - pi
        else:
            diff_angle = (2*pi + theta_star - theta + pi) % (2*pi) - pi

        # move the robot with speed and angle
        robot.move([v, diff_angle])

    # plot distance between robot and line on time
    # plt.title('Abweichungsfehler');
    # plt.plot(time_list, distance_list, "-b")
    # plt.xlabel("Time [s]")
    # plt.ylabel("Distance [m]")
    # plt.show()

def gotoGlobal(robot, v, p, tol = min_diff):
    while True:
        # get robot position
        robot_pose = robot._world.getTrueRobotPose()
        robot_position = np.array([robot_pose[0], robot_pose[1]])

        theta_star = np.arctan2(p[1] - robot_position[1], p[0] - robot_position[0])
        theta = robot_pose[2]

        #diff_angle = theta_star - theta
        diff_angle = (2*pi + theta_star - theta + pi) % (2*pi) - pi

        velocity = math.sqrt(math.pow(robot_position[0] - p[0], 2) + math.pow(robot_position[1] - p[1], 2))
        if velocity > v:
            velocity = v

        robot.move([velocity, diff_angle])
        # condition to break when p2 has been reached
        if tol > np.sqrt(np.power(robot_pose[0]- p[0] ,2) + pow(robot_pose[1]- p[1], 2)):
            break

def followPolylinePoints(robot, v, poly):
    for index, point in enumerate(poly):
        tol = 0.5
        if index == len(poly) - 1:
            tol = 0.1
        gotoGlobal(robot, v, point, tol)

def get_poly_line(points):
    polyline = []
    for index, point in enumerate(points):
        polyline.append(np.array([points[index], points[index+1]]))
        if index + 1 == len(points) - 1:
            break
    return polyline

def followPolyline_lines(robot, v, poly):
    global min_diff
    min_diff = 3
    for i in range(len(poly)-1):
        followLine(robot, v, poly[i][0], poly[i][1])
    min_diff = 0.1
    followLine(robot, v, poly[len(poly)-1][0], poly[len(poly)-1][1])

# Roboter in einer Welt positionieren:
# myWorld = emptyWorld.buildWorld()
# myRobot = Robot.Robot()
# myWorld.setRobot(myRobot, [3, 2, np.deg2rad(45)])

# Teil a)
# Lane change
# straightDrive(myRobot,0.2, 5)
# curveDrive(myRobot, 0.2, 4, 45)
# curveDrive(myRobot, 0.2, 4, -45)
# straightDrive(myRobot,0.2, 5)

# Teil b)
# 1)
# horizontal
#myWorld.setRobot(myRobot, [2, 17, np.deg2rad(45)])
#followLine(myRobot, 0.1, np.array([1, 10]), np.array([15, 10]))
#followLine(myRobot, 0.5, np.array([18, 10]), np.array([1, 10]))

# vertical
#followLine(myRobot, 0.5, np.array([10, 1]), np.array([10,19]))
#followLine(myRobot, 0.5, np.array([10,19]), np.array([10, 1]))

# left to right
#followLine(myRobot, 0.5, np.array([2,2]), np.array([18, 18]))
#followLine(myRobot, 0.5, np.array([18, 18]), np.array([2, 2]))

#2)
# myWorld.setRobot(myRobot, [10, 15, np.deg2rad(180)])
# gotoGlobal(myRobot, 0.2, np.array([10, 10]), 0.1)
#
# myWorld.setRobot(myRobot, [7.5, 12.5, np.deg2rad(225)])
# gotoGlobal(myRobot, 0.2, np.array([10, 10]), 0.1)
#
# myWorld.setRobot(myRobot, [5, 10, np.deg2rad(270)])
# gotoGlobal(myRobot, 0.2, np.array([10, 10]), 0.1)
#
# myWorld.setRobot(myRobot, [7.5, 7.5, np.deg2rad(315)])
# gotoGlobal(myRobot, 0.2, np.array([10, 10]), 0.1)
#
# myWorld.setRobot(myRobot, [10, 5, np.deg2rad(0)])
# gotoGlobal(myRobot, 0.2, np.array([10, 10]), 0.1)
#
# myWorld.setRobot(myRobot, [12.5, 7.5, np.deg2rad(45)])
# gotoGlobal(myRobot, 0.2, np.array([10, 10]), 0.1)
#
# myWorld.setRobot(myRobot, [15, 10, np.deg2rad(90)])
# gotoGlobal(myRobot, 0.2, np.array([10, 10]), 0.1)
#
# myWorld.setRobot(myRobot, [12.5, 12.5, np.deg2rad(135)])
# gotoGlobal(myRobot, 0.2, np.array([10, 10]), 0.1)

#3)
# myWorld.setRobot(myRobot, [12.5, 12.5, np.deg2rad(135)])
# points = np.array([18, 5]),  np.array([5, 5]), np.array([10,19]),  np.array([14, 14]), np.array([18, 11]),  np.array([2, 2])
# poly = get_poly_line([myRobot._world.getTrueRobotPose()[:2], np.array([18, 5]),  np.array([5, 5]), np.array([10,19]),  np.array([14, 14]), np.array([18, 11]),  np.array([2, 2])])
# myWorld.drawPolylines(poly)
# followPolylinePoints(myRobot, 0.3, points)
#followPolyline_lines(myRobot, 0.3, poly)


# Simulation schliessen:
#myWorld.close()