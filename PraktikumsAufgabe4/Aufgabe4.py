import math
import time
from math import pi

import numpy as np

from PraktikumsAufgabe3.Aufgabe3 import followLine, gotoGlobal, set_min_def, curveDrive, followLine_lab
from Robot_Simulator_V3 import labyrinthWorld
from Robot_Simulator_V3 import Robot
from Robot_Simulator_V3 import sensorUtilities

myWorld = labyrinthWorld.buildWorld()
myRobot = Robot.Robot()

# KeyboardController definieren:
myKeyboardController = myWorld.getKeyboardController()

def wander(robot, v):
    while True:
        dists = robot.sense()
        directions = robot.getSensorDirections()
        min = [-1,10]
        for i in range(0,len(dists)-1):
            if dists[i]:
                if dists[i] < min[1]:
                    min = [i,dists[i]]
        if min[0] == -1:
            robot.move([v, 0])
            continue
        if min[1] < 1 * (v - v/6) or min[1] < 0.5:
            if (directions[min[0]]) <= 0:
                robot.move([v/6, +pi])
                continue
            else:
                robot.move([v/6, -pi])
                continue
        robot.move([v, 0])

def wander_to_wall(robot, v , d):
    while True:
        dists = robot.sense() #28
        directions = robot.getSensorDirections()
        min = [-1,10]
        for i in range(0,len(dists)-1):
            if dists[i]:
                if dists[i] < min[1]:
                    min = [i,dists[i]]
        if min[0] == -1:
            robot.move([v, 0])
            continue
        if min[1] < d:
            if (directions[min[0]]) <= 0:
                robot.move([0, +pi])
            else:
                robot.move([0, -pi])
            break
        robot.move([v, 0])

def follow_wall(robot, v, d):
    set_min_def(0.1)
    wander_to_wall(robot, v, d)
    old_robot_position = np.array([-1, -1])
    while True:
        dists = myRobot.sense()
        directions = myRobot.getSensorDirections()
        lines_l = sensorUtilities.extractSegmentsFromSensorData(dists, directions)
        lines_g = sensorUtilities.transformPolylinesL2G(lines_l, myWorld.getTrueRobotPose())

        robot_pose = robot._world.getTrueRobotPose()
        robot_position = np.array([robot_pose[0], robot_pose[1]])
        lowest_distance_and_vector = [100, [], [], []]
        for line in lines_g:
            p1 = np.array([line[0][0], line[0][1]])
            p2 = np.array([line[1][0], line[1][1]])
            # calculate and normalize direction vector of the line between p1 and p2
            r_original = p2 - p1
            length_r_original = math.sqrt(math.pow(r_original[0], 2) + math.pow(r_original[1], 2))
            r = r_original / length_r_original

            # use system of linear equation to calculate parameter t, to get the nearest point between robot position and line
            t = (robot_position[1] * r[1] - r[0] * p1[0] + robot_position[0] * r[0] - p1[1] * r[1]) / (math.pow(r[0], 2) + math.pow(r[1], 2))
            nearest_point = p1 + t * r

            # shortest vector between robot and line
            n = nearest_point - robot_position
            distance_to_line = math.sqrt(math.pow(n[0], 2) + math.pow(n[1], 2))
            n_normalized_with_d = (n / distance_to_line) * -1 * d

            if distance_to_line < lowest_distance_and_vector[0]:
                lowest_distance_and_vector[0] = distance_to_line
                lowest_distance_and_vector[1] = n_normalized_with_d
                lowest_distance_and_vector[2] = p1
                lowest_distance_and_vector[3] = p2

        if round(old_robot_position[0],4) == round(robot_position[0],4) and round(old_robot_position[1],4) == round(robot_position[1],4):
            time.sleep(30)
            break

        myWorld.drawPolylines([[lowest_distance_and_vector[2], lowest_distance_and_vector[3]]])

        if  robot_pose[2] < pi or robot_pose[2] > -pi:
            followLine(robot, v, lowest_distance_and_vector[2] + lowest_distance_and_vector[1],
                   lowest_distance_and_vector[3] + lowest_distance_and_vector[1])
        else:
            followLine(robot, v, lowest_distance_and_vector[3] + lowest_distance_and_vector[1], lowest_distance_and_vector[2] + lowest_distance_and_vector[1])

        old_robot_position = robot_position

def get_out(robot, v, d = 0.75):
    wander_to_wall(robot, v, d)
    set_min_def(0.2)
    old_robot_position = np.array([-1, -1])

    while True:
        dists = robot.sense()[3:11]
        directions = robot.getSensorDirections()[3:11]

        lines_l = sensorUtilities.extractSegmentsFromSensorData(dists, directions)
        lines_g = sensorUtilities.transformPolylinesL2G(lines_l, myWorld.getTrueRobotPose())

        robot_pose = robot._world.getTrueRobotPose()
        robot_position = np.array([robot_pose[0], robot_pose[1]])
        lowest_distance_and_vector = [100, [], [], []]
        for line in lines_g:
            p1 = np.array([line[0][0], line[0][1]])
            p2 = np.array([line[1][0], line[1][1]])
            # calculate and normalize direction vector of the line between p1 and p2
            r_original = p2 - p1
            length_r_original = math.sqrt(math.pow(r_original[0], 2) + math.pow(r_original[1], 2))
            r = r_original / length_r_original

            # use system of linear equation to calculate parameter t, to get the nearest point between robot position and line
            t = (robot_position[1] * r[1] - r[0] * p1[0] + robot_position[0] * r[0] - p1[1] * r[1]) / (math.pow(r[0], 2) + math.pow(r[1], 2))
            nearest_point = p1 + t * r

            # shortest vector between robot and line
            n = nearest_point - robot_position
            distance_to_line = math.sqrt(math.pow(n[0], 2) + math.pow(n[1], 2))
            n_normalized_with_d = (n / distance_to_line) * -1 * d

            if distance_to_line < lowest_distance_and_vector[0]:
                lowest_distance_and_vector[0] = distance_to_line
                lowest_distance_and_vector[1] = n_normalized_with_d
                lowest_distance_and_vector[2] = p1
                lowest_distance_and_vector[3] = p2

        if round(old_robot_position[0],4) == round(robot_position[0],4) and round(old_robot_position[1],4) == round(robot_position[1],4):
            curveDrive(robot, v, d + v/2.2, -90)

            dists_tmp = robot.sense()[6:9]
            directions_tmp = robot.getSensorDirections()[6:9]
            lines_l_tmp = sensorUtilities.extractSegmentsFromSensorData(dists_tmp, directions_tmp)

            lines_g_tmp = sensorUtilities.transformPolylinesL2G(lines_l_tmp, myWorld.getTrueRobotPose())
            myWorld.drawPolylines(lines_g_tmp)

            if not lines_l_tmp:
                curveDrive(robot, v, d + v/ 3, -90)
        else:
            if len(lowest_distance_and_vector[3]) > 0 and len(lowest_distance_and_vector[2]) > 0:
                myWorld.drawPolylines([[lowest_distance_and_vector[2], lowest_distance_and_vector[3]]])
                followLine_lab(robot, v, lowest_distance_and_vector[2] + lowest_distance_and_vector[1], lowest_distance_and_vector[3]  + lowest_distance_and_vector[1], d)
        old_robot_position = robot_position

# Teil a)
#myWorld.setRobot(myRobot, [4, 14, np.deg2rad(210)])
#wander(myRobot, 0.7)

# Teil b)
#myWorld.setRobot(myRobot, [3, 18, np.deg2rad(-40)])
#follow_wall(myRobot, 0.5, 0.6)

# Teil  c)
myWorld.setRobot(myRobot, [4, 14, np.deg2rad(-20)])
get_out(myRobot, 0.5)

# Simulation schliessen:
myWorld.close(False)