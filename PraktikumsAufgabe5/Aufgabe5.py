import math

from numpy import pi

from Robot_Simulator_V3 import twoRoomsWorld
from Robot_Simulator_V3 import Robot
import numpy as np

def main():
    my_world = twoRoomsWorld.buildWorld()
    my_robot = Robot.Robot()

    my_world.setRobot(my_robot, [10.7, 11, np.deg2rad(90)])

    points_room1_to_depot = [np.array([5, 2.75]), np.array([11, 2.75]), np.array([11, 9.75]), np.array([16, 9.75]), np.array([16, 12.5])]
    points_room2_to_depot = [np.array([11, 9.75]), np.array([16, 9.75]), np.array([16, 12.5])]
    points_depot_to_room1 = [np.array([16, 12.5]), np.array([16, 9.75]), np.array([11, 9.75]), np.array([11, 2.75]), np.array([5, 2.75])]
    points_depot_to_room2 = [np.array([16, 12.5]), np.array([16, 9.75]), np.array([11, 9.75])]
    points_room1_to_room2 = [np.array([5, 2.75]), np.array([11, 2.75])]
    points_room2_to_room1 = [np.array([11, 2.75]), np.array([5, 2.75])]

    robot_speed = 0.4
    count_room1 = 0
    count_room2 = 0
    room_threshold = 500

    while True:
        box_dirs = my_robot.senseBoxes()
        dists = my_robot.sense()
        directions = my_robot.getSensorDirections()

        robot_pose = my_robot._world.getTrueRobotPose()
        robot_position = np.array([robot_pose[0], robot_pose[1]])
        if room_of_point(robot_position) == 3:
            follow_points(my_robot, robot_speed, points_depot_to_room1)

        if count_room1 == room_threshold:
            count_room1 = 0
            count_room2 = 0
            print("Room change")
            follow_points(my_robot, robot_speed, points_room1_to_room2)
        elif count_room2 == room_threshold:
            count_room1 = 0
            count_room2 = 0
            print("Room change")
            follow_points(my_robot, robot_speed, points_room2_to_room1)

        if box_dirs is not None:
            distance_to_box = box_dirs[0][0]
            rotate_to_box(my_robot)
            drive_straight(my_robot, robot_speed, distance_to_box * 0.95)
            rotate_to_box(my_robot)

            point_to_return_to = []
            if my_robot.boxInPickUpPosition():
                my_robot.pickUpBox()
                robot_pose = my_robot._world.getTrueRobotPose()
                point_to_return_to = np.array([robot_pose[0], robot_pose[1]])
            else:
                continue

            route = []
            back_route = []
            room = room_of_point(point_to_return_to)
            if room == 1:
                route = points_room1_to_depot
                back_route = points_depot_to_room1
            elif room == 2:
                route = points_room2_to_depot
                back_route = points_depot_to_room2
            follow_points(my_robot, robot_speed, route)
            my_robot.placeBox()
            follow_points(my_robot, robot_speed, back_route)
            go_to_global(my_robot, robot_speed, point_to_return_to)
            count_room1 = 0
            count_room2 = 0
        else:
            if room_of_point(robot_position) == 1:
                count_room1 = count_room1 + 1
            elif room_of_point(robot_position) == 2:
                count_room2 = count_room2 + 2
            wander(my_robot, robot_speed, dists, directions)

def room_of_point(point):
    if point[0] < 7.8:
        return 1
    elif point[0] < 14.8:
        return 2
    else:
        return 3

def rotate_to_box(robot):
    if robot.senseBoxes() is None:
        return
    while not 0.03 > robot.senseBoxes()[0][1] > -0.03:
        angle_to_correct_with = -robot.senseBoxes()[0][1]
        robot.move([0, angle_to_correct_with])

def wander(robot, v, dists, directions):
    min = [-1, 10]
    for i in range(0, len(dists) - 1):
        if dists[i]:
            if dists[i] < min[1]:
                min = [i, dists[i]]
    if min[0] == -1:
        robot.move([v, 0])
    if min[1] < 1 * (v - v / 6) or min[1] < 0.5:
        if (directions[min[0]]) <= 0:
            robot.move([v / 6, +pi])
        else:
            robot.move([v / 6, -pi])
    robot.move([v, 0])

def follow_points(robot, v, points):
    for i in range(len(points)):
        go_to_global(robot, v, points[i])

def go_to_global(robot, v, p, tol = 0.1):
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

def drive_straight(robot, v, l):
    # Bewege Roboter
    # v = m/s
    # l = m
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

def get_poly_line(points):
    polyline = []
    for index, point in enumerate(points):
        polyline.append(np.array([points[index], points[index+1]]))
        if index + 1 == len(points) - 1:
            break
    return polyline

if __name__ == "__main__":
    main()
