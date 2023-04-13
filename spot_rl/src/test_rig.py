#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Imu
import numpy as np
from gazebo_msgs.msg import ModelStates


def callback(data):
    ranges = list(data.ranges)
    number_of_readings = len(ranges)
    distance_threshold = 1.0
    range_max = data.range_max

    for i in range(len(ranges)):
        if ranges[i] == float("inf"):
            ranges[i] = range_max
    ranges = tuple(ranges)

    # print("middle", str(np.mean(ranges[len(ranges)*3/8:len(ranges)*5/8])))
    # print("left", str(np.mean(ranges[len(ranges)/4:len(ranges)*3/8])))
    # print("right", str(np.mean(ranges[len(ranges)*5/8:len(ranges)*3/4])))
    # print("small middle", str(np.mean(ranges[len(ranges)*7/16:len(ranges)*9/16]))) # small middle
    
    for i, value in enumerate(ranges[int(len(ranges)/4):int(len(ranges)*3/4)]):
        if value < 0.5:
            # print(i)
            pass

    observation = []
    observation[0:len(ranges[int(len(ranges)/4):int(len(ranges)*3/4)])] = ranges[int(len(ranges)/4):int(len(ranges)*3/4)]
    observation[-5] = 1.0
    observation[-4] = 1.0
    observation[-3] = 1.0
    observation[-2] = 1.0
    observation[-1] = 1.0
    # print(len(observation))

def imu_callback(data):
    # print(data.orientation.x, data.orientation.y)
    pass

def state_callback(data):
        # Get the index of Spot's position in the message
        spot_idx = data.name.index('/')

        goal_position = 3.524, 2.096

        # Get the x and y position of Spot
        x = data.pose[spot_idx].position.x
        y = data.pose[spot_idx].position.y

        # Save the position as a tuple
        robot_position = (x, y)

        waypoints = [(3.0113, -2.8049), (3.524, 2.096), (-1.367888, -3.131377)]
        
        distance_from_goal = np.sqrt((goal_position[0] - robot_position[0])**2 + (goal_position[1] - robot_position[1])**2)

        print(distance_from_goal)

if __name__ == '__main__':
    rospy.init_node('laser_scan_subscriber')
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.Subscriber('/gazebo/model_states', ModelStates, state_callback)
    rospy.Subscriber('/imu/data', Imu, imu_callback)
    rospy.spin()