#!/usr/bin/env python

import rospy
from control_msgs.msg import JointControllerState
import pandas as pd
import sys

df = pd.DataFrame(columns=['cur_pos', 'cur_vel', 'error', 'goal_pos', 'target'])
counter = 0
file_name = '/home/daniel/manipulator_ws/src/pi_robot_pkg/data/'

def dataCallback(msg):
    global df
    global counter
    current_pos = msg.process_value
    current_vel = msg.process_value_dot
    goal = msg.set_point
    error = msg.error
    command = msg.command
    df.loc[counter] = [current_pos, current_vel, error, goal, command]
    counter += 1

def shutdown_callback():
    global file_name
    global df
    df.to_csv(file_name)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        rospy.logerr('Name of data file needed')
        exit()
    file_name = file_name + sys.argv[1] + '.csv'

    rospy.init_node('data_collector')    
    sub = rospy.Subscriber('/pi_robot/head_pan_joint_position_controller/state', JointControllerState, dataCallback)

    rospy.on_shutdown(shutdown_callback)
    rospy.spin()