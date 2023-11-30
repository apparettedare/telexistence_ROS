#!/usr/bin/env python3
import rospy
import math
import csv
import datetime
import numpy as np
from oculus_telexistence.msg import Part
from trajectory_msgs.msg import JointTrajectory

x = 0
y = 0
z = 0
head_z = 0
y_rotation = 0
traj = JointTrajectory()
flag = False
flag_oculus = False
flag_traject_states = False
hand_is_left = False
first_hand_position = 0

def callback_occulus(msg):
    global x, y, z, head_z, y_rotation, first_hand_position, flag, flag_oculus, hand_is_left
    if hand_is_left:
        x = msg.Left.position.z
        y = msg.Left.position.x - first_hand_position
        z = msg.Left.position.y
        if flag == True:
            y_rotation = -msg.Left.rotation.x * 2
        else:
            y_rotation = msg.Left.rotation.x * 2
    else:
        x = msg.Right.position.z
        y = msg.Right.position.x + first_hand_position
        z = msg.Right.position.y
        if flag == True:
            y_rotation = -msg.Right.rotation.x * 2
        else:
            y_rotation = msg.Right.rotation.x * 2   
    head_z = msg.Head.position.y
    flag_oculus = True

def callback_traject_states(msg):
    global traj, flag_traject_states
    traj = msg
    flag_traject_states = True

def main():
    global x, y, z, head_z, y_rotation, first_hand_position, traj, flag, flag_oculus, flag_traject_states, hand_is_left
    arm_flex_to_arm_roll = 0.345
    wrist_to_hand = 0.192
    rospy.init_node("develop_listener")
    hand_is_left = rospy.get_param("hand")
    t_now = datetime.datetime.now()
    file_name =  "/home/sobits/catkin_ws/src/csv/develop/develop_" + str(t_now.year) + "_" + str(t_now.month) + "_" + str(t_now.hour) + "_" + str(t_now.minute) + "_" + str(t_now.second) + ".csv"
    r = rospy.Rate(100)
    rospy.Subscriber("/oculus/3d", Part, callback_occulus)
    rospy.Subscriber("/hsrb/arm_trajectory_controller/command", JointTrajectory, callback_traject_states)
    f = open(file_name, "a")
    f.write("occulus_hand_x" + "," + "occulus_hand_z" + "," + "oculus_theta_y" + "," + "oculus_head_z" + "," + "hsr_hand_x" + "," + "hsr_hand_z" + "," + "hsr_theta_y" + "," + "hsr_head_z" + "\n")
    f.close()
    while not rospy.is_shutdown():
        if ((flag_oculus) and (flag_traject_states)):
            break
    if(y_rotation > 0):
        flag = True
    first_hand_position = y
    while not rospy.is_shutdown():
        xx = math.sqrt((x) ** 2 + (y) ** 2)
        if(xx > (arm_flex_to_arm_roll + wrist_to_hand)):
            xx = arm_flex_to_arm_roll + wrist_to_hand
        elif(xx < 0):
            xx = 0
        if(z > (0.339 + 0.69 + arm_flex_to_arm_roll + wrist_to_hand)):
            z = 0.339 + 0.69 + arm_flex_to_arm_roll + wrist_to_hand
        elif(z < 0):
            z = 0
        if(y < -1):
            y = -1
        elif(y > 1):
            y = 1
        hsr_x = arm_flex_to_arm_roll * np.sin(-traj.points[0].positions[1]) + wrist_to_hand * np.sin(-traj.points[0].positions[1] - traj.points[0].positions[3])
        hsr_z = 0.339 + traj.points[0].positions[0] + arm_flex_to_arm_roll * np.cos(-traj.points[0].positions[1]) + wrist_to_hand * np.cos(-traj.points[0].positions[1] - traj.points[0].positions[3])
        hsr_theta_y = -(traj.points[0].positions[1] + traj.points[0].positions[3])
        oculus_theta_y = math.pi - math.acos(y_rotation)
        hsr_head_z = 0.89 + traj.points[0].positions[0]
        f = open(file_name, "a")
        f.write(str(xx) + "," + str(z) + "," + str(oculus_theta_y) + "," + str(head_z) + "," + str(hsr_x) + "," + str(hsr_z) + "," + str(hsr_theta_y) + "," + str(hsr_head_z) + "\n")
        f.close()
        r.sleep()


if __name__ == '__main__':
    main()