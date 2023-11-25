#!/usr/bin/env python3
import rospy
import math
import csv
import rospkg
import datetime
import numpy as np
from oculus_telexistence.msg import Part
from trajectory_msgs.msg import JointTrajectory

x = 0
y = 0
z = 0
traj = JointTrajectory()
flag = False
flag_occulus = False
flag_trajnt_states = False

def callback_occulus(msg):
    global x, y, z, flag, flag_occulus
    # x = int(msg.Left.position.z * 100) / 100
    # z = int(msg.Left.position.y * 100) / 100
    # y = int(msg.Left.rotation.x * 100) / 100 * 2

    x = msg.Left.position.z
    z = msg.Left.position.y
    if flag:
        y = -msg.Left.rotation.x * 2
    else:
        y = msg.Left.rotation.x * 2
    flag_occulus = True


def callback_trajnt_states(msg):
    global traj, flag_trajnt_states
    traj = msg
    flag_trajnt_states = True


def main():
    global x, y, z, traj, flag, flag_occulus, flag_trajnt_states
    arm_flex_to_arm_roll = 0.345
    wrist_to_hand = 0.192
    rospy.init_node("listener")
    # rospack = rospkg.RosPack()
    # package_path = rospack.get_path("oculus_telexistence")
    t_now = datetime.datetime.now()
    file_name =  "/home/sobits/catkin_ws/src/csv/previous/result_" + str(t_now.year) + "_" + str(t_now.month) + "_" + str(t_now.hour) + "_" + str(t_now.minute) + "_" + str(t_now.second) + ".csv"
    r = rospy.Rate(100)
    rospy.Subscriber("/oculus/3d", Part, callback_occulus)
    rospy.Subscriber("/hsrb/arm_trajectory_controller/command", JointTrajectory, callback_trajnt_states)
    f = open(file_name, "a")
    f.write("occulus_x" + "," + "occulus_z" + "," + "oculus_theta_y" + "," + "hsr_x" + "," + "hsr_z" + "," + "hsr_theta_y" + "\n")
    f.close()
    while not rospy.is_shutdown():
        if ((flag_occulus) and (flag_trajnt_states)):
            break
    if(y < 0):
        flag = True
    while not rospy.is_shutdown():
        # hsr_x = arm_flex_to_arm_roll * np.sin(int(traj.position[1] * 100) / 100) + wrist_to_hand * np.sin(int((traj.position[1] + traj.position[2]) * 100) / 100)
        # hsr_z = 0.339 + int(traj.position[0] * 100) / 100 + arm_flex_to_arm_roll * np.cos(int(traj.position[1] * 100) / 100) + wrist_to_hand * np.cos(int((traj.position[1] + traj.position[2]) * 100) / 100)
        # hsr_theta_y = int((traj.position[1] + traj.position[2]) * 100) / 100
        if(x > (arm_flex_to_arm_roll + wrist_to_hand)):
            x = arm_flex_to_arm_roll + wrist_to_hand
        elif(x < 0):
            x = 0
        if(z > (0.339 + 0.69 + arm_flex_to_arm_roll + wrist_to_hand)):
            z = 0.339 + 0.69 + arm_flex_to_arm_roll + wrist_to_hand
        elif(z < 0):
            z = 0
        if(y < -1):
            y = -1
        elif(y > 1):
            y = 1
        hsr_x = arm_flex_to_arm_roll * np.sin(-traj.points[0].positions[1]) + wrist_to_hand * np.sin(-traj.points[0].positions[1] - traj.points[0].positions[3])
        hsr_z = 0.339 + traj.points[0].positions[0] + arm_flex_to_arm_roll * np.cos(traj.points[0].positions[1]) + wrist_to_hand * np.cos(-traj.points[0].positions[1] - traj.points[0].positions[3])
        hsr_theta_y = -(traj.points[0].positions[1] + traj.points[0].positions[3])
        oculus_theta__y = math.pi - math.acos(y)
        f = open(file_name, "a")
        f.write(str(x) + "," + str(z) + "," + str(oculus_theta__y) + "," + str(hsr_x) + "," + str(hsr_z) + "," + str(hsr_theta_y) + "\n")
        f.close()
        r.sleep()


if __name__ == '__main__':
    main()