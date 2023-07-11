#!/usr/bin/env python3
# coding: utf-8
import rospy
from oculus_telexistence.msg import *

def callback(msg):
    rospy.loginfo("Subscribe : " + str(msg.Head.position.x))

def main():
    rospy.init_node("unity_oculus_position")
    rospy.Subscriber("oculus/3d", Part, callback)
    rospy.spin()

if __name__ == "__main__":
    main()