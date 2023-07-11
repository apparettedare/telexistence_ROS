#!/usr/bin/env python3
# coding: utf-8
import rospy
from oculus_telexistence.msg import *

def callback(msg):
    rospy.loginfo("Subscribe : " + str(msg.ButtonX))

def main():
    rospy.init_node("unity_oculus_button")
    rospy.Subscriber("oculus/button", OculusButton, callback)
    rospy.spin()

if __name__ == "__main__":
    main()