#!/usr/bin/env python3
# coding: utf-8
import rospy
import tf
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from oculus_telexistence.msg import *

class Oculus_control:
    def __init__(self):
        #subscriber
        self.sub_joy = rospy.Subscriber("oculus/button", OculusButton, self.Joy_Callback, queue_size=10)
        #publisher
        self.pub_twist = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=10)
        #rate
        self.rate = rospy.Rate(5)
        #subscriberのメッセージを受け取る変数
        self.joint_state_msg = JointState()     
        self.joy_button = [0] * 19
        self.left_joystick_lr = 0
        self.left_joystick_ud = 0
        self.right_joystick_lr = 0
        self.right_joystick_ud = 0
        self.magnifications = 0.3


    def Joy_Callback(self, msg):
        self.joy_button[0] = msg.ButtonX
        self.joy_button[1] = msg.ButtonA
        self.joy_button[2] = msg.ButtonY
        self.joy_button[3] = msg.ButtonB
        self.joy_button[4] = msg.MenuButton
        self.joy_button[5] = msg.ThumbstickL
        self.joy_button[6] = msg.ThumbstickR
        self.joy_button[7] = msg.ThumbstickLU
        self.joy_button[8] = msg.ThumbstickRU
        self.joy_button[9] = msg.ThumbstickLR
        self.joy_button[10] = msg.ThumbstickRR
        self.joy_button[11] = msg.ThumbstickLD
        self.joy_button[12] = msg.ThumbstickRD
        self.joy_button[13] = msg.ThumbstickLL
        self.joy_button[14] = msg.ThumbstickRL
        self.joy_button[15] = msg.TriggerL
        self.joy_button[16] = msg.TriggerR
        self.joy_button[17] = msg.GripButtonL
        self.joy_button[18] = msg.GripButtonR
        self.left_joystick_lr = msg.ThumbstickLA.x * self.magnifications
        self.left_joystick_ud = msg.ThumbstickLA.y * self.magnifications
        self.right_joystick_lr = msg.ThumbstickRA.x * self.magnifications
        self.right_joystick_ud = msg.ThumbstickRA.y * self.magnifications
        

    def pub_joy(self):
        while(1):
            rospy.loginfo("足を動かすモード")
            twist = Twist()
            twist.linear.x = self.left_joystick_ud * 0.2
            twist.angular.z = self.left_joystick_lr * 0.8
            self.pub_twist.publish(twist)
            rospy.Rate(10).sleep()

if __name__ == '__main__':
    rospy.init_node('sobit_edu_oculus_control_node')
    jc = Oculus_control()
    jc.pub_joy()
    rospy.spin()
