#!/usr/bin/env python3
# coding: utf-8
import rospy
import trajectory_msgs.msg
import geometry_msgs.msg
from oculus_telexistence.msg import *


class OCULUS_control:
    def __init__(self):
        self.sub_joy = rospy.Subscriber("oculus/button", OculusButton, self.subscribe_joy, queue_size=10)
        self.pub_wheel_control = rospy.Publisher('hsrb/command_velocity',geometry_msgs.msg.Twist,queue_size=10)
        self.pub_gripper_control = rospy.Publisher("hsrb/gripper_controller/command", trajectory_msgs.msg.JointTrajectory, queue_size=10)
        
        #rate
        self.rate = rospy.Rate(10)
        #subscriberのメッセージを受け取る変数
        self.joy_button = [0] * 19
        self.left_joystick_lr = 0
        self.left_joystick_ud = 0
        self.right_joystick_lr = 0
        self.right_joystick_ud = 0
        self.magnifications = 0.3
        self.joint_pub_time = 0.001
        self.hand_state = 1.0

    def subscribe_joy(self, msg):
        self.joy_button[0] = msg.ButtonX.data
        self.joy_button[1] = msg.ButtonA.data
        self.joy_button[2] = msg.ButtonY.data
        self.joy_button[3] = msg.ButtonB.data
        self.joy_button[4] = msg.MenuButton.data
        self.joy_button[5] = msg.ThumbstickL.data
        self.joy_button[6] = msg.ThumbstickR.data
        self.joy_button[7] = msg.ThumbstickLU.data
        self.joy_button[8] = msg.ThumbstickRU.data
        self.joy_button[9] = msg.ThumbstickLR.data
        self.joy_button[10] = msg.ThumbstickRR.data
        self.joy_button[11] = msg.ThumbstickLD.data
        self.joy_button[12] = msg.ThumbstickRD.data
        self.joy_button[13] = msg.ThumbstickLL.data
        self.joy_button[14] = msg.ThumbstickRL.data
        self.joy_button[15] = msg.TriggerL.data
        self.joy_button[16] = msg.TriggerR.data
        self.joy_button[17] = msg.GripButtonL.data
        self.joy_button[18] = msg.GripButtonR.data
        self.left_joystick_lr = msg.ThumbstickLA.x * self.magnifications
        self.left_joystick_ud = msg.ThumbstickLA.y * self.magnifications
        self.right_joystick_lr = msg.ThumbstickRA.x * self.magnifications
        self.right_joystick_ud = msg.ThumbstickRA.y * self.magnifications

    def check_publishers_connection(self, publisher):
        loop_rate_to_check_connection = rospy.Rate(1)
        while (publisher.get_num_connections() == 0 and not rospy.is_shutdown()):
            try:
                loop_rate_to_check_connection.sleep()
            except rospy.ROSInterruptException:
                pass
 
    
    def move_wheel(self, linear_x, linear_y, angular):
        twist = geometry_msgs.msg.Twist()
        if self.joy_button[16] == True:
            twist.angular.z = -self.right_joystick_lr * angular
        else:
            twist.linear.x = self.right_joystick_ud * linear_x
            twist.linear.y = -self.right_joystick_lr * linear_y
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["hand_motor_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        if self.joy_button[18]:
            if self.joy_button[1]:
                self.hand_state += 0.1
            else:
                self.hand_state -= 0.1
        p.positions = [self.hand_state]
        p.velocities = [0]
        p.effort = [0.1]
        p.time_from_start = rospy.Duration(0.01)
        traj.points = [p]
        self.check_publishers_connection(self.pub_wheel_control)
        self.pub_wheel_control.publish(twist)
        self.pub_gripper_control.publish(traj)

    def pub_joy(self):
        while not rospy.is_shutdown():
            rospy.loginfo("足を動かすモード")
            self.move_wheel(5.0, 5.0, 5.0)
            self.rate.sleep()




if __name__ == '__main__':
    rospy.init_node('hsr_oculus_control_node')
    jc = OCULUS_control()
    jc.pub_joy()
    rospy.spin()
