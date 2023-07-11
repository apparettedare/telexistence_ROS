#!/usr/bin/env python3
# coding: utf-8
import rospy
import tf
import sensor_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
from oculus_telexistence.msg import *


class OCULUS_control:
    def __init__(self):
        self.sub_joy = rospy.Subscriber("oculus/button", OculusButton, self.subscribe_joy, queue_size=10)
        self.pub_body_control = rospy.Publisher('/body_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.pub_head_control = rospy.Publisher('/head_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.pub_left_arm_control = rospy.Publisher('/left_arm_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.pub_right_arm_control = rospy.Publisher('/right_arm_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.pub_wheel_control = rospy.Publisher('/mobile_base/commands/velocity',geometry_msgs.msg.Twist,queue_size=10)
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

    def subscribe_joy(self, msg):
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

    def check_publishers_connection(self, publisher):
        loop_rate_to_check_connection = rospy.Rate(1)
        while (publisher.get_num_connections() == 0 and not rospy.is_shutdown()):
            try:
                loop_rate_to_check_connection.sleep()
            except rospy.ROSInterruptException:
                pass

    def move_body_joint(self, joint_name, rad, time_from_start):
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(rad)
        point.time_from_start = rospy.Duration(time_from_start)
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names.append(joint_name)
        traj.points.append(point)
        self.check_publishers_connection(self.pub_body_control)
        self.pub_body_control.publish(traj)
    
    def move_head_joint(self, joint_name, rad, time_from_start):
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(rad)
        point.time_from_start = rospy.Duration(time_from_start)
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names.append(joint_name)
        traj.points.append(point)
        self.check_publishers_connection(self.pub_head_control)
        self.pub_head_control.publish(traj)

    def move_left_arm_joint(self, joint_name, rad, time_from_start):
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(rad)
        point.time_from_start = rospy.Duration(time_from_start)
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names.append(joint_name)
        traj.points.append(point)
        self.check_publishers_connection(self.pub_left_arm_control)
        self.pub_left_arm_control.publish(traj)

    def move_right_arm_joint(self, joint_name, rad, time_from_start):
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(rad)
        point.time_from_start = rospy.Duration(time_from_start)
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names.append(joint_name)
        traj.points.append(point)
        self.check_publishers_connection(self.pub_right_arm_control)
        self.pub_right_arm_control.publish(traj)
    
    def move_wheel(self, liner, angular):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = self.left_joystick_ud * liner
        twist.angular.z = self.left_joystick_lr * angular * -1
        self.check_publishers_connection(self.pub_wheel_control)
        self.pub_wheel_control.publish(twist)

    def reset_joint(self):
        self.move_body_joint("body_lift_joint", 0.05, self.joint_pub_time)
        self.move_body_joint("body_roll_joint", 0.0, self.joint_pub_time)
        self.move_head_joint("head_tilt_joint", 0.0, self.joint_pub_time)
        self.move_head_joint("head_pan_joint", 0.0, self.joint_pub_time)
        self.move_right_arm_joint("right_shoulder_roll_joint", 0.0, self.joint_pub_time)
        self.move_right_arm_joint("right_shoulder_flex_joint", 0.40, self.joint_pub_time)
        self.move_right_arm_joint("right_wrist_flex_joint", 0.0, self.joint_pub_time)
        self.move_right_arm_joint("right_hand_motor_joint", 0.0, self.joint_pub_time)
        self.move_left_arm_joint("left_shoulder_roll_joint", 0.0, self.joint_pub_time)
        self.move_left_arm_joint("left_shoulder_flex_joint", -0.4, self.joint_pub_time)
        self.move_left_arm_joint("left_wrist_flex_joint", 0.0, self.joint_pub_time)
        self.move_left_arm_joint("left_hand_motor_joint", 0.0, self.joint_pub_time)
    def pub_joy(self):
        while not rospy.is_shutdown():
            rospy.loginfo("足を動かすモード")
            self.move_wheel(0.5, 3.0)
            self.rate.sleep()




if __name__ == '__main__':
    rospy.init_node('sobit_mini_oculus_control_node')
    jc = OCULUS_control()
    jc.pub_joy()
    rospy.spin()
