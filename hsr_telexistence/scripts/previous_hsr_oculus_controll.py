#!/usr/bin/env python3
# coding: utf-8
import rospy
import trajectory_msgs.msg
import geometry_msgs.msg
from oculus_telexistence.msg import OculusButton
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped


class OCULUS_control:

    def __init__(self):
        self.sub_joy = rospy.Subscriber("oculus/button", OculusButton, self.subscribe_joy, queue_size=10)
        self.sub_joi = rospy.Subscriber("hsrb/joint_states", JointState, self.subscribe_joi, queue_size=10)
        self._wrist_wrench_sub = rospy.Subscriber("/hsrb/wrist_wrench/raw", WrenchStamped, self.__ft_sensor_cb, queue_size=10)
        self.pub_wheel_control = rospy.Publisher("hsrb/command_velocity", geometry_msgs.msg.Twist,queue_size=10)
        self.pub_gripper = rospy.Publisher("/hsrb/gripper_controller/command", trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.pub_hand = rospy.Publisher("main_hand", Bool, queue_size=10)
        self.pub_feedback = rospy.Publisher("hsr_feedback", Bool, queue_size=10)
        self.hand_msg = Bool()
        self.feedback_msg = Bool()
        #rate
        self.rate = rospy.Rate(10)
        #subscriberのメッセージを受け取る変数
        self.joy_button = [0] * 19
        self.left_joystick_lr = 0
        self.left_joystick_ud = 0
        self.right_joystick_lr = 0
        self.right_joystick_ud = 0
        self.triggerLA = 0
        self.triggerRA = 0
        self.gripButtonLA = 0
        self.gripButtonRA = 0
        self.magnifications = 0.3
        self.hand_state = 1.0
        self.new_hand_state = 0.0
        self.hand_is_left = rospy.get_param("hand")
        self.hand_msg.data = self.hand_is_left
        self.hand_motor_state = 0
        self.hand_l_motor_state = 0
        self.new_hand_motor_state = 0
        self._force_data_x = 0
        self._force_data_y = 0
        self._force_data_z = 0
        self._torque_data_x = 0
        self._torque_data_y = 0
        self._torque_data_z = 0

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
        self.triggerLA = msg.TriggerLA.data
        self.triggerRA = msg.TriggerRA.data
        self.gripButtonLA = msg.GripButtonLA
        self.gripButtonRA = msg.GripButtonRA
        self.left_joystick_lr = msg.ThumbstickLA.x * self.magnifications
        self.left_joystick_ud = msg.ThumbstickLA.y * self.magnifications
        self.right_joystick_lr = msg.ThumbstickRA.x * self.magnifications
        self.right_joystick_ud = msg.ThumbstickRA.y * self.magnifications
        self.next_b = 0
    
    def subscribe_joi(self, msg):
        self.hand_motor_state = msg.position[7]
        self.hand_l_motor_state = msg.position[6]

    def __ft_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z
        self._torque_data_x = data.wrench.torque.x
        self._torque_data_y = data.wrench.torque.y
        self._torque_data_z = data.wrench.torque.z

    def check_publishers_connection(self, publisher):
        loop_rate_to_check_connection = rospy.Rate(1)
        while (publisher.get_num_connections() == 0 and not rospy.is_shutdown()):
            try:
                loop_rate_to_check_connection.sleep()
            except rospy.ROSInterruptException:
                pass
    
    def move_wheel(self, linear_x, linear_y):
        twist = geometry_msgs.msg.Twist()
        if self.hand_is_left:
            if self.joy_button[17]:
                twist.linear.z = -self.left_joystick_lr * linear_y
                twist.linear.x = self.left_joystick_ud * linear_x
            self.hand_state = self.triggerLA
        else:
            if self.joy_button[18]:
                twist.linear.y = -self.right_joystick_lr * linear_y
                twist.linear.x = self.right_joystick_ud * linear_x
            self.hand_state = self.triggerRA
        
        if self.hand_state == 0:
            self.hand_state = -1.3
        elif self.hand_state < -0.8:
            self.hand_state = 0.8
        if self.new_hand_state != self.hand_state:
            print("The finger is moving")
            self.grasp(self.hand_state)
        if (self.new_hand_motor_state - self.hand_state < 0):
            self.judge()
        self.check_publishers_connection(self.pub_wheel_control)
        self.pub_wheel_control.publish(twist)
        self.new_hand_state = self.hand_state

    def grasp(self, effort):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["hand_motor_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [-effort]
        p.velocities = [0]
        p.effort = [5.0]
        p.time_from_start = rospy.Duration(0.5)
        traj.points = [p]
        # publish ROS message
        self.pub_gripper.publish(traj)



    def judge(self):
        e = self.hand_motor_state - self.new_hand_motor_state
        # print("")
        if self.hand_motor_state < -0.52 or self.hand_motor_state > 0.9:
            self.feedback_msg.data = False
        elif self.hand_motor_state < -0.2 and abs(e) > 0.01:
            self.feedback_msg.data = True
            self.pub_feedback.publish(self.feedback_msg)
            rospy.sleep(0.5)
            self.feedback_msg.data = False
            self.pub_feedback.publish(self.feedback_msg)
            self.new_hand_motor_state = self.hand_motor_state


    def pub_joy(self):
        while not rospy.is_shutdown():
            self.move_wheel(0.2, 0.2)
            self.rate.sleep()
            self.pub_hand.publish(self.hand_msg)

if __name__ == '__main__':
    rospy.init_node('develop_hsr_oculus_control_node')
    jc = OCULUS_control()
    jc.pub_joy()
    rospy.spin()
