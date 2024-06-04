#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "oculus_telexistence/OculusButton.h"


class OCULUS_control:
    def __init__(self):
        self.sub_joy = rospy.Subscriber("oculus/button", OculusButton, self.subscribe_joy, queue_size=10)
        self.pub_wheel_control = rospy.Publisher('/cmd_vel_mux/input/navi',geometry_msgs.msg.Twist,queue_size=10)
        #rate
        self.rate = rospy.Rate(10)
        #subscriberのメッセージを受け取る変数
        self.joy_button = [0] * 19;
        self.left_joystick_lr = 0
        self.left_joystick_ud = 0
        self.right_joystick_lr = 0
        self.right_joystick_ud = 0
        self.magnifications = 0.3
        self.joint_pub_time = 0.001

    void oculus_sub(const oculus_telexistence::OculusButton::ConstPtr& msg) {
        self.joy_button[0] = msg.ButtonX;
        self.joy_button[1] = msg.ButtonA;
        self.joy_button[2] = msg.ButtonY;
        self.joy_button[3] = msg.ButtonB;
        self.joy_button[4] = msg.MenuButton;
        self.joy_button[5] = msg.ThumbstickL;
        self.joy_button[6] = msg.ThumbstickR;
        self.joy_button[7] = msg.ThumbstickLU;
        self.joy_button[8] = msg.ThumbstickRU;
        self.joy_button[9] = msg.ThumbstickLR;
        self.joy_button[10] = msg.ThumbstickRR;
        self.joy_button[11] = msg.ThumbstickLD;
        self.joy_button[12] = msg.ThumbstickRD;
        self.joy_button[13] = msg.ThumbstickLL;
        self.joy_button[14] = msg.ThumbstickRL;
        self.joy_button[15] = msg.TriggerL;
        self.joy_button[16] = msg.TriggerR;
        self.joy_button[17] = msg.GripButtonL;
        self.joy_button[18] = msg.GripButtonR;
        self.left_joystick_lr = msg.ThumbstickLA.x * self.magnifications;
        self.left_joystick_ud = msg.ThumbstickLA.y * self.magnifications;
        self.right_joystick_lr = msg.ThumbstickRA.x * self.magnifications;
        self.right_joystick_ud = msg.ThumbstickRA.y * self.magnifications;
    }

    void move_wheel(self, liner, angular):
        geometry_msgs::Twist twist;
        twist.linear.x = self.left_joystick_ud * liner;
        twist.angular.z = self.left_joystick_lr * angular;
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
            self.move_wheel(0.2, 0.8)
            self.rate.sleep()




if __name__ == '__main__':
    rospy.init_node('sobit_mini_oculus_control_node')
    jc = OCULUS_control()
    jc.pub_joy()
    rospy.spin()
int main(int argc, char **argv) {
    ros::init(argc, argv, "sobit_mini_oculus_control_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/oculus/button", 10, oculussub);
    ros::spin();
}