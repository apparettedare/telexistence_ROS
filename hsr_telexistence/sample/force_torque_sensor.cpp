#include <cstdlib>
#include <math.h>
#include <string>

#include <boost/format.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tmc_control_msgs/GripperApplyEffortAction.h>
#include <tmc_control_msgs/GripperApplyEffortGoal.h>
#include <tmc_manipulation_msgs/SafeJointChange.h>
#include <tmc_msgs/TalkRequestAction.h>
#include <tmc_msgs/TalkRequestGoal.h>
#include <tmc_msgs/Voice.h>


double ComputeDifference(const geometry_msgs::Vector3& pre_data,
                         const geometry_msgs::Vector3& post_data) {
  // Calculate square sum of difference
  double square_sums = pow(post_data.x - pre_data.x, 2) +
    pow(post_data.y - pre_data.y, 2) + pow(post_data.z - pre_data.z, 2);
  return sqrt(square_sums);
}


class ForceSensorCapture {
 public:
  ForceSensorCapture() : force_data_() {
    // Subscribe force torque sensor data from HSRB
    ros::NodeHandle nh;
    std::string ft_sensor_topic = "/hsrb/wrist_wrench/raw";
    wrist_wrench_sub_ = nh.subscribe(ft_sensor_topic, 1,
                                     &ForceSensorCapture::FtSensorCb,
                                     this);
    const double kConnectionTimeout = 10.0;
    // Wait for connection
    if (!ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(
        ft_sensor_topic, ros::Duration(kConnectionTimeout))) {
      ROS_ERROR("timeout exceeded while waiting for message on topic %s",
                ft_sensor_topic.c_str());
      exit(EXIT_FAILURE);
    }
  }

  void GetCurrentForce(geometry_msgs::Vector3& force_data) const {
    // Spin FtSensorCb function once
    ros::spinOnce();
    force_data = force_data_;
  }

 private:
  ros::Subscriber wrist_wrench_sub_;
  geometry_msgs::Vector3 force_data_;

  void FtSensorCb(const geometry_msgs::WrenchStampedConstPtr& data) {
    force_data_ = data->wrench.force;
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "hsrb_force_torque_sensor");

  // Start force sensor capture
  ForceSensorCapture force_sensor_capture;

  // Get data of force sensor
  geometry_msgs::Vector3 force;
  while(ros::ok) {
        ros::Duration(0.1).sleep();
        force_sensor_capture.GetCurrentForce(force);
        std::cout << "force: "<< force << std::endl;
  }
  return 0;
}
