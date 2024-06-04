#include "oculus_telexistence/Part.h"
#include <cmath>
#include <iostream>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


class HSRPanTiltControl {
public:
    HSRPanTiltControl() {
        sclpan = 2.2;
        scltilt = 1.2;
        first_pan = 0.0;
        first_tilt = 0.0;
        pan = 0.0;
        tilt = 0.0;
        next_pan = 0.0;
        next_tilt = 0.0;
        targetY = 0.0;
        firstY = 0.0;
        y = 0.0;
        next_y = 0.0;
        pan_v = 0.0;
        tilt_v = 0.0;
        angular = 0;
        orientation_z = 0.0;
        first_odom_z = 0.0;
        z = 0.0;
        flag = false;
        hand_is_left = false;
        tracked = false;
        bag = false;
        orientation_w = 0.0;
    }

    void run() {

        ros::NodeHandle nh;
        nh.getParam("hand", hand_is_left);
        ros::Subscriber sub_oculus = nh.subscribe("/oculus/3d", 10, &HSRPanTiltControl::oculusCallback, this);
        ros::Subscriber sub_odom = nh.subscribe("/hsrb/odom", 10, &HSRPanTiltControl::odomCallback, this);
        ros::Publisher pub_head = nh.advertise<trajectory_msgs::JointTrajectory>("/hsrb/head_trajectory_controller/command", 10);
        ros::Publisher pub_wheel_control = nh.advertise<geometry_msgs::Twist>("/hsrb/command_velocity", 10);

        
        while (pub_head.getNumSubscribers() == 0) {
            ros::Duration(0.1).sleep();
        }

        // make sure the controller is running
        ros::ServiceClient client = nh.serviceClient<controller_manager_msgs::ListControllers>(
            "/hsrb/controller_manager/list_controllers");
        controller_manager_msgs::ListControllers list_controllers;
        bool running = false;
        while (running == false) {
            ros::Duration(0.1).sleep();
            if (client.call(list_controllers)) {
            for (unsigned int i = 0; i < list_controllers.response.controller.size(); i++) {
                controller_manager_msgs::ControllerState c = list_controllers.response.controller[i];
                if (c.name == "head_trajectory_controller" && c.state == "running") {
                running = true;
                }
            }
            }
        }

        while (ros::ok()) {
            if (flag) {
                setting();
                break;
            }
            ros::spinOnce();
        }

        // fill ROS message
        trajectory_msgs::JointTrajectory traj;
        geometry_msgs::Twist twi;

        traj.joint_names.push_back("head_pan_joint");
        traj.joint_names.push_back("head_tilt_joint");
        traj.points.resize(1);
        traj.points[0].positions.resize(2);
        first_pan = -pan;
        first_tilt = tilt;

        while (ros::ok()) {
            ros::spinOnce();
            if (tilt >= 0.5) tilt = 0.5;
            else if (tilt <= -0.5) tilt = -0.5;

            z = first_odom_z - orientation_z;

            traj.points[0].positions[0] = 0;
            traj.points[0].positions[1] = -tilt;
            traj.points[0].velocities.resize(2);
            for (size_t i = 0; i < 2; ++i) {
                traj.points[0].velocities[i] = 0.0;
            }
            traj.points[0].time_from_start = ros::Duration(0.01);
            if (tilt != next_tilt) {
                printf("The head is moving\n");
                pub_head.publish(traj);
                next_tilt = tilt;
            }

            if (bag) pan *= -1;
            
            if (fabs(-pan - orientation_z) >= 0.25) {
                twi.angular.z = fabs(-pan - orientation_z) * ((-pan - orientation_z)/fabs(-pan - orientation_z));
                pub_wheel_control.publish(twi);
            }
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }
        ros::spin();
    }

private:
    double sclpan;
    double scltilt;
    float first_pan;
    float first_tilt;
    float pan;
    float tilt;
    float next_pan;
    float next_tilt;
    float targetY;
    float firstY;
    float y;
    float next_y;
    float pan_v;
    float tilt_v;
    int angular;
    float orientation_z;
    float first_odom_z;
    float z;
    bool flag;
    bool hand_is_left;
    bool tracked;
    bool bag;
    float orientation_w;


    void setting() {
        first_odom_z = orientation_z;
        if (orientation_w > 0) bag = true;
    }

    void oculusCallback(const oculus_telexistence::Part::ConstPtr& data) {
        pan = round(data -> Head.rotation.y * sclpan * 100) / 100;
        tilt = round(data -> Head.rotation.x * scltilt * 100) / 100;
        orientation_w = data -> Head.rotation.z;
        flag = true;
    }
    void odomCallback(const nav_msgs::Odometry::ConstPtr& data) {
        orientation_z = round((((2 * (acos(data -> pose.pose.orientation.w)))* (fabs(data->pose.pose.orientation.z*data->pose.pose.orientation.w)/((data->pose.pose.orientation.z*data->pose.pose.orientation.w)))) - first_odom_z) * 100) / 100;
        if (fabs(orientation_z) >= M_PI) {
            orientation_z -= 2*M_PI* (orientation_z / fabs(orientation_z));
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "develop_hsr_pantilt_tracking_node");

    HSRPanTiltControl pantilt_controller;
    pantilt_controller.run();
    return 0;
}
