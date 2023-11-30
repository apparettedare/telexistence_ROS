#include "oculus_telexistence/Part.h"
#include <cmath>
#include <iostream>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

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
        pan_v = 0.0;
        tilt_v = 0.0;
        flag = false;
    }

    void run() {

        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe("/oculus/3d", 10, &HSRPanTiltControl::oculusCallback, this);
        ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/hsrb/head_trajectory_controller/command", 10);
        
        while (pub.getNumSubscribers() == 0) {
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

        // fill ROS message
        trajectory_msgs::JointTrajectory traj;

        traj.joint_names.push_back("head_pan_joint");
        traj.joint_names.push_back("head_tilt_joint");
        traj.points.resize(1);
        traj.points[0].positions.resize(2);
        first_pan = -pan;
        first_tilt = tilt;

        while (ros::ok()) {
            ros::spinOnce();
            if (pan >= 1.5) pan = 1.5;
            else if (pan <= -1.5) pan = -1.5;
            if (tilt >= 0.5) tilt = 0.5;
            else if (tilt <= -0.5) tilt = -0.5;
            traj.points[0].positions[0] = -pan;
            traj.points[0].positions[1] = -tilt;
            traj.points[0].velocities.resize(2);
            for (size_t i = 0; i < 2; ++i) {
                traj.points[0].velocities[i] = 0.0;
            }
            traj.points[0].time_from_start = ros::Duration(0.01);
            if (tilt != next_tilt) {
                printf("The head is moving\n");
                pub.publish(traj);
                next_tilt = tilt;
            }
            ros::Duration(0.01).sleep();
            ros::spinOnce();
            next_pan = pan;
            
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
    float pan_v;
    float tilt_v;
    bool flag;

    void oculusCallback(const oculus_telexistence::Part::ConstPtr& data) {
        pan = round(data -> Head.rotation.y * sclpan * 10) / 10;
        tilt = round(data -> Head.rotation.x * scltilt * 100) / 100;
        flag = true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "previous_hsr_pantilt_tracking_node");

    HSRPanTiltControl pantilt_controller;
    pantilt_controller.run();
    return 0;
}