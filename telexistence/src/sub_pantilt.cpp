#include "sobit_mini_library/sobit_mini_controller.hpp"
#include "oculus_telexistence/Part.h"
#include <ros/ros.h>
#include <cmath>
#include <iostream>

double sclpan = 2.2;
double scltilt = 1.2;
double scl_velocity_y = 3.0;
double scl_velocity_x = 3.0;
float pan = 0.0;
float tilt = 0.0;
float next_pan = 0.0;
float next_tilt = 0.0;
float pan_v = 0.0;
float tilt_v = 0.0;
void oculussub(const oculus_telexistence::Part::ConstPtr& data) {

    pan = round(data -> Head.rotation.y * sclpan * 10)/10;
    tilt = round(data -> Head.rotation.x * scltilt * 10)/10;
    // pan_v = round(data -> Head.velocity.x * scl_velocity_x * 10)/10;
    // tilt_v = round(data -> Head.velocity.y * scl_velocity_y * 10)/10;

    if(pan >= 1.5) pan = 1.5;
    else if(pan <= -1.5) pan = -1.5;
    if(tilt >= 0.5) tilt = 0.5;
    else if(tilt <= -0.5) tilt = -0.5;
    // if(pan_v < 0) pan_v *= -1;
    // if(tilt_v < 0) tilt_v *= -1;

    // ROS_INFO("Subscribe rotation.x: %f", tilt);
    // ROS_INFO("Subscribe rotation.y: %f", pan);
    ROS_INFO("Subscribe velocity.x: %f", pan_v);
    ROS_INFO("Subscribe velocity.y: %f", tilt_v);

    // ros::Duration(0.5).sleep();
    sobit_mini::SobitMiniController mini_pantilt_ctr;
    if(pan != next_pan || tilt != next_tilt){
        printf("moving\n");
        mini_pantilt_ctr.moveHeadPanTilt(-pan, -tilt, 0.5, false);
        ros::Duration(0.01).sleep();
    }
    next_pan = pan;
    next_tilt = tilt;

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sobit_mini_pantilt_control_oculus_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/oculus/3d", 10, oculussub);
  ros::spin();
}

