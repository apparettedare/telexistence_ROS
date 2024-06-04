#include "sobit_mini_library/sobit_mini_controller.hpp"
#include "oculus_telexistence/Part.h"
#include <ros/ros.h>
#include <cmath>
#include <iostream>

class SobitMiniPanTiltControl {
public:
    SobitMiniPanTiltControl() {
        sclpan = 2.2;
        scltilt = 1.2;
        scl_velocity_y = 3.0;
        scl_velocity_x = 3.0;
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
        ros::Subscriber sub = nh.subscribe("/oculus/3d", 10, &SobitMiniPanTiltControl::oculusCallback, this);
        flag = false;
        while (ros::ok()) {
            ros::spinOnce();
            if (flag) {
                break;
            }
            ros::spinOnce();
        }

        ros::spinOnce();
        first_pan = pan;
        first_tilt = tilt;
        while (ros::ok()) {
            ros::spinOnce();
            if (pan >= 1.5) pan = 1.5;
            else if (pan <= -1.5) pan = -1.5;
            if (tilt >= 0.5) tilt = 0.5;
            else if (tilt <= -0.5) tilt = -0.5;
            ros::spinOnce();

            sobit_mini::SobitMiniController mini_pantilt_ctr;
            if (pan != next_pan || tilt != next_tilt) {
                printf("moving\n");
                ros::spinOnce();
                mini_pantilt_ctr.moveHeadPanTilt(pan, tilt, 0.5, false);
                ros::Duration(0.01).sleep();
            }
            ros::spinOnce();
            next_pan = pan;
            next_tilt = tilt;
        }
        ros::spin();
    }

private:
    double sclpan;
    double scltilt;
    double scl_velocity_y;
    double scl_velocity_x;
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
        tilt = round(data -> Head.rotation.x * scltilt * 10) / 10;
        flag = true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "sobit_mini_pantilt_control_oculus_node");

    SobitMiniPanTiltControl pantilt_controller;
    pantilt_controller.run();
    return 0;
}
