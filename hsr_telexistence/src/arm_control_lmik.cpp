#include "oculus_telexistence/Part.h"
#include <ros/ros.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class Arm_controll {
public:
    Arm_controll() {
    // ジョイントの角度
    arm_lift_joint = 0.01;
    arm_flex_joint = 0.01;
    arm_roll_joint = 0.01;
    wrist_flex_joint = 0.01;
    wrist_roll_joint = 0.01;
    next_arm_lift_joint = 0.0;
    next_arm_flex_joint = 0.0;
    next_arm_roll_joint = 0.0;
    next_wrist_flex_joint = 0.0;
    next_wrist_roll_joint = 0.0;

    // 目標位置
    targetX = 0.0;
    targetY = 0.0;
    targetZ = 0.0;
    rotationX = 0.0;
    rotationY = 0.0;
    rotationZ = 0.0;
    first_position_x = 0.0;
    first_position_z = 0.0;

    // リンクの長さ
    head_to_arm_lift = 0.412;   
    arm_flex_to_arm_roll = 0.345;
    wrist_to_hand = 0.192;
    arm_lift_to_arm_flex = 0.141;
    wrist_to_hand_2 = 0.029;

    tracked = false;
    deltaX = 0.0;
    deltaY = 0.0;
    deltaZ = 0.0;

    // 逆運動学の解を格納する変数
    delta_arm_lift = 0.0;
    delta_arm_flex = 0.0;
    delta_arm_roll = 0.0;
    delta_wrist_flex = 0.0;
    delta_wrist_roll = 0.0;
    
    W_N_bar = Matrix2d::Identity(2, 2) * 0.001;  // bias of dumping factor 
    W_E = Matrix2d::Identity(2, 2);              // weigth of error
    // 収束条件
    threshold = 0.000001; 
    
    }

    void run() {
        ros::NodeHandle nh;
        ros::Subscriber oculus_sub = nh.subscribe("/oculus/3d", 10, &Arm_controll::oculusCallback, this);
        ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
        ros::Time start_time = ros::Time::now();

        // wait to establish connection between the controller
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
                if (c.name == "arm_trajectory_controller" && c.state == "running") {
                running = true;
                }
            }
            }
        }
        purike();

        while (ros::ok()) {
            Vector3d q;
            Vector2d target_pos;
            q << arm_lift_joint, arm_flex_joint, wrist_flex_joint;
            target_pos << targetX, targetZ;
            if(tracked) solve(q, target_pos);
            Kinematics();
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        ros::spin();
    }
private:
    // ジョイントの角度
    double arm_lift_joint;
    double arm_flex_joint;
    double arm_roll_joint;
    double wrist_flex_joint;
    double wrist_roll_joint;
    double next_arm_lift_joint;
    double next_arm_flex_joint;
    double next_arm_roll_joint;
    double next_wrist_flex_joint;
    double next_wrist_roll_joint;
    // 目標位置
    double targetX;
    double targetY;
    double targetZ;
    double rotationX;
    double rotationY;
    double rotationZ;
    double first_position_x;
    double first_position_z;

    // リンクの長さ
    double head_to_arm_lift;   
    double arm_flex_to_arm_roll;
    double wrist_to_hand;

    // etc
    double arm_lift_to_arm_flex;
    double wrist_to_hand_2;

    double tracked;
    double deltaX;
    double deltaY;
    double deltaZ;

    // 逆運動学の解を格納する変数
    double delta_arm_lift;
    double delta_arm_flex;
    double delta_arm_roll;
    double delta_wrist_flex;
    double delta_wrist_roll;
    
    Matrix2d W_N_bar; 
    Matrix2d W_E;
    // 収束条件
    double threshold; 

    // 残差を計算する関数
    void cal_e(Vector2d& e, Vector3d& q, Vector2d& target_pos) {
        // 目標位置と現在の位置の差分を計算
        e(0) = target_pos(0) * (arm_lift_to_arm_flex + wrist_to_hand) / first_position_x - (arm_lift_to_arm_flex + arm_flex_to_arm_roll * sin(q(1)) + wrist_to_hand * sin(q(1) + q(2)));
        e(1) = target_pos(1) * (head_to_arm_lift - arm_flex_to_arm_roll + wrist_to_hand_2) / -first_position_z + (-arm_lift_joint + head_to_arm_lift - arm_flex_to_arm_roll * cos(q(1)) - wrist_to_hand * cos(q(1) + q(2)));
        return;
    }

    // ヤコビ行列を計算する関数
    void cal_J(MatrixXd& J, Vector3d& q) {
        J(0,0) = 0;
        J(0,1) = arm_flex_to_arm_roll * cos(q(1)) + wrist_to_hand * cos(q(1) + q(2));
        J(0,2) =                                    wrist_to_hand * cos(q(1) + q(2));
        J(1,0) = 1;
        J(1,1) = - arm_flex_to_arm_roll * sin(q(1)) - wrist_to_hand * sin(q(1) + q(2));
        J(1,2) =                                    - wrist_to_hand * sin(q(1) + q(2));
        return;
    }

    // 
    void cal_g(Vector2d& g, const MatrixXd& J, const Vector2d& e) {
        g = J.transpose() * W_E * e;
        return;
    }

    void cal_W_N(Matrix2d &W_N, const Vector2d &e) {
        W_N = MatrixXd::Identity(2, 3) * e.squaredNorm();
        W_N += W_N_bar;
        return;
    }

    void cal_H(Matrix2d &H, const MatrixXd &J, const Matrix2d &W_N){
        H = J.transpose() * W_E * J + W_N;
        return;
    }

    // 逆運動学を行う関数
    void solve(Vector3d& q_ref, Vector2d& target_pos) {
        
        // ヤコビ行列
        MatrixXd J(2,3);
        Matrix2d W_N, H;
        Vector2d e, g;
        Vector3d q, q_new, delta_q;
        q = q_ref;
        
        ros::NodeHandle nh;
        ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
        // 逆運動学の反復計算
        do {
            // 残差を計算
            cal_e(e, q, target_pos);
            // ヤコビ行列を計算
            cal_J(J, q);

            cal_g(g, J, e);

            cal_W_N(W_N, e);

            cal_H(H, J, W_N);

            // 角度の変化量を計算
            delta_q = H.inverse() * g;
            // 角度を更新
            q += delta_q;

            if (q(0) < 0) q(0)  = 0;
            else if (q(0) > 0.69) q(0) = 0.69;
            if (q(1) < 0) q(1)  = 0;
            else if (q(1) > 2.67) q(1) = 2.67;
            if (q(2) < -1.221) q(2)  = -1.221;
            else if (q(2) > 1.919) q(2) = 1.919;
        } while (abs(delta_q(0)) > threshold && abs(delta_q(1)) > threshold && abs(delta_q(2)) > threshold);
        q_ref = q;
        // 結果を出力
        cout << "arm_lift_joint: " << arm_lift_joint << endl;
        cout << "arm_flex_joint: " << arm_flex_joint << endl;
        cout << "wrist_flex_joint: " << wrist_flex_joint << endl << endl;

        trajectory_msgs::JointTrajectory traj;

        traj.joint_names.push_back("arm_lift_joint");
        traj.joint_names.push_back("arm_flex_joint");
        traj.joint_names.push_back("arm_roll_joint");
        traj.joint_names.push_back("wrist_flex_joint");
        traj.joint_names.push_back("wrist_roll_joint");

        traj.points.resize(1);

        traj.points[0].positions.resize(5);
        traj.points[0].positions[0] = arm_lift_joint;
        traj.points[0].positions[1] = -arm_flex_joint;
        traj.points[0].positions[2] = 0.0;
        traj.points[0].positions[3] = -wrist_flex_joint;
        traj.points[0].positions[4] = 0.0;
        traj.points[0].velocities.resize(5);
        for (size_t i = 0; i < 5; ++i) {
            traj.points[0].velocities[i] = 0.0;
        }
        traj.points[0].time_from_start = ros::Duration(0.01);

        if(arm_lift_joint != next_arm_lift_joint || arm_flex_joint != next_arm_flex_joint || arm_roll_joint != next_arm_roll_joint || wrist_flex_joint != next_wrist_flex_joint || wrist_roll_joint != next_wrist_roll_joint){
            printf("moving\n");
            pub.publish(traj);
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
        ros::spinOnce();
        next_arm_lift_joint = arm_lift_joint;
        next_arm_flex_joint = arm_flex_joint;
        next_arm_roll_joint = arm_roll_joint;
        next_wrist_flex_joint = wrist_flex_joint;
        next_wrist_roll_joint = wrist_roll_joint;
        return;
    }

    // 順運動学を計算する関数
    void Kinematics() {
        double trigonometric[2];
        trigonometric[0] = arm_lift_to_arm_flex + arm_flex_to_arm_roll * sin(arm_flex_joint) + wrist_to_hand * sin(arm_flex_joint + wrist_flex_joint);
        trigonometric[1] = arm_lift_joint -(head_to_arm_lift - arm_flex_to_arm_roll * cos(arm_flex_joint) - wrist_to_hand * cos(arm_flex_joint + wrist_flex_joint));
        // trigonometric[1] =  -(head_to_arm_lift - arm_flex_to_arm_roll * cos(arm_flex_joint) - wrist_to_hand * cos(arm_flex_joint + wrist_flex_joint));
        cout << "trigonometricX: " << trigonometric[0] << endl;
        cout << "trigonometricZ: " << trigonometric[1]<< endl << endl;
        cout << "targetX: " << targetX << endl;
        cout << "targetZ: " << targetZ<< endl << endl;
    }

    void oculusCallback(const oculus_telexistence::Part::ConstPtr& data) {
        tracked   = data -> Left.tracked.data;
        targetY   = data -> Left.position.x;
        targetZ   = data -> Left.position.y;
        targetX   = data -> Left.position.z;
        rotationY = data -> Left.rotation.x;
        rotationZ = data -> Left.rotation.y;
        rotationX = data -> Left.rotation.z;
    }

    void purike() {
        first_position_x = 0.2;
        first_position_z = -0.25;
        cout << "first_position_x: " << first_position_x  << endl;
        cout << "first_position_z: " << first_position_z << endl << endl;
    }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "hsr_arm_controll_node");
  Arm_controll ac;
  ac.run();
  return 0;
}

