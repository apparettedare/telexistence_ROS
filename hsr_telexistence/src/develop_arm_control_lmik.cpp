#include "oculus_telexistence/Part.h"
#include <ros/ros.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class Arm_controll {
public:
    Arm_controll() {
        dimension = 2;
        arm_DoF = 3;

        // ジョイントの角度
        arm_lift_joint = 0.1;
        arm_flex_joint = 0.1;
        wrist_flex_joint = 0.1;

        // 目標位置
        targetX = 0.0;
        targetY = 0.0;
        targetZ = 0.0;
        rotationY = 0.0;
        head_position_z = 0.0;
        first_head_position = 0.0;
        first_hand_position = 0.0;

        // リンクの長さ
        arm_flex_to_arm_roll = 0.345;
        wrist_to_hand = 0.192;

        tracked = false;
        set = false;
        hand_is_left = false;
        
        W_N_bar_p = MatrixXd::Identity(3, 3) * 0.01;  // bias of dumping factor 
        W_N_bar_r = MatrixXd::Identity(2, 2) * 0.01;  // bias of dumping factor 
        W_E = MatrixXd::Identity(3, 3);              // weigth of error

        // 収束条件
        threshold = 0.0001; 
        // 最大反復回数
        maxIterations = 1000;
        // 反復回数
        iteration = 0;        
    }

    void run() {
        ros::NodeHandle nh;
        ros::Subscriber oculus_sub = nh.subscribe("/oculus/3d", 10, &Arm_controll::oculusCallback, this);
        ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
        nh.getParam("hand", hand_is_left);

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

        while (ros::ok()) {
            if (tracked) {
                setting();
                break;
            }
            ros::spinOnce();
        }

        while (ros::ok()) {
            VectorXd q(3);
            Vector2d target_pos;
            float target_rot;
            q << arm_lift_joint, arm_flex_joint, wrist_flex_joint;
            target_pos << sqrt(pow(targetX, 2) + pow(targetY, 2)), targetZ;
            target_rot = rotationY;
            if(target_pos(0) > (arm_flex_to_arm_roll + wrist_to_hand)) target_pos(0) = arm_flex_to_arm_roll + wrist_to_hand;
            else if(target_pos(0) < 0) target_pos(0) = 0;
            if(target_pos(1) > (0.339 + 0.69 + arm_flex_to_arm_roll + wrist_to_hand)) target_pos(1) = 0.339 + 0.69 + arm_flex_to_arm_roll + wrist_to_hand;
            else if(target_pos(1) < 0) target_pos(1) = 0;
            if(rotationY < -1) rotationY = -1;
            else if(rotationY > 1) rotationY = 1;
            if(tracked) solve(q, target_pos, target_rot);
            ros::spinOnce();
        }

        ros::spin();
    }
private:
    int dimension;
    int arm_DoF;

    // ジョイントの角度
    float arm_lift_joint;
    float arm_flex_joint;
    float wrist_flex_joint;

    // 目標位置
    float targetX;
    float targetY;
    float targetZ;
    float rotationY;
    float head_position_z;
    float first_head_position;
    float first_hand_position;

    // リンクの長さ
    float arm_flex_to_arm_roll;
    float wrist_to_hand;

    bool tracked;
    bool set;
    bool hand_is_left;
    
    MatrixXd W_E;
    MatrixXd W_N_bar_p;
    MatrixXd W_N_bar_r; 

    // 収束条件
    float threshold; 
    // 最大反復回数
    int maxIterations;
    // 反復回数
    int iteration;

    // 残差を計算する関数
    void cal_e_pos(VectorXd& e, VectorXd& q, Vector2d& target_pos) {
        // 目標位置と現在の位置の差分を計算
        e(0) = target_pos(0) - (arm_flex_to_arm_roll * sin(q(1)) + wrist_to_hand * sin(q(1) + q(2)));
        e(1) = 0;
        e(2) = target_pos(1) - (0.339 + q(0) + arm_flex_to_arm_roll * cos(q(1)) + wrist_to_hand * cos(q(1) + q(2)));
        return;
    }

    // 残差を計算する関数
    void cal_e_rot(VectorXd& e, VectorXd& q, float target_rot) {
        // 目標位置と現在の姿勢の差分を計算
        MatrixXd dR(3, 3), R(3, 3), rrq(3, 3);
        VectorXd l(3), m(3), a(3);
        float currentY = q(1)+ q(2);
        float theta = M_PI - acos(rotationY);
        dR << cos(theta), 0, -sin(theta),
                         0, 1, 0,
              sin(theta), 0, cos(theta);

        R << cos(currentY), 0, -sin(currentY),
                           0, 1, 0,
              sin(currentY), 0, cos(currentY);
        rrq = dR * R.transpose();

        l(0) = rrq(1,2) - rrq(2,1);
        l(1) = rrq(2,0) - rrq(0,2);
        l(2) = rrq(0,1) - rrq(1,0);

        m(0) = rrq(0,0) + 1;
        m(1) = rrq(1,1) + 1;
        m(2) = rrq(2,2) + 1;

        // Rが対角行列
        if(theta == 0) a.setZero();
        else if(theta == M_PI) a = M_PI / 2 * m;
  
        // Rが対角行列でない
        else a = atan2(l.squaredNorm(), rrq(0,0) + rrq(1,1) + rrq(2,2) - 1) / l.squaredNorm() * l;
        e = a;
        return;
    }

    // ヤコビ行列を計算する関数
    void cal_J_pos(MatrixXd& J, VectorXd& q) {
        J << 0, arm_flex_to_arm_roll * cos(q(1)) + wrist_to_hand * cos(q(1) + q(2)), wrist_to_hand * cos(q(1) + q(2)),
             0,                                                                   0,                                 0,
             1, -arm_flex_to_arm_roll * sin(q(1)) - wrist_to_hand * sin(q(1) + q(2)), -wrist_to_hand * sin(q(1) + q(2));
        return;
    }

    void cal_J_rot(MatrixXd& J, VectorXd& q) {
        J << 0, 0,
             1, 1,
             0, 0;
        return;
    }

    // 勾配を計算する関数
    void cal_g(VectorXd& g, const MatrixXd& J, const VectorXd& e) {
        g = J.transpose() * W_E * e;
        return;
    }   
    
    // 減衰因子行列を計算する関数
    void cal_W_N_pos(MatrixXd &W_N_p, const VectorXd &e_p) {
        W_N_p = MatrixXd::Identity(3, 3) * e_p.squaredNorm() + W_N_bar_p;
        return;
    }

    void cal_W_N_rot(MatrixXd &W_N_r, const VectorXd &e_r) {
        W_N_r = MatrixXd::Identity(2, 2) * e_r.squaredNorm() + W_N_bar_r;
        return;
    }

    void cal_H(MatrixXd &H, const MatrixXd &J, const MatrixXd &W_N){
        H = J.transpose() * W_E * J + W_N;
        return;
    }

    // 逆運動学を行う関数
    void solve(VectorXd& q_ref, Vector2d& target_pos, float target_rot) {
        MatrixXd J_p(3, 3), J_r(3, 2), W_N_p(3, 3), W_N_r(2, 2), H_p(3, 3), H_r(2, 2), R(3, 3), dR(3, 3);
        VectorXd e_p(3), e_r(3), g_p(3), g_r(2), q(3), q_new(3), delta_q_p(3), delta_q_r(2);
        q = q_ref;
        
        ros::NodeHandle nh;
        ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);

        // 逆運動学の反復計算
        do {
            cal_e_rot(e_r, q, target_rot);
            cal_J_rot(J_r, q);
            cal_g(g_r, J_r, e_r);
            cal_W_N_rot(W_N_r, e_r);
            cal_H(H_r, J_r, W_N_r);

            // 角度の変化量を計算
            delta_q_r = H_r.inverse() * g_r;

            // 角度を更新
            q(1) += delta_q_r(0);
            q(2) += delta_q_r(1);


            cal_e_pos(e_p, q, target_pos);
            cal_J_pos(J_p, q);
            cal_g(g_p, J_p, e_p);
            cal_W_N_pos(W_N_p, e_p);
            cal_H(H_p, J_p, W_N_p);

            // 角度の変化量を計算
            delta_q_p = H_p.inverse() * g_p;

            // 角度を更新
            q += delta_q_p;

            if (q(0) > 0.69) q(0) = 0.69;
            else if (q(0) < 0) q(0)  = 0; 
            if (q(1) < 0) q(1)  = 0;
            else if (q(1) > 2.617) q(1) = 2.617;
            if (q(2) < -1.221) q(2)  = -1.221;
            else if (q(2) > 1.919) q(2) = 1.919;
            // 反復回数をインクリメント
            iteration++;
        } while (abs(delta_q_p(0)) > threshold && abs(delta_q_p(1)) > threshold && abs(delta_q_p(2)) > threshold  && iteration < maxIterations);

        q(0) = (head_position_z - first_head_position);
        if (q(0) < 0) q(0) = 0;
        q_ref = q;

        trajectory_msgs::JointTrajectory traj;
        traj.joint_names.push_back("arm_lift_joint");
        traj.joint_names.push_back("arm_flex_joint");
        traj.joint_names.push_back("arm_roll_joint");
        traj.joint_names.push_back("wrist_flex_joint");
        traj.joint_names.push_back("wrist_roll_joint");

        traj.points.resize(1);

        traj.points[0].positions.resize(5);
        traj.points[0].positions[0] = q(0);
        traj.points[0].positions[1] = -q(1);
        traj.points[0].positions[2] = 0.0;
        traj.points[0].positions[3] = -q(2);
        traj.points[0].positions[4] = 0.0;
        traj.points[0].velocities.resize(5);
        for (size_t i = 0; i < 5; ++i) {
            traj.points[0].velocities[i] = 0.0;
        }
        
        traj.points[0].time_from_start = ros::Duration(0.4);

        if(q(0) != q_new(0) || q(1) != q_new(1) || q(2) != q_new(2)){
            printf("The arm is moving\n");
            pub.publish(traj);
            ros::spinOnce();
            ros::Duration(0.01).sleep();
            q_new  = q;
        }

        ros::spinOnce();
        iteration = 0;
        return;
    }

    void oculusCallback(const oculus_telexistence::Part::ConstPtr& data) {
        if (hand_is_left) {
            tracked           = data -> Left.tracked.data;
            targetX           = round((data -> Left.position.z - data -> Head.position.z) * 100) / 100;
            targetY           = round((data -> Left.position.x - first_hand_position - data -> Head.position.x) * 100) / 100;
            targetZ           = round(data -> Left.position.y * 100) / 100;
            if(set) rotationY = -round(data -> Left.rotation.x * 100) / 100 * 2;
            else rotationY    = round(data -> Left.rotation.x * 100) / 100 * 2;
        }
        else {
            tracked           = data -> Right.tracked.data;
            targetX           = round((data -> Right.position.z - data -> Head.position.z) * 100) / 100;
            targetY           = round((data -> Right.position.x + first_hand_position - data -> Head.position.x) * 100) / 100;
            targetZ           = round(data -> Right.position.y * 100) / 100;
            if(set) rotationY = -round(data -> Right.rotation.x * 100) / 100 * 2;
            else rotationY    = round(data -> Right.rotation.x * 100) / 100 * 2;
        }
        head_position_z = data -> Head.position.y;
    }

    void setting() {
        if(rotationY > 0){
            set = true;
        }
        first_head_position = head_position_z;
        first_hand_position = targetY;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "develop_hsr_arm_control_lmik_node");
    Arm_controll ac;
    ac.run();
    return 0;
}
