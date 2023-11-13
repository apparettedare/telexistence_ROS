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
        Dimension = 2;
        arm_DoF = 3;
        // ジョイントの角度
        arm_lift_joint = 0.1;
        arm_flex_joint = 0.1;
        arm_roll_joint = 0.1;
        wrist_flex_joint = 0.1;
        wrist_roll_joint = 0.1;

        // 目標位置
        targetX = 0.0;
        targetY = 0.0;
        targetZ = 0.0;
        rotationX = 0.0;
        rotationY = 0.0;
        rotationZ = 0.0;
        head_position_z = 0.0;
        max_position_x = 0.0;
        max_position_z = 0.0;
        first_head_position = 0.0;

        // リンクの長さ
        head_to_arm_lift = 0.412;   
        arm_flex_to_arm_roll = 0.345;
        wrist_to_hand = 0.192;
        arm_lift_to_arm_flex = 0.141;
        wrist_to_hand_2 = 0.029;

        //人間の情報
        // 頭のz座標:1.2 1.3~0.8
        // 肩のｚ座標:0.95
        // 頭と肩の差:0.25


        // HSRの情報
        // HSRの頭の初期値:0.89
        // HSRのliftの初期値:0.3
        // HSRの手の初期値:0.34 0.07 0.65
        // HSRの手を前に伸ばしたときの初期値:0.67 0.10 0.306

        tracked = false;
        
        W_N_bar = MatrixXd::Identity(3, 3) * 0.01;  // bias of dumping factor 
        W_E = Matrix2d::Identity(2, 2);              // weigth of error
        // 収束条件
        threshold = 0.000001; 
        // threshold = 0.001;
        epsilon = 1e-10;
        
        }

        void run() {
            ros::NodeHandle nh;
            ros::Subscriber oculus_sub = nh.subscribe("/oculus/3d", 10, &Arm_controll::oculusCallback, this);
            ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
            time_t start_time = time(NULL);        

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
        while (difftime(time(NULL), start_time) < 4) { 
            setting();
        }
            while (ros::ok()) {
                VectorXd q(3);
                Vector2d target_pos;
                q << arm_lift_joint, arm_flex_joint, wrist_flex_joint;
                target_pos << targetX, targetZ;
                cout << "attyonpurike q(0): " << q(0) << endl;
                cout << "q(1): " << q(1) << endl;
                cout << "q(2): " << q(2) << endl << endl;
                if(tracked) solve(q, target_pos);
                Kinematics();
                ros::spinOnce();
                ros::Duration(0.1).sleep();
            }

            ros::spin();
    }
private:
    int Dimension;
    int arm_DoF;
    // ジョイントの角度
    double arm_lift_joint;
    double arm_flex_joint;
    double arm_roll_joint;
    double wrist_flex_joint;
    double wrist_roll_joint;
    // 目標位置
    double targetX;
    double targetY;
    double targetZ;
    double rotationX;
    double rotationY;
    double rotationZ;
    double head_position_z;
    double max_position_x;
    double max_position_z;
    double first_head_position;

    // リンクの長さ
    double head_to_arm_lift;   
    double arm_flex_to_arm_roll;
    double wrist_to_hand;

    // etc
    double arm_lift_to_arm_flex;
    double wrist_to_hand_2;

    bool tracked;
    
    MatrixXd W_N_bar; 
    Matrix2d W_E;
    // 収束条件
    double threshold; 
    double epsilon;


    // 残差を計算する関数
    void cal_e(Vector2d& e, VectorXd& q, Vector2d& target_pos) {
        // 目標位置と現在の位置の差分を計算
        // e(0) = target_pos(0) * 0.67 / max_position_x - (arm_flex_to_arm_roll * sin(q(1)) + wrist_to_hand * sin(q(1) + q(2)) + wrist_to_hand_2 * sin(q(2)));
        // e(1) = target_pos(1) * 0.1 / max_position_z - (0.339 + arm_lift_joint + arm_flex_to_arm_roll * cos(q(1)) + wrist_to_hand * cos(q(1) + q(2)) - wrist_to_hand_2 * cos(q(2)));
        // e(0) = target_pos(0) * 0.67 / max_position_x - (arm_flex_to_arm_roll * sin(q(1)) + wrist_to_hand * sin(q(1) + q(2)));
        // e(1) = target_pos(1) * 0.306 / max_position_z - (0.339 + arm_lift_joint + arm_flex_to_arm_roll * cos(q(1)) + wrist_to_hand * cos(q(1) + q(2)));
        // e(0) = target_pos(0) * 0.339 / max_position_x - (arm_flex_to_arm_roll * sin(-q(1)) + wrist_to_hand * sin(-q(1) - q(2)));
        // e(1) = target_pos(1) * 0.656 / max_position_z - (0.339 + q(0) + arm_flex_to_arm_roll * cos(-q(1)) + wrist_to_hand * cos(-q(1) - q(2)));
        // e(0) = (target_pos(0) - max_position_x )- ((arm_flex_to_arm_roll * sin(q(1)) + wrist_to_hand * sin(q(1) + q(2)) - wrist_to_hand));
        // e(1) = (target_pos(1) - max_position_z) - ((0.339 + q(0) + arm_flex_to_arm_roll * cos(q(1)) + wrist_to_hand * cos(q(1) + q(2))) - arm_flex_to_arm_roll);
        e(0) = target_pos(0) - (arm_flex_to_arm_roll * sin(q(1)) + wrist_to_hand * sin(q(1) + q(2)));
        e(1) = target_pos(1) - (0.339 + q(0) + arm_flex_to_arm_roll * cos(q(1)) + wrist_to_hand * cos(q(1) + q(2)));
        return;
    }

    // ヤコビ行列を計算する関数
    void cal_J(MatrixXd& J, VectorXd& q) {
        J(0,0) = 0;
        J(0,1) = arm_flex_to_arm_roll * cos(q(1)) + wrist_to_hand * cos(q(1) + q(2));
        // J(0,2) =                                    wrist_to_hand * cos(q(1) + q(2)) + wrist_to_hand_2 * cos(q(2));
        J(0,2) =                                    wrist_to_hand * cos(q(1) + q(2));
        J(1,0) = 1;
        J(1,1) = - arm_flex_to_arm_roll * sin(q(1)) - wrist_to_hand * sin(q(1) + q(2));
        // J(1,2) =                                    - wrist_to_hand * sin(q(1) + q(2) - wrist_to_hand_2 * sin(q(2)));
        J(1,2) =                                    - wrist_to_hand * sin(q(1) + q(2));
        return;
    }

    // 勾配を計算する関数
    void cal_g(VectorXd& g, const MatrixXd& J, const Vector2d& e) {
        g = J.transpose() * W_E * e;
        // cout << "g(0): " << g(0) << endl;
        // cout << "g(1): " << g(1) << endl;
        // cout << "g(2): " << g(2) << endl << endl;
        return;
    }
    
    // 減衰因子行列を計算する関数
    void cal_W_N(MatrixXd &W_N, const Vector2d &e) {
        W_N = MatrixXd::Identity(3, 3) * e.squaredNorm() + W_N_bar;
        // cout << "W_N(0,0): " << W_N(0,0) << endl;
        // cout << "W_N(1,1): " << W_N(1,1) << endl;
        // cout << "W_N(2,2): " << W_N(2,2) << endl << endl;
        return;
    }

    void cal_H(MatrixXd &H, const MatrixXd &J, const MatrixXd &W_N){
        H = J.transpose() * W_E * J + W_N;
        // cout << "H(0,0): " << H(0,0) << endl;
        // cout << "H(0,1): " << H(0,1) << endl;
        // cout << "H(0,2): " << H(0,2) << endl;
        // cout << "H(1,0): " << H(1,0) << endl;
        // cout << "H(1,1): " << H(1,1) << endl;
        // cout << "H(1,2): " << H(1,2) << endl;
        // cout << "H(2,0): " << H(2,0) << endl;
        // cout << "H(2,1): " << H(2,1) << endl;
        // cout << "H(2,2): " << H(2,2) << endl << endl;
        return;
    }

    // 逆運動学を行う関数
    void solve(VectorXd& q_ref, Vector2d& target_pos) {

        // ヤコビ行列
        MatrixXd J(Dimension, arm_DoF), W_N(arm_DoF, arm_DoF), H(arm_DoF, arm_DoF);
        Vector2d e;
        VectorXd g(arm_DoF), q(arm_DoF), q_new(arm_DoF), delta_q(arm_DoF);
        q = q_ref;

        ros::NodeHandle nh;
        ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
        // 逆運動学の反復計算(位置)
        do {
            cal_e(e, q, target_pos);
            cal_J(J, q);
            cal_g(g, J, e);
            cal_W_N(W_N, e);
            cal_H(H, J, W_N);

            // 角度の変化量を計算
            delta_q = H.inverse() * g;

            // 角度を更新
            q += delta_q;

            // 結果を出力
            cout << "arm_lift_joint: " << q(0) << endl;
            cout << "arm_flex_joint: " << q(1) << endl;
            cout << "wrist_flex_joint: " << q(2) << endl << endl;

            if (q(0) > 0.69) q(0) = 0.69;
            else if (q(0) < 0) q(0)  = 0; 
            if (q(1) < 0) q(1)  = 0;
            else if (q(1) > 2.617) q(1) = 2.617;
            if (q(2) < -1.221) q(2)  = -1.221;
            else if (q(2) > 1.919) q(2) = 1.919;
        } while (abs(delta_q(0)) > threshold && abs(delta_q(1)) > threshold && abs(delta_q(2)) > threshold);

        // 逆運動学の反復計算(向き)
        do {
            MatrixXd R(3, 3);
            VectorXd l(3), m(3);

            R(0,0) = cos(rotationY);
            R(0,1) = 0;
            R(0,2) = -sin(rotationY);
            R(1,0) = 0;
            R(1,1) = 1;
            R(1,2) = 0;
            R(2,0) = sin(rotationY);
            R(2,1) = 0;
            R(2,2) = cos(rotationY);

            l(0) = R(2,1) - R(1,2);
            l(1) = R(0,2) - R(2,0);
            l(2) = R(1,0) - R(0,1);

            m(0) = R(0,0) + 1;
            m(1) = R(1,1) + 1;
            m(2) = R(2,2) + 1;

            // // Rが対角行列
            // if(cos(rotationY) == 1) {
            //     // a = 0
            // }   
            // // Rが対角行列でない
            // if(sin(rotationY) != 0) {
            //     // a = atan2(l.squaredNorm(), R(0,0) + R(1,1) + R(2,2) - 1) / l.squaredNorm() * l;
            // }
            // if(cos(rotationY) == -1) {
            //     // a = M_PI / 2 * m;
            // }

            // Rが対角行列
            if(abs(cos(rotationY) - 1) < epsilon) {
                // a = 0
            }   
            // Rが対角行列でない
            if(sin(rotationY) < epsilon) {
                // a = atan2(l.squaredNorm(), R(0,0) + R(1,1) + R(2,2) - 1) / l.squaredNorm() * l;
            }
            if(abs(cos(rotationY) + 1) < epsilon) {
                // a = M_PI / 2 * m;
            }

        } while (abs(delta_q(0)) > threshold && abs(delta_q(1)) > threshold && abs(delta_q(2)) > threshold);

        q_ref = q;

        cout << "delta_q(0): " << delta_q(0) << endl;
        cout << "delta_q(1): " << delta_q(1) << endl;
        cout << "delta_q(2): " << delta_q(2) << endl << endl;

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
        traj.points[0].time_from_start = ros::Duration(0.01);

        if(q(0) != q_new(0) || q(1) != q_new(1) || q(2) != q_new(2)){
            printf("moving\n");
            pub.publish(traj);
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
        ros::spinOnce();
        q_new << arm_lift_joint, arm_flex_joint, wrist_flex_joint;
        return;
    }

    // 順運動学を計算する関数
    void Kinematics() {
        double trigonometric[2];
        // trigonometric[0] = arm_flex_to_arm_roll * sin(-q(1)) + wrist_to_hand * sin(-q(1) - q(2));
        // trigonometric[1] = 0.339 + q(0) + arm_flex_to_arm_roll * cos(-q(1)) + wrist_to_hand * cos(-q(1) - q(2));
        cout << "trigonometricX: " << trigonometric[0] << endl;
        cout << "trigonometricZ: " << trigonometric[1]<< endl << endl;
        cout << "targetX: " << targetX << endl;
        cout << "targetZ: " << targetZ<< endl << endl;
    }

    void oculusCallback(const oculus_telexistence::Part::ConstPtr& data) {
        tracked   = data -> Left.tracked.data;
        targetY   = round(data -> Left.position.x + 100) / 100;
        targetZ   = round(data -> Left.position.y * 100) / 100;
        targetX   = round(data -> Left.position.z * 100) / 100;
        rotationY = round(data -> Left.rotation.x * 100) / 100;
        rotationZ = round(data -> Left.rotation.y * 100) / 100;
        rotationX = round(data -> Left.rotation.z * 100) / 100;
        head_position_z = round(data -> Head.position.y * 100) / 100;
    }

    void setting() {
        first_head_position = head_position_z;
        // if (max_position_x < targetX){
        //     max_position_x = targetX;
        //     max_position_z = targetZ;
        // }
        max_position_x = targetX;
        max_position_z = targetZ;     
        cout << "max_position_x: " << max_position_x  << endl;
        cout << "max_position_z: " << max_position_z << endl << endl;
        cout << "first_head_position: " << first_head_position << endl << endl;
        ros::spinOnce();
    }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "hsr_arm_controll_node");
  Arm_controll ac;
  ac.run();
  return 0;
}
