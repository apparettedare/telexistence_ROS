#include "sobit_mini_library/sobit_mini_controller.hpp"
#include "oculus_telexistence/Part.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class JacobianInverseKinematics {
public:
    JacobianInverseKinematics() {
    // ジョイントの角度
    shoulder_roll = 0.01;
    shoulder_pan = -1.57;
    elbow_tilt = 0.01;
    wrist_tilt = 0.01;
    body_roll = 0.0;

    // 目標位置
    targetX = 0.0;
    // targetY = 0.0;
    targetZ = 0.0;
    // rotationX = 0.0;
    rotationY = 0.0;
    // rotationZ = 0.0;

    // リンクの長さ
    arm_shoulder = 11.124982;
    arm_elbow = 10.274915;
    arm_wrist = 16.403333;
    shoulder_width = 69.037267;

    tracked = false;
    set = false;
    is_left_hand = true;
    
    W_N_bar_p = MatrixXd::Identity(3, 3) * 0.01;  // bias of dumping factor 
    W_N_bar_r = MatrixXd::Identity(2, 2) * 0.01;  // bias of dumping factor 
    W_E = MatrixXd::Identity(3, 3);              // weigth of error

    // 収束条件
    threshold = 0.01; 
    // 最大反復回数
    maxIterations = 1000;
    // 反復回数
    iteration = 0;  
    
    }

    void run() {
        ros::NodeHandle nh;
        ros::Subscriber oculus_sub = nh.subscribe("/oculus/3d", 10, &JacobianInverseKinematics::oculusCallback, this);


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
            q << shoulder_roll, elbow_tilt, wrist_tilt;
            target_pos << targetX, targetZ;
            target_rot = rotationY;
            if(target_pos(0) > (arm_shoulder + arm_elbow + arm_wrist)) target_pos(0) = arm_shoulder + arm_elbow + arm_wrist;
            else if(target_pos(0) < 0) target_pos(0) = 0;
            if(target_pos(1) > (arm_shoulder + arm_elbow + arm_wrist)) target_pos(1) = arm_shoulder + arm_elbow + arm_wrist;
            else if(target_pos(1) < 0) target_pos(1) = 0;
            if(rotationY < -1) rotationY = -1;
            else if(rotationY > 1) rotationY = 1;
            if(tracked) solve(q, target_pos, target_rot);
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        ros::spin();
    }
private:
    // ジョイントの角度
    double shoulder_roll;
    double shoulder_pan;
    double elbow_tilt;
    double wrist_tilt;
    double body_roll;
    // 目標位置
    double targetX;
    // double targetY;
    double targetZ;
    // double rotationX;
    double rotationY;
    // double rotationZ;
    double MaxtargetX;
    double MaxtargetY;
    double MintargetZ;

    // リンクの長さ
    double head_to_shoulder;
    double arm_shoulder;
    double arm_elbow;
    double arm_wrist;
    double shoulder_width;

    bool tracked;
    bool set;
    bool is_left_hand;
    
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
        e(0) = target_pos(0) - (arm_shoulder * sin(p(0)) + arm_elbow * sin(q(0) + q(1)) + arm_wrist * sin(q(0) + q(1) + q(2)));
        e(1) = 0;
        e(2) = target_pos(1) - (arm_shoulder * cos(q(0)) + arm_elbow * cos(q(0) + q(1)) + arm_wrist * cos(q(0) + q(1) + q(2)));
        return;
    }

    // 残差を計算する関数
    void cal_e_rot(VectorXd& e, VectorXd& q, float target_rot) {
        // 目標位置と現在の姿勢の差分を計算
        MatrixXd dR(3, 3), R(3, 3), rrq(3, 3);
        VectorXd l(3), m(3), a(3);
        float currentY = q(1) + q(2) + q(3);
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
        J << arm_shoulder * cos(q(0)) + arm_elbow * cos(q(0) + q(1)) + arm_wrist * cos(q(0) + q(1) + q(2)), arm_elbow * cos(q(0) + q(1)) + arm_wrist * cos(q(0) + q(1) + q(2)), arm_wrist * cos(q(0) + q(1) + q(2)),
             0,                                                                   0,                                 0,
             arm_shoulder * sin(q(0)) + arm_elbow * sin(q(0) + q(1)) + arm_wrist * sin(q(0) + q(1) + q(2)), arm_elbow * sin(q(0) + q(1)) + arm_wrist * sin(q(0) + q(1) + q(2)), arm_wrist * sin(q(0) + q(1) + q(2));
        return;
    }

    void cal_J_rot(MatrixXd& J, VectorXd& q) {
        J << 0, 0, 0,
             1, 1, 1,
             0, 0, 0;
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
            cal_J_rot(J_r);
            cal_g(g_r, J_r, e_r);
            cal_W_N_rot(W_N_r, e_r);
            cal_H(H_r, J_r, W_N_r);

            // 角度の変化量を計算
            delta_q_r = H_r.inverse() * g_r;

            // 角度を更新            
            q += delta_q_p;


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

        q_ref = q;
        // 結果を出力
        cout << "shoulder_roll: " << shoulder_roll << endl;
        cout << "elbow_tilt: " << elbow_tilt << endl;
        cout << "wrist_tilt: " << wrist_tilt << endl << endl;
        sobit_mini::SobitMiniController mini_arm_ctr;

        if(shoulder_roll != next_shoulder_roll || elbow_tilt != next_elbow_tilt || wrist_tilt != next_wrist_tilt){
            if(abs(shoulder_roll - next_shoulder_roll) < 0.4 && abs(elbow_tilt - next_elbow_tilt) < 0.4 && abs(wrist_tilt - next_wrist_tilt) < 0.4){
                printf("moving\n");
                ros::spinOnce();
                mini_arm_ctr.moveLeftArm( shoulder_roll, -1.35, elbow_tilt, wrist_tilt, 0.0, 0.5, false);
                ros::Duration(0.01).sleep();
            }
        }
        q_new  = q;
        iteration = 0;
        ros::spinOnce();

        return;
    }




    // ヤコビ行列を計算する関数
    void calculateJacobian_2d(double shoulder_roll, double elbow_tilt, double wrist_tilt, Eigen::MatrixXd& jacobian2d) {
        // ヤコビ行列の要素を計算
        jacobian2d(0,0) = arm_shoulder * cos(shoulder_roll) + arm_elbow * cos(shoulder_roll + elbow_tilt) + arm_wrist * cos(shoulder_roll + elbow_tilt + wrist_tilt);
        jacobian2d(0,1) = arm_elbow * cos(shoulder_roll + elbow_tilt) + arm_wrist * cos(shoulder_roll + elbow_tilt + wrist_tilt);
        jacobian2d(0,2) = arm_wrist * cos(shoulder_roll + elbow_tilt + wrist_tilt);
        jacobian2d(1,0) = arm_shoulder * sin(shoulder_roll) + arm_elbow * sin(shoulder_roll + elbow_tilt) + arm_wrist * sin(shoulder_roll + elbow_tilt + wrist_tilt);
        jacobian2d(1,1) = arm_elbow * sin(shoulder_roll + elbow_tilt) + arm_wrist * sin(shoulder_roll + elbow_tilt + wrist_tilt);
        jacobian2d(1,2) = arm_wrist * sin(shoulder_roll + elbow_tilt + wrist_tilt);

    }
    void calculateJacobian_3d(double shoulder_pan, double shoulder_roll, double elbow_tilt, double wrist_tilt, Eigen::MatrixXd& jacobian3d) {
        // ヤコビ行列の要素を計算
        jacobian3d(0,0) = -arm_shoulder * cos(-shoulder_pan) * sin(shoulder_roll) - arm_elbow * cos(elbow_tilt) * cos(-shoulder_pan) * sin(shoulder_roll) - arm_wrist * cos(-shoulder_pan) * cos(-elbow_tilt -wrist_tilt) * sin(shoulder_roll);
        jacobian3d(0,1) = -arm_shoulder * cos(shoulder_roll) * sin(shoulder_pan) - arm_elbow * cos(shoulder_roll) * cos(elbow_tilt) * sin(shoulder_pan) - arm_wrist * cos(shoulder_roll) * cos(-elbow_tilt -wrist_tilt) * sin(shoulder_pan) + arm_elbow * sin(shoulder_roll) * sin(elbow_tilt) - arm_wrist * sin(shoulder_roll) * sin(-elbow_tilt -wrist_tilt);
        jacobian3d(0,2) = -arm_elbow * sin(shoulder_pan) * sin(shoulder_roll) * sin(-elbow_tilt) - arm_wrist * sin(shoulder_pan) * sin(shoulder_roll) * sin(-elbow_tilt -wrist_tilt) - arm_elbow * cos(shoulder_roll) * cos(-elbow_tilt) - arm_wrist * cos(shoulder_roll) * cos(-elbow_tilt -wrist_tilt);
        jacobian3d(0,3) = -arm_wrist * sin(shoulder_pan) * sin(shoulder_roll) * sin(-elbow_tilt -wrist_tilt) - arm_wrist * cos(shoulder_roll) * cos(-elbow_tilt -wrist_tilt);

        jacobian3d(1,0) = -arm_shoulder * sin(shoulder_pan) - arm_elbow * cos(elbow_tilt) * sin(shoulder_pan) - arm_wrist * cos(-elbow_tilt -wrist_tilt) * sin(shoulder_pan);
        jacobian3d(1,1) = 0;
        jacobian3d(1,2) = -arm_elbow * cos(shoulder_pan) * sin(elbow_tilt) + arm_wrist * cos(shoulder_pan) * sin(-elbow_tilt -wrist_tilt);
        jacobian3d(1,3) = arm_wrist * cos(shoulder_pan) * sin(-elbow_tilt -wrist_tilt);

        jacobian3d(2,0) = -arm_shoulder * cos(shoulder_pan) * cos(shoulder_roll) - arm_elbow * cos(shoulder_pan) * cos(shoulder_roll) * cos(elbow_tilt) + arm_wrist * cos(shoulder_pan) * cos(shoulder_roll) * cos(-elbow_tilt -wrist_tilt);
        jacobian3d(2,1) = arm_shoulder * sin(shoulder_pan) * sin(shoulder_roll) + arm_elbow * cos(elbow_tilt) * sin(shoulder_pan) * sin(shoulder_roll) - arm_wrist * cos(-elbow_tilt -wrist_tilt) * sin(shoulder_pan) * sin(shoulder_roll) + arm_elbow * cos(shoulder_roll) * sin(elbow_tilt) + arm_wrist * cos(shoulder_roll) * sin(-elbow_tilt -wrist_tilt);
        jacobian3d(2,2) = -arm_elbow * cos(shoulder_roll) * sin(shoulder_pan) * sin(-elbow_tilt) + arm_wrist * cos(shoulder_roll) * sin(shoulder_pan) * sin(-elbow_tilt -wrist_tilt) + arm_elbow * cos(-elbow_tilt) * sin(shoulder_roll) - arm_wrist * cos(-elbow_tilt -wrist_tilt) * sin(shoulder_roll);
        jacobian3d(2,3) = arm_wrist * cos(shoulder_roll) * sin(shoulder_pan) * sin(-elbow_tilt -wrist_tilt) - arm_wrist * cos(-elbow_tilt -wrist_tilt) * sin(shoulder_roll);
        
        // jacobian3d(0,0) = cos(-shoulder_pan) * sin(shoulder_roll) * (-arm_shoulder -(arm_elbow * cos(elbow_tilt) + arm_wrist * cos(-elbow_tilt -wrist_tilt)));
        // jacobian3d(0,1) = sin(shoulder_pan) * cos(shoulder_roll) * -(arm_shoulder + arm_elbow * cos(elbow_tilt) + arm_wrist * cos(-elbow_tilt -wrist_tilt)) + arm_elbow * sin(shoulder_roll) * sin(elbow_tilt) -arm_wrist * sin(shoulder_roll) * sin(-elbow_tilt -wrist_tilt);
        // jacobian3d(0,2) = sin(shoulder_pan) * sin(shoulder_roll) * -(arm_elbow * sin(-elbow_tilt) + arm_wrist * sin(-elbow_tilt -wrist_tilt)) -arm_elbow * cos(shoulder_roll) * cos(-elbow_tilt) -arm_wrist * cos(shoulder_roll) * cos(-elbow_tilt -wrist_tilt);
        // jacobian3d(0,3) = -arm_wrist * (sin(shoulder_pan) * sin(shoulder_roll) * sin(-elbow_tilt -wrist_tilt) + cos(shoulder_roll) * cos(-elbow_tilt -wrist_tilt));

        // jacobian3d(1,0) = sin(shoulder_pan) * -(arm_elbow * cos(elbow_tilt) + arm_wrist * cos(-elbow_tilt -wrist_tilt) + arm_shoulder);
        // jacobian3d(1,1) = 0.0;
        // jacobian3d(1,2) = cos(shoulder_pan) * (arm_elbow * sin(-elbow_tilt) + arm_wrist * sin(-elbow_tilt -wrist_tilt));
        // jacobian3d(1,3) = arm_wrist * cos(shoulder_pan) * sin(-elbow_tilt -wrist_tilt);

        // jacobian3d(2,0) = cos(shoulder_pan) * cos(shoulder_roll) * -(arm_shoulder + arm_elbow * cos(-elbow_tilt) + arm_wrist * cos(-elbow_tilt -wrist_tilt));
        // jacobian3d(2,1) = sin(shoulder_pan) * sin(shoulder_roll) * (arm_shoulder + arm_elbow * cos(-elbow_tilt) + arm_wrist * cos(-elbow_tilt -wrist_tilt)) - arm_elbow * cos(shoulder_roll) * sin(-elbow_tilt) - arm_wrist * cos(shoulder_roll) + sin(-elbow_tilt -wrist_tilt);
        // jacobian3d(2,2) = sin(shoulder_pan) * cos(shoulder_roll) * -(arm_elbow * sin(-elbow_tilt) + arm_wrist * sin(-elbow_tilt -wrist_tilt)) + arm_elbow * cos(-elbow_tilt) * sin(shoulder_roll) + arm_wrist * cos(-elbow_tilt -wrist_tilt) * sin(shoulder_roll);
        // jacobian3d(2,3) = arm_wrist * (-cos(shoulder_roll) * sin(shoulder_pan) * sin(-elbow_tilt -wrist_tilt) + cos(-elbow_tilt -wrist_tilt) * sin(shoulder_roll));


    }

    // 順運動学を計算する関数
    void Kinematics() {
        double trigonometric[3];
        trigonometric[0] = arm_shoulder * sin(shoulder_roll) + arm_elbow * sin(shoulder_roll + elbow_tilt) + arm_wrist * sin(shoulder_roll + elbow_tilt + wrist_tilt);
        trigonometric[1] = 174;
        trigonometric[2] = -(141.953 + arm_shoulder * cos(shoulder_roll) + arm_elbow * cos(shoulder_roll + elbow_tilt) + arm_wrist * cos(shoulder_roll + elbow_tilt + wrist_tilt));
        double trigonometric_3d[3];
        double point0[3] = {0, 174, 141.953};
        double point1[3] = {point0[0] + arm_shoulder * sin(-shoulder_pan) * sin(shoulder_roll), point0[1] + arm_shoulder * cos(shoulder_pan), point0[2] + arm_shoulder * sin(shoulder_pan) * cos(shoulder_roll)};
        double t[2] = {arm_elbow * cos(-elbow_tilt) + arm_wrist * cos(-elbow_tilt - wrist_tilt), arm_elbow * sin(-elbow_tilt) + arm_wrist * sin(-elbow_tilt  - wrist_tilt)};
        double point01_vec[3] = {(point1[0] - point0[0]) / arm_shoulder, (point1[1] - point0[1]) / arm_shoulder , (point1[2] - point0[2]) / arm_shoulder};
        trigonometric_3d[0] = point1[0] + point01_vec[0] * t[0] + cos(shoulder_roll) * t[1];
        trigonometric_3d[1] = point1[1] + point01_vec[1] * t[0];
        trigonometric_3d[2] = -(point1[2] + point01_vec[2] * t[0] + sin(shoulder_roll) * t[1]);
        // trigonometric_3d[0] = arm_shoulder * sin(-shoulder_pan) * sin(shoulder_roll) + (sin(-shoulder_pan) * sin(shoulder_roll)) * (arm_elbow * cos(-elbow_tilt) + arm_wrist * cos(-elbow_tilt - wrist_tilt)) + cos(shoulder_roll) * (arm_elbow * sin(-elbow_tilt) + arm_wrist * sin(-elbow_tilt  - wrist_tilt));
        // trigonometric_3d[1] = 174 + arm_shoulder * cos(shoulder_pan) + cos(shoulder_pan) * (arm_elbow * cos(-elbow_tilt) + arm_wrist * cos(-elbow_tilt - wrist_tilt));
        // trigonometric_3d[2] = -(141.953 + arm_shoulder * sin(shoulder_pan) * cos(shoulder_roll) + sin(shoulder_pan) * cos(shoulder_roll) * (arm_elbow * cos(-elbow_tilt) + arm_wrist * cos(-elbow_tilt - wrist_tilt)) + sin(shoulder_roll) * (arm_elbow * sin(-elbow_tilt) + arm_wrist * sin(-elbow_tilt  - wrist_tilt)));

        // arm_shoulder = A, arm_elbow = B, arm_wrist = C, shoulder_pan = a, shoulder_roll = b, elbow_tilt = c, wrist_tilt = g,
        // trigonometric_3d[0] = A * sin(-a) * sin(b) + (sin(-a) * sin(b)) * (B * cos(-c) + C * cos(-c - g)) + cos(b) * (B * sin(-c) + C * sin(-c  - g));
        // trigonometric_3d[1] = 174 + A * cos(a) + cos(a) * (B * cos(-c) + C * cos(-c - g));
        // trigonometric_3d[2] = -(141.953 + A * sin(a) * cos(b) + sin(a) * cos(b) * (B * cos(-c) + C * cos(-c - g)) + sin(b) * (B * sin(-c) + C * sin(-c  - g)));
        cout << "trigonometricX: " << trigonometric_3d[0] << endl;
        cout << "trigonometricY: " << trigonometric_3d[1] << endl;
        cout << "trigonometricZ: " << trigonometric_3d[2]<< endl << endl;
    }

    void oculusCallback(const oculus_telexistence::Part::ConstPtr& data) {
        if (is_left_hand) {
            tracked           = data -> Left.tracked.data;
            targetX           = round(data -> Left.position.z * 100) / 100;
            targetZ           = round(data -> Left.position.y * 100) / 100;
            if(set) rotationY = -round(data -> Left.rotation.x * 100) / 100 * 2;
            else rotationY    = round(data -> Left.rotation.x * 100) / 100 * 2;
        }
        else {
            tracked           = data -> Right.tracked.data;
            targetX           = round(data -> Right.position.z * 100) / 100;
            targetZ           = round(data -> Right.position.y * 100) / 100;
            if(set) rotationY = -round(data -> Right.rotation.x * 100) / 100 * 2;
            else rotationY    = round(data -> Right.rotation.x * 100) / 100 * 2;
        }
    }
    void setting() {
        if(rotationY > 0){
            set = true;
        }
    }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "jacobian_inverse_kinematics_node");
  JacobianInverseKinematics jik;
  jik.run();
  return 0;
}

