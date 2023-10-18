#include "sobit_mini_library/sobit_mini_controller.hpp"
#include "oculus_telexistence/Part.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <iostream>

using namespace std;

// ジョイントの角度
double shoulder_roll, shoulder_pan, elbow_tilt, wrist_tilt, body_roll;

// 目標位置
double targetX = 0.0;
double targetY = 0.0;
double targetZ = 0.0;
double rotationX = 0.0;
double rotationY = 0.0;
double rotationZ = 0.0;

// リンクの長さ
const double arm_elbow = 102.57457;
const double arm_wrist = 156.55295;
const double arm_shoulder = 108.75985;
const double shoulder_width = 195.0;

// ヤコビ行列を計算する関数
void calculateJacobian_2d(double shoulder_roll, double elbow_tilt, double wrist_tilt, double jacobian[2][3]) {
    // ヤコビ行列の要素を計算
    jacobian[0][0] = arm_shoulder * cos(shoulder_roll) + arm_elbow * cos(shoulder_roll + elbow_tilt) + arm_wrist * cos(shoulder_roll + elbow_tilt + wrist_tilt);
    jacobian[0][1] = arm_elbow * cos(shoulder_roll + elbow_tilt) + arm_wrist * cos(shoulder_roll + elbow_tilt + wrist_tilt);
    jacobian[0][2] = arm_wrist * cos(shoulder_roll + elbow_tilt + wrist_tilt);
    
    jacobian[1][0] = -arm_shoulder * sin(shoulder_roll) - arm_elbow * sin(shoulder_roll + elbow_tilt) - arm_wrist * sin(shoulder_roll + elbow_tilt + wrist_tilt);
    jacobian[1][1] = -arm_elbow * sin(shoulder_roll + elbow_tilt) - arm_wrist * sin(shoulder_roll + elbow_tilt + wrist_tilt);
    jacobian[1][2] = -arm_wrist * sin(shoulder_roll + elbow_tilt + wrist_tilt);
}

void InverseKinematics_2d() {
    // 逆運動学の解を格納する変数
    double delta_shoulder_roll, delta_elbow_tilt, delta_wrist_tilt;
    
    // 収束条件
    double epsilon = 0.1;
    
    // 最大反復回数
    int maxIterations = 1000;
    
    // 反復回数
    int iteration = 0;
    
    // ヤコビ行列
    double jacobian[2][3];
    
    // 逆運動学の反復計算
    do {
        cout << "ヤコビってます" << endl;
        // ヤコビ行列を計算
        calculateJacobian_2d(shoulder_roll, elbow_tilt, wrist_tilt, jacobian);
        
        // ヤコビ行列の擬似逆行列を計算
        double jacobianInverse[3][2];
        double determinant = jacobian[0][0] * jacobian[1][1] - jacobian[0][1] * jacobian[1][0];
        jacobianInverse[0][0] = jacobian[1][1] / determinant;
        jacobianInverse[0][1] = -jacobian[0][1] / determinant;
        jacobianInverse[1][0] = -jacobian[1][0] / determinant;
        jacobianInverse[1][1] = jacobian[0][0] / determinant;
        jacobianInverse[2][0] = (jacobian[1][0] * jacobian[2][1] - jacobian[1][1] * jacobian[2][0]) / determinant;
        jacobianInverse[2][1] = -(jacobian[0][0] * jacobian[2][1] - jacobian[0][1] * jacobian[2][0]) / determinant;
        
        // 目標位置と現在の位置の差分を計算
        double deltaX = targetX - (arm_shoulder * sin(shoulder_roll) + arm_elbow * sin(shoulder_roll + elbow_tilt) + arm_wrist * cos(shoulder_roll + elbow_tilt + wrist_tilt));
        double deltaZ = targetZ - (arm_shoulder * sin(shoulder_roll) + arm_elbow * sin(shoulder_roll + elbow_tilt) + arm_wrist * sin(shoulder_roll + elbow_tilt + wrist_tilt));
        
        // 角度の変化量を計算
        delta_shoulder_roll = jacobianInverse[0][0] * deltaX + jacobianInverse[0][1] * deltaZ;
        delta_elbow_tilt = jacobianInverse[1][0] * deltaX + jacobianInverse[1][1] * deltaZ;
        delta_wrist_tilt = jacobianInverse[2][0] * deltaX + jacobianInverse[2][1] * deltaZ;
        
        // 角度を更新
        shoulder_roll += delta_shoulder_roll;
        elbow_tilt += delta_elbow_tilt;
        elbow_tilt += delta_wrist_tilt;
        
        // 反復回数をインクリメント
        iteration++;
    } while ((fabs(delta_shoulder_roll) > epsilon || fabs(delta_elbow_tilt) > epsilon || fabs(delta_wrist_tilt) > epsilon) && iteration < maxIterations);
    
    // 結果を出力
    cout << "shoulder_roll: " << shoulder_roll << endl;
    cout << "elbow_tilt: " << elbow_tilt << endl;
    cout << "wrist_tilt: " << wrist_tilt << endl;
    
    sobit_mini::SobitMiniController mini_arm_ctr;
    // smini_arm_ctr.moveRightArm( shoulder_roll, -1.4, elbow_tilt, wrist_tilt, 0.0);

}


void Kinematics() {
    double trigonometric[3];
    trigonometric[0] = arm_shoulder * sin(shoulder_roll) + arm_elbow * sin(shoulder_roll + elbow_tilt) + arm_wrist * sin(shoulder_roll + elbow_tilt + wrist_tilt);
    trigonometric[1] = 174;
    trigonometric[2] = 141.953 + arm_shoulder * cos(shoulder_roll) + arm_elbow * cos(shoulder_roll + elbow_tilt) + arm_wrist * cos(shoulder_roll + elbow_tilt + wrist_tilt);
    // double trigonometric_3d[3];
    // double point0[3] = {0, 174, -141.953};
    // double point1[3] = {point0[0] + arm_shoulder * sin(-shoulder_pan) * sin(shoulder_roll), point0[1] + arm_shoulder * cos(shoulder_pan), point0[2] + arm_shoulder * sin(shoulder_pan) * cos(shoulder_roll)};
    // double t[2] = {arm_elbow * cos(-elbow_tilt) + arm_wrist * cos(-elbow_tilt - wrist_tilt), arm_elbow * sin(-elbow_tilt) + arm_wrist * sin(-elbow_tilt  - wrist_tilt)};
    // double point01_vec[3] = {(point1[0] - point0[0]) / arm_shoulder, (point1[1] - point0[1]) / arm_shoulder , (point1[2] - point0[2]) / arm_shoulder};
    // trigonometric_3d[0] = point1[0] + point01_vec[0] * t[0] + cos(shoulder_roll) * t[1];
    // trigonometric_3d[1] = point1[1] + point01_vec[1] * t[0];
    // trigonometric_3d[2] = point1[2] + point01_vec[2] * t[0] + sin(shoulder_roll) * t[1];

    cout << "x座標: " << trigonometric_3d[0] << endl;
    cout << "y座標: " << trigonometric_3d[1] << endl;
    cout << "z座標: " << -trigonometric_3d[2]<< endl << endl;
}

void position_sub(const oculus_telexistence::Part::ConstPtr& data) {
    targetX = data -> Left.position.x;
    targetY = data -> Left.position.y;
    targetZ = data -> Left.position.z;
    rotationX = data -> Left.rotation.x;
    rotationY = data -> Left.rotation.y;
    rotationZ = data -> Left.rotation.z;
}

void position_cb(const sensor_msgs::JointState::ConstPtr& data) {
    shoulder_roll = data -> position[3];
    shoulder_pan = data -> position[4];
    elbow_tilt = -data -> position[5];
    wrist_tilt = -data -> position[6];
    body_roll = data -> position[2];
}

void attyon(){
    cout << "x座標: " << targetZ  << endl;
    cout << "z座標: " << targetY << endl << endl;
    cout << "修正" << endl << "x座標: " << round(targetZ * 367.887 / 0.6 * 1000) / 1000  << endl;
    cout << "z座標: " << round(targetY * 509.84 / 0.9 * 1000) / 1000 << endl << endl;
    // cout << "y回転: " << rotationX << endl;
    // cout << "z回転: " << rotationY << endl;
    // cout << "x回転: " << rotationZ << endl << endl;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "jacobian_inverse_kinematics_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/joint_states", 10, position_cb);
  ros::Subscriber sub = nh.subscribe("/oculus/3d", 10, position_sub);
  ros::Rate loop_rate(1);  // 10Hzのループレート
  while (ros::ok()){
    Kinematics();
    // InverseKinematics_2d();
    // attyon();
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
