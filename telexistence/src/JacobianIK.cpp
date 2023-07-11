#include "sobit_mini_library/sobit_mini_controller.hpp"
#include "oculus_telexistence/Part.h"
#include <ros/ros.h>
#include <cmath>
#include <iostream>

using namespace std;

// ジョイントの角度
double theta1, theta2, theta3;

// 目標位置
double targetX = 0.0;
double targetY = 0.0;

// リンクの長さ
const double arm_shoulder = 110.96352;
const double arm_elbow = 102.57457;
const double arm_wrist = 156.55295;

// ヤコビ行列を計算する関数
void calculateJacobian(double theta1, double theta2, double theta3, double jacobian[2][3]) {
    // ヤコビ行列の要素を計算
    jacobian[0][0] = -arm_shoulder * sin(theta1) - arm_elbow * sin(theta1 + theta2) - arm_wrist * sin(theta1 + theta2 + theta3);
    jacobian[0][1] = -arm_elbow * sin(theta1 + theta2) - arm_wrist * sin(theta1 + theta2 + theta3);
    jacobian[0][2] = -arm_wrist * sin(theta1 + theta2 + theta3);
    
    jacobian[1][0] = arm_shoulder * cos(theta1) + arm_elbow * cos(theta1 + theta2) + arm_wrist * cos(theta1 + theta2 + theta3);
    jacobian[1][1] = arm_elbow * cos(theta1 + theta2) + arm_wrist * cos(theta1 + theta2 + theta3);
    jacobian[1][2] = arm_wrist * cos(theta1 + theta2 + theta3);
}

void InverseKinematics() {
    // 逆運動学の解を格納する変数
    double deltaTheta1, deltaTheta2, deltaTheta3;
    
    // 収束条件
    double epsilon = 0.01;
    
    // 最大反復回数
    int maxIterations = 1000;
    
    // 反復回数
    int iteration = 0;
    
    // ヤコビ行列
    double jacobian[2][3];
    
    // 逆運動学の反復計算
    do {
        // ヤコビ行列を計算
        calculateJacobian(theta1, theta2, theta3, jacobian);
        
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
        double deltaX = targetX - (arm_shoulder * cos(theta1) + arm_elbow * cos(theta1 + theta2) + arm_wrist * cos(theta1 + theta2 + theta3));
        double deltaY = targetY - (arm_shoulder * sin(theta1) + arm_elbow * sin(theta1 + theta2) + arm_wrist * sin(theta1 + theta2 + theta3));
        
        // 角度の変化量を計算
        deltaTheta1 = jacobianInverse[0][0] * deltaX + jacobianInverse[0][1] * deltaY;
        deltaTheta2 = jacobianInverse[1][0] * deltaX + jacobianInverse[1][1] * deltaY;
        deltaTheta3 = jacobianInverse[2][0] * deltaX + jacobianInverse[2][1] * deltaY;
        
        // 角度を更新
        theta1 += deltaTheta1;
        theta2 += deltaTheta2;
        theta3 += deltaTheta3;
        
        // 反復回数をインクリメント
        iteration++;
    } while ((fabs(deltaTheta1) > epsilon || fabs(deltaTheta2) > epsilon || fabs(deltaTheta3) > epsilon) && iteration < maxIterations);
    
    // 結果を出力
    cout << "Theta1: " << theta1 << endl;
    cout << "Theta2: " << theta2 << endl;
    cout << "Theta3: " << theta3 << endl;
    
    sobit_mini::SobitMiniController mini_arm_ctr;
    mini_arm_ctr.moveRightArm( theta1, 0, -theta2, -theta3, 0.0);

}

void Kinematics() {
    double trigonometric[2];
    trigonometric[0] = arm_shoulder * cos(theta1) + arm_elbow * cos(theta1 + theta2) + arm_wrist * cos(theta1 + theta2 + theta3);
    trigonometric[1] = arm_shoulder * sin(theta1) + arm_elbow * sin(theta1 + theta2) + arm_wrist * sin(theta1 + theta2 + theta3);
    cout << "x座標: " << trigonometric[0] << endl;
    cout << "y座標: " << trigonometric[1] << endl;
}

void position_sub(const oculus_telexistence::Part::ConstPtr& data) {
    targetX = data -> Right.position.x;
    targetY = data -> Right.position.y;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sobit_mini_arm_control_oculus_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/oculus/3d", 10, position_sub);
  InverseKinematics();
  Kinematics();
  ros::spin();
}

