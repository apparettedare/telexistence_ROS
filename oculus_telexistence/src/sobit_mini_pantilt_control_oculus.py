#!/usr/bin/env python3
import rospy
from sobit_mini_module import SobitMiniController
from sobit_mini_module import Joint
from oculus_telexistence.msg import *
import sys
import math

def oculussub(data):
    rospy.loginfo("Subscribe x: " + str(data.Head.rotation.x))
    rospy.loginfo("Subscribe y: " + str(data.Head.rotation.y))

    args = sys.argv
    mini_pantilt_ctr = SobitMiniController(args[0]) # args[0] : C++上でros::init()を行うための引数

    sclpan = 1.2
    scltilt = 0.9

    pan = math.floor(data.Head.rotation.y * sclpan * 10)/10
    tilt = math.floor(data.Head.rotation.x * scltilt * 10)/10
    # print("pan" + str(pan))
    # print("tilt" + str(tilt))

    # if(pan > 1.5):
    #     pan = 1.5
    # elif(pan < -1.5):
    #     pan = -1.5

    # if(tilt > 0.5):
    #     tilt = 0.5
    # elif(tilt < -0.5):
    #     tilt = -0.5


    # # カメラパンチルトを動かす
    # mini_pantilt_ctr.moveHeadPanTilt(-pan, -tilt, 0.01, False)

def main():
    rospy.init_node('sobit_mini_pantilt_control_oculus_node')
    rospy.Subscriber("oculus/3d", Part, oculussub)
    rospy.spin()

if __name__ == '__main__':
    main()
