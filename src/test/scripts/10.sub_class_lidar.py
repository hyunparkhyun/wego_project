#!/usr/bin/env python3
#-*- coding:utf-8*-

import rospy
from sensor_msgs.msg import LaserScan
from math import *
import os
from turtlesim.msg import Pose

class Sub_class:
    def __init__(self):
        rospy.init_node("wego_sub")
        rospy.Subscriber("/scan",LaserScan,callback=self.callback)
        msg=LaserScan()
        msg.ranges
        self.angle_flag=0
    def callback(self,msg):
        os.system('clear')
        #print(f"msg.ranges:{msg.ranges[0:100]}")
        #print(f"msg.angle_increment:{msg.angle_increment*180/pi}")
        #print(f"msg.angle_max:{msg.angle_max*180/pi}")
        #print(f"msg.angle_min:{msg.angle_min*180/pi}")
        #print(f"msg.range_max:{msg.range_max}")
        #print(f"msg.range_min:{msg.range_min}")
        #degree to radian
        #*180/pi
        #index=0
        #angles=[]


        #각도별 거리.
        #for n in msg.ranges:
        #     angle=(msg.angle_min+msg.angle_increment*index)*180/pi
        #     index+=1
        #     angles.append(angle)
        # print(angles)
        
        #한줄로
        if self.angle_flag ==0:
            self.angles=[(msg.angle_min+msg.angle_increment*index)*180/pi for index,value in enumerate(msg.ranges)]
            self.angle_flag=1
        # print(angles)
        for index,value in enumerate(msg.ranges):
            if 0<value<0.5 and 150 < abs(self.angles[index])< 180:
                print("장애물")

if __name__=="__main__":
    sub_class=Sub_class()
    rospy.spin()
