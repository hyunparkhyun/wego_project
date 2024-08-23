#!/usr/bin/env python3
#-*- coding:utf-8*-

import rospy
from math import *
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from random import*

class Sub_class:
    def __init__(self):
        rospy.init_node("wego_pub")
        self.pub_=
        self.pub_speed=rospy.Publisher("/commands/motor/speed",Float64,queue_size=1)
        rospy.Subscriber("/scan",LaserScan,callback=self.callback)

        self.lidar_msg=LaserScan()
        self.speed_msg=Float64()
        self.angle_flag = 0
    
    def callback(self,msg):
        if self.angle_flag ==0:
            self.angles=[(msg.angle_min+msg.angle_increment*index)*180/pi for index,value in enumerate(msg.ranges)]
            self.angle_flag=1
        obstacle = 0
        for index,value in enumerate(msg.ranges):
            left_obstacle=0
            right_obstacle=0
            steer=0.5      
            if 0<value<0.5 and 150 < abs(self.angles[index])< 180:
                print(f"장애물:{self.angles[index]}")
                if -180 <self.angles[index]<-150:
                    left_obstacle=left_obstacle+1
                else:
                    right_obstacle=right_obstacle+1
                obstacle+=1
                print("obstacle", obstacle)
            else:
                pass
        if left_obstacle>right_obstacle:
            steer=0.5
        else:
            if left_obstacle<right_obstacle:
                steer=1
            else:
                steer=0
        # if 0<obstacle:
        #     speed_msg=0.0
        # else:
        #     speed_msg=5000.0

        #     self.speed_msg.data=speed_msg  
        #     self.publisher.publish(self.speed_msg)              
if __name__=="__main__":
    sub_class=Sub_class()
    rospy.spin()
