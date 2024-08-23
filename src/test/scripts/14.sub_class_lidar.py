#!/usr/bin/env python3
#-*- coding:utf-8*-
#유태현 박현 바보
import rospy
from sensor_msgs.msg import CompressedImage


from math import *
import os
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np

# from turtlesim.msg import Pose


class Sub_class:
    def __init__(self):
        rospy.init_node("wego_sub")
        rospy.Subscriber("/usb_cam/image_raw/compressed",CompressedImage,callback=self.camera_callback)
        rospy.Subscriber("/scan",LaserScan,callback=self.lidar_callback)
        self.pub_speed=rospy.Publisher("/commands/motor/speed",Float64,queue_size=1)
        self.pub_steer=rospy.Publisher("/commands/servo/position",Float64, queue_size=1)

        image_msg=CompressedImage()

        self.bridge=CvBridge()
        
        self.lidar_msg=LaserScan()
        self.speed_msg=Float64()
        self.steer_msg=Float64()
        
        self.angles = []
        self.index = 0
        self.angle_flag=0
        self.lidar_flag=0
        self.camera_flag=0
        
        # self.angle_flag=0
    def camera_callback(self,msg):
        cv_img=self.bridge.compressed_imgmsg_to_cv2(msg)
        img_y,img_x,img_channel= cv_img.shape
        hsv_img = cv2.cvtColor(cv_img,cv2.COLOR_BGR2HSV)
      
        yellow_lower = (15,20,150)
        yellow_upper = (45,255,255)
        yellow_range_img = cv2.inRange(hsv_img, yellow_lower,yellow_upper)
        yellow_masked_img=cv2.bitwise_and(cv_img,cv_img,mask=yellow_range_img)

        white_lower = (0,0,150)    
        white_upper = (180,45,255)    
        white_range_img = cv2.inRange(hsv_img, white_lower,white_upper)          
        white_masked_img=cv2.bitwise_and(cv_img,cv_img,mask=white_range_img)

        # blended_masked_img=cv2.bitwise_or(yellow_masked_img,white_masked_img)

        x1=160
        y1=480
        x2=260
        y2=330

        pt1=((x1,y1))
        pt2=((x2,y2))
        pt3=((img_x-x2,y2))
        pt4=((img_x-x1,y1))
        src_pts = np.float32([pt1,pt2,pt3,pt4])
        cv2.circle(cv_img,pt1,10,[255,0,0],-1)
        cv2.circle(cv_img,pt2,10,[0,255,0],-1)
        cv2.circle(cv_img,pt3,10,[0,0,255],-1)
        cv2.circle(cv_img,pt4,10,[0,255,255],-1) 
        
        dst_pt1 = [100,480]
        dst_pt2 = [100, 0]
        dst_pt3 = [400, 0]
        dst_pt4 = [400, 480]

        dst_pts = np.float32([dst_pt1,dst_pt2, dst_pt3, dst_pt4])
        matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        wraped_img= cv2.warpPerspective(cv_img, matrix,(500,500))
        cv2.imshow("img", cv_img)
        cv2.imshow("cv_img",wraped_img)
        cv2.waitKey(1)
    
    def lidar_callback(self, msg):
        if msg !=-1:
            self.lidar_msg=msg
            if self.angle_flag==0:
                self.angles=[(self.lidar_msg.angle_min+self.lidar_msg.angle_increment*index)*180/pi for index, value in enumerate(self.lidar_msg.ranges)]
                self.angle_flag=1
            self.lidar_flag=1
        else:
            self.lidar_flag= False

    def camera_callback(self,msg):
        if msg!=-1:
            self.image_msg=msg
            self.camera_flag=True
        else:
            self.camera_flag=False
    def avoidance(self):
        obstacles=0
        left_obstacle=0
        right_obstacle=0
        for index,value in enumerate(self.angles):
            
            
            if 0<value<0.5 and 150<abs(self.angles[index])<180:
                print(f"장애물:{self.angles[index]}")
                if -180<self.angles[index]<-150:
                    left_obstacle=left_obstacle+1
                else:
                    right_obstacle=right_obstacle+1
                # obstacles=obstacles+1
            else:
                pass
        if left_obstacle == 0 and right_obstacle == 0:
            self.speed_msg = 2000
            self.steer_msg = 0.5
        
        else:
            self.speed_msg = 1000
            if left_obstacle > right_obstacle:
                self.steer_msg = 0.5 + min(0.05*right_obstacle,0.5)
            else:
                self.steer_msg = 0.5 -min(0.05*left_obstacle,0.5)
        self.pub_speed.publish(self.speed_msg)
        self.pub_steer.publish(self.steer_msg)
    def run(self):
        while not rospy.is_shutdown():
            
            self.avoidance()
if __name__=="__main__":
    sub_class=Sub_class()
    sub_class.run()