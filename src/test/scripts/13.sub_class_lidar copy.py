#!/usr/bin/env python3
#-*- coding:utf-8*-
import rospy
from sensor_msgs.msg import CompressedImage


from math import *
import os
import cv2
from cv_bridge import CvBridge
import numpy as np

# from turtlesim.msg import Pose


class Sub_class:
    def __init__(self):
        rospy.init_node("wego_sub")
        rospy.Subscriber("/usb_cam/image_raw/compressed",CompressedImage,callback=self.callback)
        image_msg=CompressedImage()
        self.bridge=CvBridge()
        # msg=CompressedImage()
        # msg.ranges
        # self.angle_flag=0
    def callback(self,msg):
        cv_img=self.bridge.compressed_imgmsg_to_cv2(msg)
        img_y,img_x,img_channel= cv_img.shape
        hsv_img = cv2.cvtColor(cv_img,cv2.COLOR_BGR2HSV)
        # h,s,v=cv2.split(hsv_img)
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

        #cv2.imshow("hsv_img",hsv_img)
        #cv2.imshow("yellow_masked_img",yellow_masked_img)    
        
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

        # print(msg)
        # os.system('clear')
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
        # if self.angle_flag ==0:
        #     self.angles=[(msg.angle_min+msg.angle_increment*index)*180/pi for index,value in enumerate(msg.ranges)]
        #     self.angle_flag=1
        # # print(angles)
        # for index,value in enumerate(msg.ranges):
        #     if 0<value<0.5 and 150 < abs(self.angles[index])< 180:
        #         print("장애물")

if __name__=="__main__":
    sub_class=Sub_class()
    rospy.spin()
