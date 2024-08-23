#!/usr/bin/env python3
#-*- coding:utf-8*-

import rospy
from std_msgs.msg import Float64


class Pub_class:
    def __init__(self):
        rospy.init_node("wego_pub")
        self.speed_pub=rospy.Publisher("/commands/motor/speed",Float64,queue_size=1)
        self.steer_pub=rospy.Publisher("/commands/servo/position",Float64,queue_size=1)

        self.num=0
        self.speed_msg=Float64()
        self.steer_msg=Float64()
   
        self.rate=rospy.Rate(10)

    def pub_run(self):
        while not rospy.is_shutdown():
            self.speed_msg.data=self.num%5000
            self.steer_msg.data=0.5+((self.num%61-30)/30*0.5)
            self.speed_pub.publish(self.speed_msg)
            self.steer_pub.publish(self.steer_msg)
            self.num+=1
            self.rate.sleep()

if __name__=="__main__":
    pub_class=Pub_class()
    pub_class.pub_run()
