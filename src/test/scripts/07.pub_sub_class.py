import rospy
from math import *
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from random import*

class Pub_class:
    def __init__(self):
        rospy.init_node("wego_pub")
        self.publisher=rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=1)
        rospy.Subscriber("/turtle1/pose",Pose,callback=self.callback)

        self.twist_msg=Twist()
        self.pose_msg=Pose()
        self.rate=rospy.Rate(10)

    def pub_run(self):
        while not rospy.is_shutdown():
            self.twist_msg.linear.x=random()*2
            
            self.twist_msg.angular.z=(random()*2-1)

            

            self.publisher.publish(self.twist_msg)
            self.rate.sleep()

    def callback(self,msg):
        self.pose_msg=msg

if __name__=="__main__":
    pub_class=Pub_class()
    pub_class.pub_run()
