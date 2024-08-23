import rospy
from math import *

from turtlesim.msg import Pose

class Sub_class:
    def __init__(self):
        rospy.init_node("wego_sub")
        rospy.Subscriber("/turtle1/pose",Pose,callback=self.callback)
        msg=Pose()
    def callback(self,msg):
        print(msg.x)
        print(msg.y)
        print(msg.theta*180/pi)

if __name__=="__main__":
    sub_class=Sub_class()
    rospy.spin()
