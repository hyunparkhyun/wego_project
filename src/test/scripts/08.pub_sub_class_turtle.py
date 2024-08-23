import rospy
from turtlesim.msg import Pose
from math import *
from geometry_msgs.msg import Twist

class Sub_class:
    def __init__(self):
        rospy.init_node("wego_sub")
        rospy.Subscriber("/turtle1/pose",Pose,callback=self.callback)
        self.publisher=rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=1)
        self.twist_msg=Twist()
        self.pose_msg=Pose()      

    def callback(self,msg):
        print(msg.x)
        print(msg.y)
        print(msg.theta*180/pi)
        if msg.x < 8:
            self.swist_msg.linear.x=1
        else:
            self.swist_msg.linear.x=0
        self.publisher.publish(self.twist_msg)
if __name__=="__main__":
    sub_class=Sub_class()
    rospy.spin()
