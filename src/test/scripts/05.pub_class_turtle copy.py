import rospy
from geometry_msgs.msg import Twist

class Pub_class:
    def __init__(self):
        rospy.init_node("wego_pub")
        self.publisher=rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=1)
        self.num=0
        self.msg=Twist()
        self.msg.linear.x


        self.rate=rospy.Rate(10)

    def pub_run(self):
        while not rospy.is_shutdown():
            self.msg.linear.x=0.2
            self.msg.linear.y=0.2
            self.msg.angular.z=0.5

            self.publisher.publish(self.msg)
            self.num+=1
            self.rate.sleep()

if __name__=="__main__":
    pub_class=Pub_class()
    pub_class.pub_run()
