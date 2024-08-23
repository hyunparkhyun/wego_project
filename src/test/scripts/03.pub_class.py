import rospy
from std_msgs.msg import Int32

class Pub_class:
    def __init__(self):
        rospy.init_node("wego_pub")
        self.publisher=rospy.Publisher("count",Int32,queue_size=1)
        self.num=0
        self.msg=Int32()
        self.rate=rospy.Rate(10)

    def pub_run(self):
        
        while not rospy.is_shutdown():
            self.msg.data=self.num
            self.publisher.publish(self.msg)
            self.num+=1
            self.rate.sleep()

if __name__=="__main__":
    pub_class=Pub_class()
    pub_class.pub_run()
