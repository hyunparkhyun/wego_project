import rospy
from std_msgs.msg import Int32

class Sub_class:
    def __init__(self):
        rospy.init_node("wego_sub")
        rospy.Subscriber("count",Int32,callback=self.callback)
        
    def callback(self,msg):
        print(msg.data*2)

if __name__=="__main__":
    sub_class=Sub_class()
    rospy.spin()
