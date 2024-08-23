import rospy
from std_msgs.msg import Int32


rospy.init_node("wego_sub")
msg=Int32()
msg.data
def callback(msg):
    print(msg.data*2)


rospy.Subscriber("count",Int32,callback=callback)
rospy.spin()
