import rospy
from std_msgs.msg import Int32

rospy.init_node("wego_pub")
publisher=rospy.Publisher("count",Int32,queue_size=1)
num=0
msg=Int32()

rate=rospy.Rate(10)
while not rospy.is_shutdown():
    msg.data=num
    publisher.publish(msg)
    num+=1
    rate.sleep()
