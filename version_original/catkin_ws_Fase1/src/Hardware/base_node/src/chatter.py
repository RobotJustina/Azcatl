#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32, Float32MultiArray

def talker():
    pub = rospy.Publisher('/hardware/mobile_base/speeds', Float32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    vel=Float32MultiArray()
    while not rospy.is_shutdown():
        
        vel.data = [0.5,0.5]
##        vel= -0.5
        rospy.loginfo(vel)
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
