#!/usr/bin/env python
# license removed for brevity
import rospy
#from data_type_init import Coordinates
from std_msgs.msg import String
from geometry_msgs.msg import Point



def publisher():
    pub = rospy.Publisher('chatter', Point, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i=0.0
    while not rospy.is_shutdown():
        i=i+0.1
        x=10+i
        y=10-i
        dummy_cor=Point(x,y,0)
        if i==10:
            i=0
        pub.publish(dummy_cor)
        rospy.loginfo("Published coordinates: x=%f y=%f",x,y)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass