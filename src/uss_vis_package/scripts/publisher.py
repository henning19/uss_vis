#!/usr/bin/env python
# license removed for brevity
import rospy
#from data_type_init import Coordinates
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped 
from geometry_msgs.msg import Point


def publisher():
    pub = rospy.Publisher('chatter', PointStamped, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i=0.0
    while not rospy.is_shutdown():
        i=i+1
        x_i=-5+i
        y_i=-5+i
        dummy_cor=PointStamped()
        dummy_cor.point=Point(x=x_i, y=y_i, z=0)
        #dummy_cor.point.y=y_i
        #dummy_cor.point.z=0.0
        dummy_cor.header.stamp=rospy.Time.now()
        dummy_cor.header.frame_id="dummy_cor"
        if i==10:
            x_i=-5
            y_i=-5
            i=0
        pub.publish(dummy_cor)
        rospy.loginfo("Published coordinates: x=%f y=%f",x_i,y_i)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass