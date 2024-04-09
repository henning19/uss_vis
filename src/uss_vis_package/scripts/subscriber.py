#!/usr/bin/env python
import rospy
from std_msgs.msg import String
#from data_type_init import Coordinates
from geometry_msgs.msg import Point
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "X-Coordinates :%f Y-Coordinates: %f", data.x, data.y)
    
def subscriber():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('subscriber', anonymous=True)

    rospy.Subscriber("chatter", Point, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscriber()