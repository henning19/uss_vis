#!/usr/bin/env python
import rospy
from std_msgs.msg import String , Float64MultiArray
#from data_type_init import Coordinates
#from geometry_msgs.msg import Point
import publisher_viz
import copy
data_return=Float64MultiArray()
def callback(data):
    global data_return
    rospy.loginfo(rospy.get_caller_id() + "X-Coordinates :%f Y-Coordinates: %f", data.data[0], data.data[1])
    data_return=copy.deepcopy(data.data)
    return(data)
def subscriber():
    global data_return

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('subscriber', anonymous=True)

    rospy.Subscriber("points", Float64MultiArray , callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    rospy.loginfo("X-Coordinates :%f Y-Coordinates: %f", data_return[0], data_return[1])       # wird nicht angezeigt
    publisher_viz.publisher(data_return)

if __name__ == '__main__':
    subscriber()