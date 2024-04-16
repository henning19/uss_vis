#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def publish_image():
    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher('/image_raw', Image, queue_size=10)
    rate = rospy.Rate(1)  # Veröffentlichungsfrequenz (1 Hz)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        # Lade das Bild von der Festplatte
        image = cv2.imread('/uss_vis/src/uss_vis_package/textures/car.png')
        # Konvertiere das Bild in ein ROS-Image
        ros_image = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        # Veröffentliche das ROS-Image
        pub.publish(ros_image)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass

