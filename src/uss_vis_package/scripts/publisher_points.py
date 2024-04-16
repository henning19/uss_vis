#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Float64MultiArray
from geometry_msgs.msg import Point32


import numpy as np

def publisher():
    pub = rospy.Publisher('points', Float64MultiArray, queue_size=10)
    rospy.init_node('publisher_points', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
       
        # Punkt-Wolken-Nachricht erstellen
        points = Float64MultiArray()
        coordinates_info = ""  # String zur Speicherung der Koordinaten für rospy.loginfo
        
        for i in range(10):  # Nur 10 Punkte pro Iteration
            x = np.random.uniform(-20, 20)  # Zufällige x-Koordinate zwischen -20 und 20
            y = np.random.uniform(-20, 20)  # Zufällige y-Koordinate zwischen -20 und 20
            points.data.append(x)
            points.data.append(y)
            coordinates_info += "({}, {}) ".format(x, y)  # Koordinaten dem String hinzufügen

        # Die Punkt-Wolken-Nachricht veröffentlichen
        pub.publish(points)
        rospy.loginfo("Veröffentlichte Koordinaten für 10 Punkte gleichzeitig: %s", coordinates_info)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

