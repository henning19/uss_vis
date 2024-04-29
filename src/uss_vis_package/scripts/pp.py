#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Float64MultiArray
from geometry_msgs.msg import Point32

import numpy as np

def talker():
    pub = rospy.Publisher('points', Float64MultiArray, queue_size=10)
    rospy.init_node('talker_points', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        # Punkt-Wolken-Nachricht erstellen
        points = Float64MultiArray()
        coordinates_info = ""  # String zur Speicherung der Koordinaten für rospy.loginfo

        """
        # Generiere 6 Koordinaten gleichzeitig
        for i in range(6):
            # Zufällige x- und y-Koordinaten zwischen -6 und 6 generieren
            
            x = np.random.uniform(-6, 6)
            y = np.random.uniform(-6, 6)

            # Überprüfen, ob x und y größer als 2 oder kleiner als -2 sind
            while abs(x) < 2 or abs(y) < 2:
                x = np.random.uniform(-6, 6)
                y = np.random.uniform(-6, 6)
            
            # Testdiagonale
            x = 3 - i
            y = -3 + i +1

            points.data.append(x)
            points.data.append(y)
            coordinates_info += "({}, {}) ".format(x, y)  # Koordinaten dem String hinzufügen
        """
        points.data = [4.5, 0, 3.75, 0, 3.25, 0, 2.75, 0, 0, 0, 0, 3]

        # Die Punkt-Wolken-Nachricht veröffentlichen
        pub.publish(points)
        #rospy.loginfo("Veröffentlichte Koordinaten für 6 Punkte gleichzeitig: %s", coordinates_info)
        rospy.loginfo("Veröffentlichte Koordinaten")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
