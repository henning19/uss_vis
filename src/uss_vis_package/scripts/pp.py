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

        # Generiere 6 Koordinaten gleichzeitig
        for i in range(6):
            x = np.random.uniform(2, 20) * np.random.choice([-1, 1])  # Mindestens 2 Einheiten von 0 in x-Richtung entfernt
            y = np.random.uniform(2, 20) * np.random.choice([-1, 1])  # Mindestens 2 Einheiten von 0 in y-Richtung entfernt
            points.data.append(x)
            points.data.append(y)
            coordinates_info += "({}, {}) ".format(x, y)  # Koordinaten dem String hinzufügen

        # Die Punkt-Wolken-Nachricht veröffentlichen
        pub.publish(points)
        rospy.loginfo("Veröffentlichte Koordinaten für 6 Punkte gleichzeitig: %s", coordinates_info)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
