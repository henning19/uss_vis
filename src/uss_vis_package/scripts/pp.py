#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Float64MultiArray
from geometry_msgs.msg import Point32

import numpy as np
init=-10
def talker():
    pub = rospy.Publisher('points', Float64MultiArray, queue_size=10)
    rospy.init_node('talker_points', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    global init
    #x_init=[-10.0,-9.0,-8.0,-7.0,-6.0,-5.0,-4.0,-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,20.0]
    #y_init=[]
    
    while not rospy.is_shutdown():
        # Punkt-Wolken-Nachricht erstellen
        points = Float64MultiArray()
        coordinates_info = ""  # String zur Speicherung der Koordinaten für rospy.loginfo
        
        # Generiere 6 Koordinaten gleichzeitig
        for i in range(6):
            
            if i==1:
                x=init
                init=init+1
                if x==10:
                    init=-10
            else:
                x=0
            y=0
            """
            x=0
            y=-1.5
            """
            """
            if i==1:

                x=x_init[x_init.index(x)+1]
                y=0
                if x==20.0:
                    x=x_int[0]
            else:
                x=0
                y=0
            
            ###
            x = np.random.uniform(2, 20) * np.random.choice([-1, 1])  # Mindestens 2 Einheiten von 0 in x-Richtung entfernt
            y = np.random.uniform(2, 20) * np.random.choice([-1, 1])  # Mindestens 2 Einheiten von 0 in y-Richtung entfernt
            """
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
