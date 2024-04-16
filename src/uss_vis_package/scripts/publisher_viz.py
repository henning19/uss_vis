#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Float64MultiArray
from geometry_msgs.msg import Point32

import numpy as np
import copy

def publisher(data):                # Exspecting FLoat64MultiArray as Input
    pub = rospy.Publisher('points_dtype_cloud', PointCloud2, queue_size=10)
    rospy.loginfo("Veröffentlichte Koordinaten für 10 Punkte gleichzeitig: %s", str(type(data.data)))
    rospy.init_node('publisher_viz', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        # Header erstellen
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "dummy_cor"

        # Punkt-Wolken-Nachricht erstellen
        points = []
        coordinates_info = ""  # String zur Speicherung der Koordinaten für rospy.loginfo
        """
        for i in range(10):  # Nur 10 Punkte pro Iteration
            x = np.random.uniform(-20, 20)  # Zufällige x-Koordinate zwischen -20 und 20
            y = np.random.uniform(-20, 20)  # Zufällige y-Koordinate zwischen -20 und 20
            point = Point32()
            point.x = x
            point.y = y
            point.z = 0.0
            points.append(point)
            coordinates_info += "({}, {}) ".format(x, y)  # Koordinaten dem String hinzufügen
        """
        points_without_z=copy.deepcopy(data.data)
        
        j=-1
        for i in range(0,len(points_without_z)):
            if i==j+2:
                j=i+1
                points.append(0.0)
            points.append(points_without_z[i])

        # Punkte in ein numpy-Array konvertieren
        points_arr = np.zeros((len(points), 3), dtype=np.float32)
        for i, p in enumerate(points):
            points_arr[i, 0] = p.x
            points_arr[i, 1] = p.y
            points_arr[i, 2] = p.z

        # Punkt-Wolken-Nachricht erstellen
        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = header
        point_cloud_msg.height = 1
        point_cloud_msg.width = len(points)
        point_cloud_msg.fields.append(PointField(
            name="x", offset=0, datatype=7, count=1))
        point_cloud_msg.fields.append(PointField(
            name="y", offset=4, datatype=7, count=1))
        point_cloud_msg.fields.append(PointField(
            name="z", offset=8, datatype=7, count=1))
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.point_step = 12
        point_cloud_msg.row_step = 12 * len(points)
        point_cloud_msg.is_dense = True
        point_cloud_msg.data = points_arr.tostring()

        # Die Punkt-Wolken-Nachricht veröffentlichen
        pub.publish(point_cloud_msg)

        rospy.loginfo("Veröffentlichte Koordinaten für 10 Punkte gleichzeitig: {}".format(coordinates_info))
        rate.sleep()
"""
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
"""
