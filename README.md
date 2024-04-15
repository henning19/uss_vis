# README
This is the README of the visualisation team of the ultrasonicsensor implementation team

-- Funktionierende Publisher.py um einzelne Punkte mit Pointcloud2 anzuzeigen: 

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Point32

import numpy as np

def publisher():
    pub = rospy.Publisher('chatter', PointCloud2, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 0.0
    while not rospy.is_shutdown():
        i += 1  # Inkrementiere i
        if i >= 10:  # Wenn i 10 erreicht, setze es auf 0 zurück
            i = 0.0
        x_i = -5 + i
        y_i = -5 + i

        # Header erstellen
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "dummy_cor"

        # Punkt-Wolken-Nachricht erstellen
        points = []
        point = Point32()
        point.x = x_i
        point.y = y_i
        point.z = 0.0
        points.append(point)

        # Punkte in ein numpy-Array konvertieren
        points_arr = np.zeros((len(points), 3), dtype=np.float32)
        for j, p in enumerate(points):
            points_arr[j, 0] = p.x  # Benutze j als Index, um Verwechslungen zu vermeiden
            points_arr[j, 1] = p.y
            points_arr[j, 2] = p.z

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

        rospy.loginfo("Veröffentlichte Koordinaten: x=%f y=%f", x_i, y_i)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
