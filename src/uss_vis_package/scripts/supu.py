#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Point32

import numpy as np

def callback(data):
    # Verarbeite die empfangenen Daten

    # Bearbeite die empfangenen Daten, indem nach jedem zweiten Wert eine 0.0 eingefügt wird
    processed_data = []
    for i, value in enumerate(data.data):
        processed_data.append(value)
        if (i + 1) % 2 == 0:
            processed_data.append(0.0)

    # Veröffentliche die verarbeiteten Daten auf einem anderen Topic
    pub = rospy.Publisher('points_dtype_cloud', PointCloud2, queue_size=10)
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "dummy_cor"

    # Erstelle Punkt-Wolken-Nachricht
    points = []
    for i in range(0, len(processed_data), 2):
        point = Point32()
        point.x = processed_data[i]
        point.y = processed_data[i + 1]
        point.z = 0.0
        points.append(point)

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

    # Konvertiere Punkte in ein numpy-Array
    points_arr = np.zeros((len(points), 3), dtype=np.float32)
    for i, p in enumerate(points):
        points_arr[i, 0] = p.x
        points_arr[i, 1] = p.y
        points_arr[i, 2] = p.z

    # Fülle die Daten der Punkt-Wolken-Nachricht
    point_cloud_msg.data = points_arr.tostring()

    # Schreibe die Koordinaten in die loginfo
    coordinates = ""
    for p in points:
        coordinates += "({}, {}, {}) ".format(p.x, p.y, p.z)
    rospy.loginfo("Coordinates to be published: %s", coordinates)

    # Veröffentliche die Punkt-Wolken-Nachricht
    pub.publish(point_cloud_msg)

def listener():
    rospy.init_node('listener', anonymous=True)

    # Abonniere das "points" Topic, um Daten zu empfangen
    rospy.Subscriber("points", Float64MultiArray, callback)

    # Lasse das Programm laufen, bis es beendet wird
    rospy.spin()

if __name__ == '__main__':
    listener()
