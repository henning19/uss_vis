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

    # Veröffentliche die verarbeiteten Daten auf einem anderen Topic
    pub = rospy.Publisher('points_dtype_cloud', PointCloud2, queue_size=10)
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "dummy_cor"

    # Farbwerte für die Punkte
    color_map = {
        1: [255, 0, 0],      # Rot für Kategorie 1
        2: [255, 0, 0],    # Rot für Kategorie 2
        3: [255, 165, 0],  # Orange für Kategorie 3
        4: [0, 255, 0]     # Grün für Kategorie 4
    }

    # Erstelle Punkt-Wolken-Nachricht
    points = []
    category_vector = []
    colors = []
    for i in range(0, len(data.data), 2):
        point = Point32()
        point.x = data.data[i]
        point.y = data.data[i + 1]
        point.z = 0.0
        category = zone_cal(data.data[i], data.data[i+1])
        category_vector.append(category)
        points.append(point)

        # Wähle Farbe entsprechend der Kategorie
        color = color_map.get(category, [0, 0, 255])  # Default auf Blau setzen
        colors.append(color)

    rospy.loginfo("Categories of published points: %s" % str(category_vector))

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
    point_cloud_msg.fields.append(PointField(
        name="rgb", offset=12, datatype=7, count=1))  # Änderung hier

    point_cloud_msg.is_bigendian = False
    point_cloud_msg.point_step = 16
    point_cloud_msg.row_step = 16 * len(points)
    point_cloud_msg.is_dense = True

    # Konvertiere Punkte und Farben in ein numpy-Array
    points_arr = np.zeros((len(points), 3), dtype=np.float32)
    colors_arr = np.zeros((len(points), 1), dtype=np.uint32)
    for i, p in enumerate(points):
        points_arr[i, 0] = p.x
        points_arr[i, 1] = p.y
        points_arr[i, 2] = p.z
        rgb = (colors[i][0] << 16) | (colors[i][1] << 8) | colors[i][2]
        colors_arr[i] = rgb

    # Fülle die Daten der Punkt-Wolken-Nachricht
    point_cloud_msg.data = np.column_stack(
        (points_arr, colors_arr)).astype(np.float32).tostring()

    # Schreibe die Koordinaten in die loginfo
    coordinates = ""
    for p in points:
        coordinates += "({}, {}, {}) ".format(p.x, p.y, p.z)
    rospy.loginfo("Coordinates to be published: %s" % coordinates)

    # Veröffentliche die Punkt-Wolken-Nachricht
    pub.publish(point_cloud_msg)

def zone_cal(x, y):
    distance = 0
    width = 0.698     # Half width
    len_front = 1.999
    len_back = -0.339
    category = 0
    if x == 0.0 and y == 0.0:
        category = 10             # a not initialized point
        return category
    elif abs(y) <= width:
        if x >= len_front:
            distance = x - len_front
        elif x <= len_back:
            distance = abs(x - len_back)
        else:
            distance = 0
    elif x <= len_front and x >= len_back:
        distance = abs(y) - width
    elif abs(y) > width and x >= len_front:
        distance = np.sqrt(np.square(x - len_front) + np.square(y - width))
    elif abs(y) > width and x <= len_back:
        distance = np.sqrt(np.square(x - len_back) + np.square(y - width))

    if distance <= 1.0:
        category = 1
    elif distance <= 1.5:
        category = 2
    elif distance <= 2.0:
        category = 3
    elif distance > 2.0:
        category = 4

    return category

def listener():
    rospy.init_node('listener', anonymous=True)

    # Abonniere das "points" Topic, um Daten zu empfangen
    rospy.Subscriber("points", Float64MultiArray, callback)

    # Lasse das Programm laufen, bis es beendet wird
    rospy.spin()

if __name__ == '__main__':
    listener()
