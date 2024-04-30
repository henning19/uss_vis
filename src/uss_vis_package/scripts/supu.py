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
    # Order here: Green , Blue , Red 
    color_map = {
        1: [0, 0, 255],      # Rot für Kategorie 1
        2: [0, 0, 255],    # Rot für Kategorie 2
        3: [128, 0, 255],  # Orange für Kategorie 3
        4: [255, 0, 0]     # Grün für Kategorie 4
    }

    # Erstelle Punkt-Wolken-Nachricht
    points = []
    category_vector = []
    colors = []
    colors_vector = []
    for i in range(0, len(data.data), 2):
        point = Point32()
        point.x = data.data[i]
        point.y = data.data[i + 1]
        point.z = 0.0
        category = zone_cal(data.data[i], data.data[i+1])
        category_vector.append(category)
        points.append(point)

        # Wähle Farbe entsprechend der Kategorie
        color = color_map.get(category, [0, 255, 0])  # Default auf Blau setzen
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
        name="rgb", offset=12, datatype=7, count=1))

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
        rgba = (colors[i][0] << 16) | (colors[i][1] << 8) | colors[i][2] | (255 << 24)
        colors_arr[i] = rgba
    

    # Debugging Ausgabe
    """
    rospy.loginfo("Points of published points: %s" % str(points_arr))
    rospy.loginfo("2Colors of published points: %s" % str(colors_arr))
    # Konvertiere die 32-Bit-Integer in RGB-Farbwerte
    for rgba in colors_arr:
        # Extrahiere die Farbkanäle (Rot, Grün, Blau) aus dem 32-Bit-Integer
        r = (rgba >> 16) & 0xFF  # Rote Farbkomponente (bits 16-23)
        g = (rgba >> 8) & 0xFF   # Grüne Farbkomponente (bits 8-15)
        b = rgba & 0xFF          # Blaue Farbkomponente (bits 0-7)

        # Ausgabe der extrahierten Farbkomponenten
        print("R: {}, G: {}, B: {}".format(r, g, b))
    """
   
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