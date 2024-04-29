#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Point32
import pygame
import time
import copy

import numpy as np

def callback(data):
    # Verarbeite die empfangenen Daten

    # Bearbeite die empfangenen Daten, indem nach jedem zweiten Wert eine 0.0 eingefügt wird

    #rospy.loginfo("Coordinates to be published1: %s", str(processed_data))
    # Veröffentliche die verarbeiteten Daten auf einem anderen Topic
    pub = rospy.Publisher('points_dtype_cloud', PointCloud2, queue_size=10)
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "dummy_cor"

    # Erstelle Punkt-Wolken-Nachricht
    points = []
    category_vector=[]
    for i in range(0, len(data.data), 2):
        point = Point32()
        point.x = data.data[i]
        point.y = data.data[i + 1]
        point.z = 0.0
        category_vector.append(zone_cal(data.data[i], data.data[i+1]))
        points.append(point)
    if min(category_vector)<10:
        audio_gen(min(category_vector))
    rospy.loginfo("Categories of published points: %s", str(category_vector))
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
    #calculate_sound(points_arr)
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


    


def audio_gen(category):
    frequencies = [16000, 12000, 8000, 2000]  # Example frequencies
    durations = [10.0, 20.0, 30.0]     # Example durations
    frequency=frequencies[category-1]
    pygame.mixer.init()
    sample_rate=44100
    sound_data=np.sin(2*np.pi * frequency * np.arange(sample_rate)/float(sample_rate)).astype(np.float32)
    pygame.mixer.Sound(sound_data).play()
    pygame.quit()

def zone_cal(x,y):
    distance=0
    width=0.698     # Half width
    len_front=1.999
    len_back=-0.339
    category=0
    if x==0.0 and y==0.0:
        category=10             # a not initilized point
        return(category)
    elif abs(y)<=width:
        if  x>=len_front:
            distance=x-len_front
        elif x<=len_back:
            distance=abs(x-len_back)
        else:
            distance=0
    elif x<=len_front and x>=len_back:
        distance=abs(y)-width
    elif abs(y)>width and x>=len_front:
        distance=np.sqrt(np.square(x-len_front)+np.square(y-width))
    elif abs(y)>width and x<=len_back:
        distance=np.sqrt(np.square(x-len_back)+np.square(y-width))
    
    if distance<=1.0:
        category=1
        return(category)
    elif distance<=1.5:
        category=2
        return(category)
    elif distance<=2.0:
        category=3
        return(category)
    elif distance>2.0:
        category=4
        return(category)
    else:
        return(False)
    
if __name__ == '__main__':
    listener()
