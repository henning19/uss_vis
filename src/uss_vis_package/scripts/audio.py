#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math # type: ignore
import rospy
import sys
from sensor_msgs.msg import Range
from std_msgs.msg import Float64MultiArray


# create distances
sensor0_distance = None
sensor1_distance = None
sensor2_distance = None
sensor3_distance = None
sensor4_distance = None
sensor5_distance = None
sensor6_distance = None
sensor7_distance = None

# sensor positions
s0 = [1.8, -0.3]  # front left
s1 = [1.8, 0.3]   # front right
s2 = [1.4, 0.4]   # right side front
s3 = [0, 0.4]     # right side back
s4 = [-0.4, 0.25] # back right
s5 = [-0.4, -0.25]# back left
s6 = [0, -0.4]    # left side back
s7 = [1.4, -0.4]  # left side front

# create list for object-positions  
objects_list = [0, 0, 0, 0, 0, 0, 0, 0]


# normalize objects to decide beetween list or list-list
def normalizeObject(detectedObject):

    if isinstance(detectedObject[0], list):
        return detectedObject[0]
    else:
        return detectedObject

# calculate object-position in a global vehicle-coordinate-system
def calculateObject(sensor1, distance_sensor1_object, sensor2, distance_sensor2_object):
    s1_x, s1_y = sensor1
    s2_x, s2_y = sensor2

    deltaX = s2_x - s1_x
    deltaY = s2_y - s1_y
    distance_sensor1_sensor2 = math.sqrt(deltaX**2 + deltaY**2)
    
    x = (distance_sensor1_object**2 + distance_sensor1_sensor2**2 - distance_sensor2_object**2) / (2*distance_sensor1_sensor2)
    y = distance_sensor1_object**2 - x**2

    if y < 0:
        return [[0, 0]]
    if y > 0:
        y = math.sqrt(y)

    ex1 = deltaX / distance_sensor1_sensor2
    ex2 = deltaY / distance_sensor1_sensor2
    ey1 = -ex2
    ey2 = ex1
    p1x = s1_x + x * ex1
    p1y = s1_y + x * ex2
    
    if y == 0:
        return [[p1x, p1y]]
    
    p2x = p1x - y * ey1
    p2y = p1y - y * ey2
    p1x += y * ey1
    p1y += y * ey2

    return [[p1x, p1y], [p2x, p2y]]

# test if distance is valid for the calculation
def is_distance_valid(distance):
    return distance is not None and 0.2 < distance < 3.3

# filter wrong position front 
def check_positions_front(positions):

    if len(positions) == 1: # type: ignore
        return positions
    elif len(positions) == 2: # type: ignore
        if positions[0][1] > positions[1][1]:
            return positions[0]
        else:
            return positions[1]    

# filter wrong position right side
def check_positions_right(positions):

    if len(positions) == 1: # type: ignore
        return positions
    elif len(positions) == 2: # type: ignore
        if positions[0][0] > positions[1][0]:
            return positions[0]
        else:
            return positions[1]

# filter wrong position back
def check_positions_back(positions):
     if len(positions) == 1: # type: ignore
         return positions
     elif len(positions) == 2: # type: ignore
         if positions[0][1] < positions[1][1]:
             return positions[0]
         else:
             return positions[1] 

# filter wrong position left side
def check_positions_left(positions):

    if len(positions) == 1: # type: ignore
        return positions
    elif len(positions) == 2: # type: ignore
        if positions[0][0] < positions[1][0]:
            return positions[0]
        else:
            return positions[1]


"""
callbacks for sensors 0 to 7
first create global sensor distance
write data from ros-sensor in the global var

in every second callback (from a sensor-pair) we're calculating the position of the object 
we sperate beetween 5 cases:
case 1: both distance are valid, object can be calculated
case 2: one sensor-distance is too small, no calculation
case 3: both sensor-distances are too small, no calculation
case 4: one sensor-distance is too far, no calculation
case 5: both sensor-distance are too far, no calculation
"""


def callback_sensor0(data):
    global sensor0_distance
    sensor0_distance = data.range


def callback_sensor1(data):
    global sensor1_distance
    sensor1_distance = data.range
    detectedObject = [0, 0]
    if is_distance_valid(sensor0_distance) and is_distance_valid(sensor1_distance):
        detectedObject = normalizeObject(check_positions_front(calculateObject(s0, sensor0_distance, s1, sensor1_distance)))
    elif sensor0_distance is not None and sensor0_distance < 0.2 and sensor1_distance is not None and sensor1_distance < 0.2:
        detectedObject = [-100, -101]
    elif sensor0_distance is not None and sensor0_distance < 0.2:
        detectedObject = [-100, -100]
    elif sensor1_distance is not None and sensor1_distance < 0.2:
        detectedObject = [-101, -101]
    elif sensor0_distance == float('inf') and sensor1_distance == float('inf'):
        detectedObject = [100, 101]
    elif sensor0_distance == float('inf'):
        detectedObject = [100, 100]
    elif sensor1_distance == float('inf'):
        detectedObject = [101, 101]


    objects_list[0] = detectedObject[0]
    objects_list[1] = detectedObject[1]


def callback_sensor2(data):
    global sensor2_distance
    sensor2_distance = data.range


def callback_sensor3(data):
    global sensor3_distance
    sensor3_distance = data.range
    detectedObject = [0, 0]
    if is_distance_valid(sensor2_distance) and is_distance_valid(sensor3_distance):
        detectedObject = normalizeObject(check_positions_right(calculateObject(s2, sensor2_distance, s3, sensor3_distance)))
    elif sensor2_distance is not None and sensor2_distance < 0.2 and sensor3_distance is not None and sensor3_distance < 0.2:
        detectedObject = [-102, -103]
    elif sensor2_distance is not None and sensor2_distance < 0.2:
        detectedObject = [-102, -102]
    elif sensor3_distance is not None and sensor3_distance < 0.2:
        detectedObject = [-103, -103]
    elif sensor2_distance == float('inf') and sensor3_distance == float('inf'):
        detectedObject = [102, 103]
    elif sensor2_distance == float('inf'):
        detectedObject = [102, 102]
    elif sensor3_distance == float('inf'):
        detectedObject = [103, 103]


    objects_list[2] = detectedObject[0]
    objects_list[3] = detectedObject[1]


def callback_sensor4(data):
    global sensor4_distance
    sensor4_distance = data.range


def callback_sensor5(data):
    global sensor5_distance
    sensor5_distance = data.range
    detectedObject = [0, 0]
    if is_distance_valid(sensor4_distance) and is_distance_valid(sensor5_distance):
        detectedObject = normalizeObject(check_positions_back(calculateObject(s4, sensor4_distance, s5, sensor5_distance)))
    elif sensor4_distance is not None and sensor4_distance < 0.2 and sensor5_distance is not None and sensor5_distance < 0.2:
        detectedObject = [-104, -105]
    elif sensor4_distance is not None and sensor4_distance < 0.2:
        detectedObject = [-104, -104]
    elif sensor5_distance is not None and sensor5_distance < 0.2:
        detectedObject = [-105, -105]
    elif sensor4_distance == float('inf') and sensor5_distance == float('inf'):
        detectedObject = [104, 105]
    elif sensor4_distance == float('inf'):
        detectedObject = [104, 104]
    elif sensor5_distance == float('inf'):
        detectedObject = [105, 105]


    objects_list[4] = detectedObject[0]
    objects_list[5] = detectedObject[1]

    

def callback_sensor6(data):
    global sensor6_distance
    sensor6_distance = data.range


def callback_sensor7(data):
    global sensor7_distance
    sensor7_distance = data.range
    detectedObject = [0, 0]
    if is_distance_valid(sensor6_distance) and is_distance_valid(sensor7_distance):
        detectedObject = normalizeObject(check_positions_left(calculateObject(s6, sensor6_distance, s7, sensor7_distance)))
    elif sensor6_distance is not None and sensor6_distance < 0.2 and sensor7_distance is not None and sensor7_distance < 0.2:
        detectedObject = [-106, -107]
    elif sensor6_distance is not None and sensor6_distance < 0.2:
        detectedObject = [-106, -106]
    elif sensor7_distance is not None and sensor7_distance < 0.2:
        detectedObject = [-107, -107]
    elif sensor6_distance == float('inf') and sensor7_distance == float('inf'):
        detectedObject = [106, 107]
    elif sensor6_distance == float('inf'):
        detectedObject = [106, 106]
    elif sensor7_distance == float('inf'):
        detectedObject = [107, 107]


    objects_list[6] = detectedObject[0]
    objects_list[7] = detectedObject[1]


# create Subscriber for sensor 0 to 7
def listener():

    rospy.Subscriber("usboard_v2/sensor0", Range, callback_sensor5) #richtig ist: 0
    rospy.Subscriber("usboard_v2/sensor1", Range, callback_sensor6) #richtig ist: 1
    rospy.Subscriber("usboard_v2/sensor2", Range, callback_sensor7) #richtig ist: 2
    rospy.Subscriber("usboard_v2/sensor3", Range, callback_sensor3)
    rospy.Subscriber("usboard_v2/sensor4", Range, callback_sensor4)
    rospy.Subscriber("usboard_v2/sensor5", Range, callback_sensor5)
    rospy.Subscriber("usboard_v2/sensor6", Range, callback_sensor6)
    rospy.Subscriber("usboard_v2/sensor7", Range, callback_sensor7)
    
"""
create Publisher, which publish's in a Float64MultiArray with the Format:
(SP1x, SP1y, SP2x, SP2y, SP3x, SP3y, SP4x, SP4y) SP = Sensor-Pair
"""

def publisher():
    pub = rospy.Publisher('positions', Float64MultiArray, queue_size=4)
    #rospy.init_node('publisher_positions', anonymous=True)
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        if objects_list:
            positions = Float64MultiArray()
            positions.data = objects_list
            rospy.loginfo("Coordinates to be published: %s" % str(objects_list))
            pub.publish(positions)

        rate.sleep()
    


if __name__ == '__main__':
    listener()

    rospy.init_node('positions_ultrasonic', anonymous=True)

    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()