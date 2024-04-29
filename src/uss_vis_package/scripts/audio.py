#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import pygame

# Funktion zum Erzeugen eines Audio-Signals mit variabler Frequenz basierend auf dem Abstand zum Ursprung
def generate_audio(data):
    # Extrahiere x- und y-Koordinaten aus den empfangenen Daten
    x = data.data[0]
    y = data.data[1]

    # Berechne den Abstand zum Ursprung (0, 0) mit dem Satz des Pythagoras
    distance = np.sqrt(x**2 + y**2)

    # Definiere den minimalen und maximalen Abstand und die entsprechenden minimalen und maximalen Frequenzen
    min_distance = 0.0
    max_distance = 8.0
    min_frequency = 1000.0  # Minimale Frequenz in Hz
    max_frequency = 4000.0  # Maximale Frequenz in Hz

    # Berechne die Frequenz basierend auf dem Abstand zum Ursprung
    frequency = min_frequency + (max_frequency - min_frequency) * ((distance - min_distance) / (max_distance - min_distance))

    # Erzeuge das Audio-Signal mit der berechneten Frequenz
    pygame.mixer.init()

    # Erzeuge einen Sinus-Ton mit der berechneten Frequenz
    sample_rate = 44100
    sound_data = np.sin(2 * np.pi * frequency * np.arange(sample_rate) / float(sample_rate)).astype(np.float32)

    # Spiele den Ton ab
    pygame.mixer.Sound(sound_data).play()

# ROS-Node-Initialisierung
rospy.init_node('audio_generator', anonymous=True)

# Pygame initialisieren
pygame.init()

# Abonniere das Topic "points", um Daten zu empfangen
rospy.Subscriber("points", Float64MultiArray, generate_audio)

# Lasse das Programm laufen, bis es beendet wird
rospy.spin()
