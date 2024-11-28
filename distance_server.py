#!/usr/bin/env python

import rospy
from beginner_tutorials.srv import MeasureDistance, MeasureDistanceResponse
import serial  # Untuk membaca data dari sensor

def handle_measure_distance(req):
    try:
        # Hubungkan ke sensor MaxBotix melalui USB
        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Sesuaikan port USB
        raw_data = ser.readline().decode('utf-8').strip()  # Baca data dari sensor
        ser.close()

        # Konversi data menjadi float (jarak dalam cm)
        distance = float(raw_data)

        # Tampilkan jarak yang terdeteksi
        rospy.loginfo(f"Measured Distance: {distance} cm")

        # Kirim jarak sebagai respons
        return MeasureDistanceResponse(distance)

    except Exception as e:
        rospy.logerr(f"Error reading sensor: {e}")
        return MeasureDistanceResponse(-1.0)  # -1 sebagai indikator error

def measure_distance_server():
    rospy.init_node('measure_distance_server')  # Inisialisasi node
    service = rospy.Service('measure_distance', MeasureDistance, handle_measure_distance)
    rospy.loginfo("Measure Distance Server is ready.")
    rospy.spin()

if __name__ == "__main__":
    measure_distance_server()
