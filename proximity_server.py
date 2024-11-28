#!/usr/bin/env python

from __future__ import print_function
import rospy
from beginner_tutorials.srv import ProxBoolean, ProxBooleanResponse
import serial  # Untuk komunikasi serial

# Konfigurasi port serial
SERIAL_PORT = '/dev/ttyUSB0'  # Sesuaikan dengan port sensor Anda
BAUD_RATE = 9600

def handle_proximity_request(req):
    try:
        # Membuka koneksi serial
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            ser.flushInput()

            # Membaca data dari sensor
            raw_data = ser.readline().decode('utf-8').strip()

            # Validasi data (pastikan data adalah angka)
            if raw_data.replace('.', '', 1).isdigit():
                distance = float(raw_data)  # Mengonversi data menjadi float
                rospy.loginfo(f"Distance received: {distance} cm")
                return ProxBooleanResponse(distance)
            else:
                rospy.logwarn("Received invalid data from sensor.")
                return ProxBooleanResponse(-1.0)  # Mengembalikan -1 jika data invalid
    except serial.SerialException as e:
        rospy.logerr(f"Serial error: {e}")
        return ProxBooleanResponse(-1.0)  # Mengembalikan -1 jika terjadi error

def proximity_server():
    rospy.init_node('proximity_server')  # Inisialisasi node ROS
    rospy.Service('proximity_sensor', ProxBoolean, handle_proximity_request)  # Menyiapkan service
    rospy.loginfo("Proximity Sensor Server is ready.")  # Log info bahwa server siap
    rospy.spin()  # Menunggu request dari client

if __name__ == "__main__":
    proximity_server()
