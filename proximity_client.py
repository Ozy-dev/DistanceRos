#!/usr/bin/env python

import rospy
from beginner_tutorials.srv import ProxBoolean

def proximity_client():
    rospy.wait_for_service('proximity_sensor')  # Menunggu hingga service tersedia
    try:
        # Membuat proxy untuk service
        get_distance = rospy.ServiceProxy('proximity_sensor', ProxBoolean)

        # Panggil service dan dapatkan jarak
        response = get_distance()
        if response.distance == -1.0:
            rospy.loginfo("Failed to get distance.")  # Jika ada error
        else:
            rospy.loginfo(f"Measured Distance: {response.distance} cm")  # Menampilkan jarak

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")  # Menangani kesalahan

if __name__ == "__main__":
    rospy.init_node('proximity_client')  # Inisialisasi node client
    proximity_client()

