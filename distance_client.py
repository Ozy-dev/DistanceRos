#!/usr/bin/env python

import rospy
from beginner_tutorials.srv import MeasureDistance

def measure_distance_client():
    rospy.wait_for_service('measure_distance')
    try:
        # Buat proxy untuk memanggil service
        measure_distance = rospy.ServiceProxy('measure_distance', MeasureDistance)

        # Panggil service dan dapatkan hasil
        response = measure_distance()
        if response.distance == -1.0:
            rospy.loginfo("Failed to measure distance.")
        else:
            rospy.loginfo(f"Measured Distance: {response.distance} cm")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('measure_distance_client')  # Inisialisasi node
    measure_distance_client()
