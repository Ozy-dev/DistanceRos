#!/usr/bin/env python

import rospy
from math import pi
from beginner_tutorials.srv import CalculateIncentive, CalculateIncentiveResponse

def handle_calculate_incentive(req):
    # Hitung jari-jari dan luas lingkaran
    radius = req.diameter / 2.0
    area = pi * (radius ** 2)  # Luas lingkaran

    # Hitung total insentif
    incentive_per_m2 = 150000  # Rp per m²
    total_incentive = area * incentive_per_m2

    rospy.loginfo(f"Diameter: {req.diameter} m, Area: {area:.2f} m², Incentive: Rp {total_incentive:.2f}")
    return CalculateIncentiveResponse(area, total_incentive)

def calculate_incentive_server():
    rospy.init_node('circle_server')
    service = rospy.Service('calculate_incentive', CalculateIncentive, handle_calculate_incentive)
    rospy.loginfo("Ready to calculate area and incentive.")
    rospy.spin()

if __name__ == "__main__":
    calculate_incentive_server()
