#!/usr/bin/env python

import rospy
from beginner_tutorials.srv import CalculateIncentive

def calculate_incentive_client(diameter):
    rospy.wait_for_service('calculate_incentive')
    try:
        calculate_incentive = rospy.ServiceProxy('calculate_incentive', CalculateIncentive)
        response = calculate_incentive(diameter)
        return response.area, response.incentive
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('circle_client')

    # Daftar penduduk dengan diameter tanah masing-masing
    diameters = [200.0] * 12  # Semua diameter sama: 200 m

    for i, diameter in enumerate(diameters, start=1):
        area, incentive = calculate_incentive_client(diameter)
        rospy.loginfo(f"Penduduk {i}: Diameter = {diameter} m, Area = {area:.2f} m², Incentive = Rp {incentive:.2f}")
