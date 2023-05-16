#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

# Callback function to receive GPS data for drone 1
def drone1_gps_callback(data):
    # Extract GPS data for drone 1 (latitude, longitude, altitude)
    drone1_latitude = data.latitude/ 10000000.0
    drone1_longitude = data.longitude/ 10000000.0
    drone1_altitude = data.altitude

# Callback function to receive GPS data for drone 2
def drone2_gps_callback(data):
    # Extract GPS data for drone 2 (latitude, longitude, altitude)
    drone2_latitude = data.latitude/ 10000000.0
    drone2_longitude = data.longitude/ 10000000.0
    drone2_altitude = data.altitude

# Function to calculate distance between two GPS coordinates
def calculate_distance(lat1, lon1, lat2, lon2):
    # Calculate the distance between the two drones using the Haversine formula
    R = 6371.0  # radius of the earth in km
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R * c * 1000 * 100
    return distance

# ROS initialization
rospy.init_node('distance_calculation_node')
#rospy.init_node('gps_subscriber_node')

# Subscribe to the GPS topics of both drones
rospy.Subscriber('drone1/mavros/global_position/global', NavSatFix, drone1_gps_callback)
rospy.Subscriber('drone2/mavros/global_position/global', NavSatFix, drone2_gps_callback)

# Main loop
while not rospy.is_shutdown():
    # Calculate distance between the two drones using the received GPS coordinates
    distance = calculate_distance(drone1_latitude, drone1_longitude, drone2_latitude, drone2_longitude)

    # Display the coordinates of the drones
    rospy.loginfo("Position drone 1: (" + str(drone1_latitude) + "," + str(drone1_longitude) + ")\n")
    rospy.loginfo("Position drone 2: (" + str(drone2_latitude) + "," + str(drone2_longitude) + ")\n")
    # Display or publish the distance as needed
    rospy.loginfo("Distance between drones: %.2f cm" % distance)

    # Sleep to control the loop rate
    rospy.sleep(1.0)
