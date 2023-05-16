import time
from pymavlink import mavutil

#Port problem -> cannot connect but we change to MAVROS

# Set the connection parameters (choose your port and baud rate)
connection = mavutil.mavlink_connection('udp:192.168.1.11:14575', baud=921600)

# Wait for the heartbeat message to find the system ID
connection.wait_heartbeat()

# Get the system ID of this drone
my_sys_id = connection.target_system

# Set the target system ID to the other drone
target_sys_id = 2  # change this to the system ID of the other drone

# Wait for GPS lock
print("Waiting for GPS lock...")
while True:
    msg = connection.recv_match(type='GPS_RAW_INT', blocking=True)
    if msg.fix_type > 1:
        print("GPS lock acquired")
        break

# Set the MAVLink mode to AUTO
connection.mav.set_mode_send(
    connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)

# Arm the drone
connection.arducopter_arm()

# Wait for the arming to complete
while True:
    msg = connection.recv_match(type='HEARTBEAT', blocking=True)
    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("Drone armed")
        break

# Get the GPS coordinates of this drone
lat1 = connection.messages['GLOBAL_POSITION_INT'].lat / 10000000.0
lon1 = connection.messages['GLOBAL_POSITION_INT'].lon / 10000000.0

# Wait for the other drone to arm and take off
print("Waiting for other drone to take off...")
while True:
    msg = connection.recv_match(type='HEARTBEAT', blocking=True)
    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        break
while True:
    msg = connection.recv_match(type='VFR_HUD', blocking=True)
    if msg.alt > 1.0:
        print("Other drone in air")
        break

# Get the GPS coordinates of the other drone
lat2 = connection.messages['GLOBAL_POSITION_INT'].lat / 10000000.0
lon2 = connection.messages['GLOBAL_POSITION_INT'].lon / 10000000.0

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

print("Distance between drones: {:.2f} cm".format(distance))
