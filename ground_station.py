from pymavlink import mavutil
import time

# --- Connection Parameters ---
# Use the correct 'udp' protocol
SCAN_DRONE_PORT = 'udp:127.0.0.1:14550'
DELIVERY_DRONE_PORT = 'udp:127.0.0.1:14560'
ALTITUDE = 30 # meters

# --- Mission Parameters ---
SURVIVOR_LAT = 13.13885
SURVIVOR_LON = 77.61275

# --- Main Script ---

print("Waiting for SITL instances to start...")
time.sleep(5) 

# 1. Connect to both drones
print("Connecting to scan drone...")
# Wait for the first heartbeat on the UDP connection
# The `mavutil.mavlink_connection` will handle the UDP "connection"
scan_master = mavutil.mavlink_connection(SCAN_DRONE_PORT)
scan_master.wait_heartbeat()
print("Scan drone connected!")

print("Connecting to delivery drone...")
delivery_master = mavutil.mavlink_connection(DELIVERY_DRONE_PORT)
delivery_master.wait_heartbeat()
print("Delivery drone connected!")

# 2. Command the scan drone to fly its mission
print("Arming scan drone...")
scan_master.arducopter_arm()
scan_master.set_mode('AUTO')
print("Scan drone launched!")

# 3. Listen for scan drone position and simulate survivor detection
print("Listening for scan drone position to find survivors...")
survivor_found = False
while not survivor_found:
    msg = scan_master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    scan_lat = msg.lat * 1.0e-7
    scan_lon = msg.lon * 1.0e-7
    
    if abs(scan_lat - SURVIVOR_LAT) < 0.0001 and abs(scan_lon - SURVIVOR_LON) < 0.0001:
        print(" Survivor detected! Geotagging location...")
        survivor_location = (scan_lat, scan_lon, ALTITUDE)
        survivor_found = True
    time.sleep(1)

# 4. Command the delivery drone
print("Sending mission to delivery drone...")
delivery_master.mav.mission_clear_all_send(delivery_master.target_system, delivery_master.target_component)
time.sleep(1)
delivery_master.mav.mission_item_send(
    delivery_master.target_system, delivery_master.target_component,
    0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0, 0, 0, 0, 0, 0,
    survivor_location[0], survivor_location[1], survivor_location[2]
)
print("Mission sent to delivery drone.")
time.sleep(1)
delivery_master.arducopter_arm()
delivery_master.set_mode('AUTO')
print("Delivery drone launched!")
