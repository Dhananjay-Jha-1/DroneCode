from pymavlink import mavutil
import time
import csv

# -----------------------------
# 1. Connect to SITL
# -----------------------------
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print(f"✅ Heartbeat received (system {master.target_system} component {master.target_component})")

# -----------------------------
# 2. Read CSV waypoints
# -----------------------------
csv_file = "/home/dhananjay/coding/droneCode/waypoints.csv"
waypoints = []

with open(csv_file, newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        lat = float(row['lat'])
        lon = float(row['lon'])
        alt = 30  # Altitude in metersa
        waypoints.append((lat, lon, alt))

print(f"✅ Loaded {len(waypoints)} waypoints")

# -----------------------------
# 3. Clear old mission
# -----------------------------
master.waypoint_clear_all_send()
time.sleep(1)

# -----------------------------
# 4. Send mission count
# -----------------------------
mission_count = len(waypoints) + 1  # +1 for TAKEOFF
master.mav.mission_count_send(master.target_system,
                              master.target_component,
                              mission_count)

# -----------------------------
# 5. Upload mission items
# -----------------------------

seq = 0

# First item: Takeoff
master.mav.mission_item_send(
    master.target_system,
    master.target_component,
    seq,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0,
    waypoints[0][0], waypoints[0][1], waypoints[0][2]
)
seq += 1

# Rest: Waypoints
for lat, lon, alt in waypoints:
    master.mav.mission_item_send(
        master.target_system,
        master.target_component,
        seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0, 0, 0, 0, 0,
        lat, lon, alt
    )
    seq += 1

print("✅ Mission uploaded")

# -----------------------------
# 6. Start mission
# -----------------------------
master.mav.mission_set_current_send(
    master.target_system,
    master.target_component,
    0
)
time.sleep(1)

# Arm
master.arducopter_arm()
print("✅ Drone armed")
time.sleep(2)

# Switch to AUTO
mode = 'AUTO'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)
print("✅ AUTO mode set")

print("🚀 Drone should now take off and follow mission!")
