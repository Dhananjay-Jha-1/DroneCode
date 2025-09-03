#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import pyproj
import math
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import transform
from shapely.affinity import rotate
from pymavlink import mavutil
import time

# --- 1. Parameters ---
# Your KML file path
kml_file = "/home/dhananjay/KML/newKml.kml"

# Drone and camera parameters
altitude_ft = 110          # Drone altitude
fov_deg = 62               # Raspberry Pi Cam v2 horizontal FOV
overlap_m = 20             # Desired overlap between passes in meters

# SITL connection details
sitl_connection_string = 'udp:127.0.0.1:14550'
mission_altitude_m = 33.53 # Same as altitude_ft converted to meters

# --- 2. KML Parsing and Waypoint Generation (The Logic We Built) ---

def generate_waypoints(kml_path):
    """Parses KML, generates lawn-mower grid, and returns GPS waypoints."""
    # 2.1 Parse KML
    tree = ET.parse(kml_path)
    root = tree.getroot()
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}

    polygon_elem = root.find(".//kml:Polygon", ns)
    if polygon_elem is None:
        raise ValueError("❌ No polygon found in KML")

    coords_text = polygon_elem.find(".//kml:coordinates", ns).text.strip()
    coords = []
    for line in coords_text.split():
        lon, lat, alt = map(float, line.split(','))
        coords.append((lon, lat))
    polygon = Polygon(coords)

    # 2.2 Project to UTM
    centroid = polygon.centroid
    lon_c, lat_c = centroid.x, centroid.y
    utm_zone = int((lon_c + 180) // 6) + 1
    epsg_code = 32600 + utm_zone if lat_c >= 0 else 32700 + utm_zone
    utm_crs = pyproj.CRS.from_epsg(epsg_code)
    project = pyproj.Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True).transform
    polygon_utm = transform(project, polygon)

    # 2.3 Calculate swath and spacing
    altitude_m = altitude_ft * 0.3048
    swath_width = 2 * altitude_m * math.tan(math.radians(fov_deg) / 2)
    grid_spacing = swath_width - overlap_m

    # 2.4 Generate lawn-mower grid
    minx, miny, maxx, maxy = polygon_utm.bounds
    width = maxx - minx
    height = maxy - miny
    angle = 0
    if height > width:
        angle = 90
    polygon_rot = rotate(polygon_utm, -angle, origin='centroid', use_radians=False)

    minx, miny, maxx, maxy = polygon_rot.bounds
    lines = []
    y = miny
    while y <= maxy:
        lines.append(LineString([(minx, y), (maxx, y)]))
        y += grid_spacing

    waypoints_utm = []
    for i, line in enumerate(lines):
        intersection = polygon_rot.intersection(line)
        if intersection.is_empty:
            continue
        coords_line = []
        if intersection.geom_type == 'LineString':
            coords_line = list(intersection.coords)
        elif intersection.geom_type == 'MultiLineString':
            for seg in intersection:
                coords_line.extend(list(seg.coords))
        if not coords_line:
            continue
        if i % 2 == 1:
            coords_line.reverse()
        waypoints_utm.extend(coords_line)

    waypoints_final = [rotate(Point(p[0], p[1]), angle, origin=polygon_utm.centroid, use_radians=False) for p in waypoints_utm]
    
    # 2.5 Convert back to GPS
    inverse_project = pyproj.Transformer.from_crs(utm_crs, "EPSG:4326", always_xy=True).transform
    waypoints_gps = []
    for wp in waypoints_final:
        lon, lat = inverse_project(wp.x, wp.y)
        waypoints_gps.append((lat, lon))
        
    print(f"✅ Generated {len(waypoints_gps)} waypoints")
    return waypoints_gps

# --- 3. SITL Connection and Mission Upload ---

def upload_and_start_mission(master, waypoints, altitude):
    """Uploads waypoints and starts the mission."""
    print("Clearing existing mission...")
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    master.waypoint_clear_all_send()
    
    # Give the SITL a moment to process the clear command
    time.sleep(1)

    print(f"Uploading {len(waypoints)} waypoints...")
    for i, (lat, lon) in enumerate(waypoints):
        master.mav.mission_item_send(
            master.target_system, master.target_component,
            i,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0, 0, 
            lat, lon, altitude
        )
    
    # Set the first waypoint as the current mission
    master.mav.mission_set_current_send(master.target_system, master.target_component, 0)
    print("✅ Mission uploaded!")
    
    # Give the SITL time to receive the mission
    time.sleep(2)
    
    print("Arming drone and setting to AUTO mode...")
    # Change mode to AUTO
    master.set_mode('AUTO')
    
    # Arm the drone
    master.arducopter_arm()
    
    # Wait for arming to complete and mission to start
    print("Waiting for drone to start mission...")
    time.sleep(5)
    print("🚀 Mission started!")

# --- Main Script Execution ---

if __name__ == "__main__":
    # Generate waypoints using the combined logic
    all_waypoints_gps = generate_waypoints(kml_file)
    
    print("\n--- Connecting to ArduPilot SITL ---")
    
    # Ensure SITL is running in a separate terminal before running this script
    # sim_vehicle.py -v ArduCopter --map --console
    master = mavutil.mavlink_connection(sitl_connection_string)
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))
    
    # Upload waypoints and start the mission
    upload_and_start_mission(master, all_waypoints_gps, mission_altitude_m)