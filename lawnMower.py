import xml.etree.ElementTree as ET
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import transform
from shapely.affinity import rotate
import pyproj
import math

# -----------------------------
# Parameters
# -----------------------------
kml_file = "/home/dhananjay/KML/newKml.kml"
altitude_ft = 110          # Drone altitude
fov_deg = 62              # Raspberry Pi Cam v2 horizontal FOV
overlap_m = 20            # Desired overlap between passes in meters

# -----------------------------
# 1. Parse KML
# -----------------------------
tree = ET.parse(kml_file)
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
print("✅ Polygon loaded")
print(f"Number of vertices: {len(polygon.exterior.coords)}")
print(f"First few coordinates: {list(polygon.exterior.coords)[:3]}")

# -----------------------------
# 2. Project to UTM
# -----------------------------
centroid = polygon.centroid
lon_c, lat_c = centroid.x, centroid.y
utm_zone = int((lon_c + 180) // 6) + 1
epsg_code = 32600 + utm_zone if lat_c >= 0 else 32700 + utm_zone
utm_crs = pyproj.CRS.from_epsg(epsg_code)
project = pyproj.Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True).transform
polygon_utm = transform(project, polygon)

area_m2 = polygon_utm.area
area_ha = area_m2 / 10000.0

print(f"UTM zone: {utm_zone}, EPSG: {epsg_code}")
print(f"Polygon area: {area_m2:.2f} m² ≈ {area_ha:.2f} hectares")

# -----------------------------
# 3. Camera swath and grid spacing
# -----------------------------
altitude_m = altitude_ft * 0.3048
swath_width = 2 * altitude_m * math.tan(math.radians(fov_deg)/2)
grid_spacing = swath_width - overlap_m

print(f"Drone altitude: {altitude_m:.2f} m")
print(f"Camera swath width: {swath_width:.2f} m")
print(f"Grid spacing (with {overlap_m} m overlap): {grid_spacing:.2f} m")

# -----------------------------
# 4. Generate lawn-mower scan grid
# -----------------------------
minx, miny, maxx, maxy = polygon_utm.bounds
width = maxx - minx
height = maxy - miny
angle = 0
if height > width:
    angle = 90  # rotate polygon for optimal grid orientation

polygon_rot = rotate(polygon_utm, -angle, origin='centroid', use_radians=False)

minx, miny, maxx, maxy = polygon_rot.bounds
lines = []
y = miny
while y <= maxy:
    lines.append(LineString([(minx, y), (maxx, y)]))
    y += grid_spacing

waypoints = []
for i, line in enumerate(lines):
    intersection = polygon_rot.intersection(line)
    if intersection.is_empty:
        continue
    coords_line = []

    # Handle different geometry types safely
    if intersection.geom_type == 'LineString':
        coords_line = list(intersection.coords)
    elif intersection.geom_type == 'MultiLineString':
        for seg in intersection:
            coords_line.extend(list(seg.coords))
    # Ignore if nothing to add
    if not coords_line:
        continue

    # Reverse every other line for lawn-mower path
    if i % 2 == 1:
        coords_line.reverse()

    waypoints.extend(coords_line)

# Rotate back to original orientation
waypoints_final = [rotate(Point(p[0], p[1]), angle, origin=polygon_utm.centroid, use_radians=False) for p in waypoints]

print(f"✅ Generated {len(waypoints_final)} waypoints")
print("First few waypoints (x, y in meters):")
for wp in waypoints_final[:5]:
    print((wp.x, wp.y))


# -----------------------------
# 5. Convert waypoints back to GPS
# -----------------------------
inverse_project = pyproj.Transformer.from_crs(utm_crs, "EPSG:4326", always_xy=True).transform
waypoints_gps = []

for wp in waypoints_final:
    lon, lat = inverse_project(wp.x, wp.y)
    waypoints_gps.append((lat, lon))

print(f"✅ Converted {len(waypoints_gps)} waypoints to GPS coordinates")
print("First few GPS waypoints (lat, lon):")
for wp in waypoints_gps[:5]:
    print(wp)

#saving the waypoints of the scan drone into a CSV file

import csv

csv_file = "/home/dhananjay/coding/droneCode/waypoints.csv"

with open(csv_file, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["lat", "lon"])  # header
    for wp in waypoints_gps:
        writer.writerow([wp[0], wp[1]])

print(f"✅ Saved {len(waypoints_gps)} GPS waypoints to {csv_file}")
