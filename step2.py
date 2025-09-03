import xml.etree.ElementTree as ET
from shapely.geometry import Polygon
from shapely.ops import transform
import pyproj
import math

# -----------------------------
# Parameters
# -----------------------------
kml_file = "/home/dhananjay/KML/newKml.kml"
altitude_ft = 70          # Drone altitude
fov_deg = 62              # Raspberry Pi Cam v2 horizontal FOV
overlap_m = 20            # Desired overlap between passes in meters

# -----------------------------
# Step 2a: Parse KML
# -----------------------------
tree = ET.parse(kml_file)
root = tree.getroot()
ns = {'kml': 'http://www.opengis.net/kml/2.2'}

# Extract first polygon
polygon_elem = root.find(".//kml:Polygon", ns)
if polygon_elem is None:
    raise ValueError("No polygon found in KML")

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
# Step 2b: Project to UTM (meters)
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
# Step 2c: Camera swath and grid spacing
# -----------------------------
altitude_m = altitude_ft * 0.3048
swath_width = 2 * altitude_m * math.tan(math.radians(fov_deg)/2)
grid_spacing = swath_width - overlap_m

print(f"Drone altitude: {altitude_m:.2f} m")
print(f"Camera swath width: {swath_width:.2f} m")
print(f"Grid spacing (with {overlap_m} m overlap): {grid_spacing:.2f} m")
