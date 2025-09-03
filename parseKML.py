import xml.etree.ElementTree as ET

kml_file = "/home/dhananjay/KML/newKml.kml"

# Parse XML
tree = ET.parse(kml_file)
root = tree.getroot()

# KML namespace
ns = {'kml': 'http://www.opengis.net/kml/2.2'}

# Find all polygons
polygons = root.findall(".//kml:Polygon", ns)

if not polygons:
    raise ValueError("❌ No polygon found in KML")

# Extract coordinates from the first polygon
coords_text = polygons[0].find(".//kml:coordinates", ns).text.strip()
coords = []
for line in coords_text.split():
    lon, lat, alt = map(float, line.split(','))
    coords.append((lon, lat, alt))

print("✅ Polygon loaded from KML")
print(f"Number of vertices: {len(coords)}")
print(f"First few coordinates: {coords[:3]}")
