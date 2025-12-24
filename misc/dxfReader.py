import sys
import ezdxf
import json
import math

# Python dictionary
data = []

try:
    doc = ezdxf.readfile("field.dxf")
    print(f"DXF file '{doc.filename}' loaded successfully.")
    for entity in doc.modelspace():
        if entity.dxftype() == 'LINE':
            data.append({
                'type': entity.dxftype(),
                'startX': entity.dxf.start.x,
                'startY': entity.dxf.start.y,
                'endX': entity.dxf.end.x,
                'endY': entity.dxf.end.y
            })
        elif entity.dxftype() == 'CIRCLE':
            data.append({
                'type': entity.dxftype(),
                'centerX': entity.dxf.center.x,
                'centerY': entity.dxf.center.y,
                'radius': entity.dxf.radius
            })
        elif entity.dxftype() == 'ARC':
            data.append({
                'type': entity.dxftype(),
                'centerX': entity.dxf.center.x,
                'centerY': entity.dxf.center.y,
                'radius': entity.dxf.radius,
                'startAngle': entity.dxf.start_angle,
                'endAngle': entity.dxf.end_angle
            })
except IOError:
    print(f"Not a DXF file or a generic I/O error.")
    sys.exit(1)
except ezdxf.DXFStructureError:
    print(f"Invalid or corrupted DXF file.")
    sys.exit(2)

# Writing JSON to a file
with open('dxfFile.json', 'w') as file:
    json.dump(data, file, indent=4)  # Converts Python dictionary to JSON

print("({")
for i in data:
    if (i['type'] == 'LINE'):
        print(f"limelib::LineSegment2D({i['startX']}, {i['startY']}, {i['endX']}, {i['endY']}), ", end = "")
    elif (i['type'] == 'CIRCLE'):
        # Approximate circle with a polygon
        num_segments = 1
        for j in range(num_segments):
            angle1 = 2 * math.pi * j / num_segments
            angle2 = 2 * math.pi * (j + 1) / num_segments
            startX = i['centerX'] + i['radius'] * math.cos(angle1)
            startY = i['centerY'] + i['radius'] * math.sin(angle1)
            endX = i['centerX'] + i['radius'] * math.cos(angle2)
            endY = i['centerY'] + i['radius'] * math.sin(angle2)
            print(f"limelib::LineSegment2D({startX}, {startY}, {endX}, {endY}), ", end = "")
    elif (i['type'] == 'ARC'):
        # Approximate arc with line segments
        num_segments = 1
        start_angle_rad = math.radians(i['startAngle'])
        end_angle_rad = math.radians(i['endAngle'])
        angle_range = end_angle_rad - start_angle_rad
        for j in range(num_segments):
            angle1 = start_angle_rad + angle_range * j / num_segments
            angle2 = start_angle_rad + angle_range * (j + 1) / num_segments
            startX = i['centerX'] + i['radius'] * math.cos(angle1)
            startY = i['centerY'] + i['radius'] * math.sin(angle1)
            endX = i['centerX'] + i['radius'] * math.cos(angle2)
            endY = i['centerY'] + i['radius'] * math.sin(angle2)
            print(f"limelib::LineSegment2D({startX}, {startY}, {endX}, {endY}), ", end = "")
print("});")
