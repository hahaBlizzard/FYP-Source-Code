import math
import numpy as np

class Point:
    def __init__(self, coordinates, codename,tag):
        self.coordinates = coordinates
        self.codename = codename
        self.tag = tag
        self.linked_points = []

    def add_linked_point(self, other_point):
        self.linked_points.append(other_point)

    def distance_to(self, other_point):
        return math.sqrt((self.coordinates[0] - other_point.coordinates[0]) ** 2 +
                         (self.coordinates[1] - other_point.coordinates[1]) ** 2)

# Assuming 'document_content' is a string containing the contents of the document.
# Replace the '...' with the actual content from 'landmark_result_extract.txt'.
document_content = """
(-38.904,-4.48), Codename: 1000, tag: 15
(-28.904,12.84), Codename: 0111, tag: 16
(-18.904,-4.48), Codename: 1011, tag: 17
(-8.904,12.84), Codename: 1010, tag: 18
(1.096,-4.48), Codename: 1101, tag: 19
(10.096,12.84), Codename: 0010, tag: 20
(20.096,-4.48), Codename: 0101, tag: 21
(30.096,12.84), Codename: 0100, tag: 22
(40.096,-4.48), Codename: 1000, tag: 23
"""

# Split the document by lines and parse each line
points = []
for line in document_content.strip().split('\n'):
    if line:  # Make sure the line is not empty
        # Extracting coordinates, codename, and tag from the line
        coords_part, codename_part, tag_part = line.split(', ')
        coordinates = tuple(map(float, coords_part.strip('()').split(',')))
        codename = codename_part.split(': ')[1]
        tag = int(tag_part.split(': ')[1]) # Assuming tag is always an integer
        
        # Append the new Point with coordinates, codename, and tag to the points list
        points.append(Point(coordinates, codename, tag))

# Calculate distances and link points
for i, base_point in enumerate(points):
    for j, other_point in enumerate(points):
        if i != j:  # Do not calculate distance to itself
            distance = base_point.distance_to(other_point)
            
            if distance < 21:
                
                base_point.add_linked_point(other_point)

# Print codenames and linked points
'''for point in points:
    linked_tags = [linked_point.tag for linked_point in point.linked_points]
    print(f"Codename: {point.tag}, Linked Points: {linked_tags}")'''