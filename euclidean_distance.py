from docx import Document
import math

def read_table_from_docx(docx_path):
    doc = Document(docx_path)
    table = doc.tables[0]  # Assuming the first table contains the data
    data = []
    for row in table.rows[1:]:  # Skip the header row
        frame, calculated, ground_truth = (cell.text for cell in row.cells)
        # Parse the string to extract numerical values
        calculated_coords = tuple(map(float, calculated.strip('()').split(',')))
        ground_truth_coords = tuple(map(float, ground_truth.strip('()').split(',')))
        data.append((frame, calculated_coords, ground_truth_coords))
    return data

def calculate_distances(data):
    distances = []
    for _, calc_coords, gt_coords in data:
        distance = math.sqrt((gt_coords[0] - calc_coords[0])**2 + (gt_coords[1] - calc_coords[1])**2)
        distances.append(distance)
    return distances

# Path to your .docx file
docx_path = 'localization_results.docx'

# Read the data from the Word document
data = read_table_from_docx(docx_path)

# Calculate the Euclidean distances
distances = calculate_distances(data)

# Calculate summary statistics
mean_distance = sum(distances) / len(distances)
max_distance = max(distances)
min_distance = min(distances)
median_distance = sorted(distances)[len(distances) // 2]

# Output results
print(f"Mean Euclidean distance: {mean_distance}")
print(f"Max Euclidean distance: {max_distance}")
print(f"Min Euclidean distance: {min_distance}")
print(f"Median Euclidean distance: {median_distance}")