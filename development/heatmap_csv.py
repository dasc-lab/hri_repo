import csv
import numpy as np

def read_csv(file_path):
    column1 = []
    column2 = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        next(csv_reader)  # Skip header if present
        for row in csv_reader:
            if len(row) >= 2:  # Ensure at least two columns in the row
                column1.append(int(float(row[0])))
                column2.append(float(row[1]))
    return column1, column2

# Example usage
file_path = '/HRI/video_streaming/world_discrepancy.csv'  # Replace 'data.csv' with your CSV file path
#file_path = '/Users/albusfang/Coding\ Projects/VS\ Code\ Projects/HRI/video_streaming/world_discrepancy.csv'
file_path = 'world_discrepancy.csv'
pixel_distances, error = read_csv(file_path)
heatmap = np.zeros((480,640))

def find_elements_same_distance(matrix):
    center_x = matrix.shape[0] // 2
    center_y = matrix.shape[1] // 2
    center = (center_x, center_y)

    distances = {}  # Dictionary to store distances as keys and elements as values

    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            distance = np.sqrt((i - center[0])**2 + (j - center[1])**2)
            if distance not in distances:
                distances[distance] = []
            distances[distance].append((i, j))

    return distances

distances = find_elements_same_distance(heatmap)
print(len(distances))
print(len(error))


# Zip both arrays together
zipped_arrays = zip(pixel_distances, error)

# Sort based on the first array
sorted_zipped_arrays = sorted(zipped_arrays)

# Extract sorted arrays

def combine_and_average(sorted_pairs):
    combined_pairs = []  # Combined pairs with averaged second values
    current_first = None  # Current first value
    sum_second = 0  # Sum of second values for current first value
    count_second = 0  # Count of occurrences for current first value

    for first, second in sorted_pairs:
        if first != current_first:
            if current_first is not None:
                # Append combined pair with averaged second value
                combined_pairs.append((current_first, sum_second / count_second))
            current_first = first
            sum_second = 0
            count_second = 0
        sum_second += second
        count_second += 1

    # Append combined pair for the last group of second values
    if current_first is not None:
        combined_pairs.append((current_first, sum_second / count_second))

    return combined_pairs

# Example usage:

result = combine_and_average(sorted_zipped_arrays)
sorted_pixel_distances = [pair[0] for pair in sorted_zipped_arrays]
sorted_errors = [pair[1] for pair in sorted_zipped_arrays]
for pair, in result.item():
    