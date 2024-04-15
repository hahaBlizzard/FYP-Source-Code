def read_location_data_truth(file_path):
    # Initialize an empty dictionary to store the data
    location_data_truth = {}

    # Open the file and read the lines
    with open(file_path, 'r') as file:
        # Read the lines of the file
        lines = file.readlines()

        # Skip the header line
        header = lines[0].strip().split(',')
        frame_index = header.index('frame')
        loc_x_index = header.index('loc_x')
        loc_y_index = header.index('loc_y')

        # Process each line after the header
        for line in lines[1:]:
            # Split the line by comma and strip to remove any leading/trailing whitespace
            parts = line.strip().split(',')
            # Map the frame to loc_x and loc_y in the dictionary
            frame = int(parts[frame_index])
            loc_x = float(parts[loc_x_index])
            loc_y = float(parts[loc_y_index])
            location_data_truth[frame] = (loc_x, loc_y)

    return location_data_truth

# Specify the path to the text file within the Data folder
file_path = 'Data/gt_location.txt'

# Call the function and store the result
location_data_truth = read_location_data_truth(file_path)

# Print the dictionary to verify the content
'''for frame, loc in location_data_truth.items():
    print(f"Frame: {frame}, Location: {loc}")'''