import pandas as pd

# List of CSV files to load
data_files = [
    #SouthEast
    'AllDataUpdated3_filtered_data.csv',
]

# Load all datasets into one DataFrame
combined_data = pd.concat([pd.read_csv(f) for f in data_files], ignore_index=True)

# Define the conditions
conditions = [
    (combined_data['rpm_value'] == 0),
    combined_data['rpm_value'].between(40, 50),
    combined_data['rpm_value'].between(58, 61),
]

# Check for each direction if at least one row satisfies the conditions
directions = combined_data['direction_value'].unique()  # Get all unique directions
all_directions_valid = True  # Assume all directions are valid initially

for direction in range(1, 9):  # Loop through all directions (1 to 8)
    direction_data = combined_data[combined_data['direction_value'] == direction]
    if not any(
        direction_data[condition].shape[0] > 0 for condition in conditions
    ):
        print(f"Direction {direction} does not satisfy the conditions.")
        all_directions_valid = False

# Final confirmation
if all_directions_valid:
    print("Yes, all directions have at least one RPM value in the specified ranges.")
else:
    print("No, some directions are missing the required RPM values.")
