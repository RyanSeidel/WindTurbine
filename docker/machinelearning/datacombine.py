import pandas as pd

# Map direction numbers to corresponding degrees
direction_to_degrees = {
    1: 360,  # North
    2: 315,  # NE
    3: 270,  # East
    4: 225,  # SE
    5: 180,  # South
    6: 135,  # SW
    7: 90,   # West
    8: 45    # NW
}

# List of CSV files to combine
data_files = [
    'wind_turbine_North_90_Degree_HighFan.csv',
    'West_90Degree_HighFan.csv',
    'SouthEast_90Degree_HighFan.csv',
    'SouthWest_90Degree_HighFan.csv',
    'NorthEast_90Degree_HighFan.csv',
    'NorthWest_90Degree_HighFan.csv',
    'East_90Degree_HighFan.csv',
    'South_90Degree_HighFan.csv'
]

# Load and combine all datasets
data = pd.concat([pd.read_csv(f) for f in data_files], ignore_index=True)

# Map the direction values to degrees based on your mapping
data['direction_degrees'] = data['direction_value'].map(direction_to_degrees)

# Add the alignment category column with a fixed value of 2
data['alignment_category'] = 2

# Save updated dataset to a new file for verification (optional)
data.to_csv('AllDataWith90Degree.csv', index=False)

# Print confirmation and check a few rows
print("Datasets combined and alignment category added.")
print(data[['direction_value', 'direction_degrees', 'alignment_category']].head())
