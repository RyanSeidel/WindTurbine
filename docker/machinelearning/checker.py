import pandas as pd

# Load dataset
file_path = 'AllDataUpdated3_filtered_data.csv'  # Replace with your file path
data = pd.read_csv(file_path)

# Define a function to compute the alignment category
def calculate_alignment_category(row):
    orientation = row['orientation_heading']
    servo = row['servo_value']
    
    # Calculate the raw difference
    difference = abs(orientation - servo)
    
    # Handle wraparound cases (e.g., 350° vs. 10° should be 20° difference)
    difference = min(difference, 360 - difference)
    
    # Assign alignment categories
    if difference == 0:
        return 0  # Fully aligned
    elif difference <= 45:  # Within 45° is partially aligned
        return 1
    else:  # Greater than 45° is fully misaligned
        return 2

# Apply the function row-wise to calculate alignment_category
data['alignment_category'] = data.apply(calculate_alignment_category, axis=1)

# Save the updated dataset to a new CSV file (optional)
data.to_csv('AllDataWithAlignmentCategory.csv', index=False)

# Print confirmation and check the updated DataFrame
print("Alignment category added to dataset.")
print(data[['orientation_heading', 'servo_value', 'alignment_category']].head())
