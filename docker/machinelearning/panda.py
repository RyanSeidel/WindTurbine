import pandas as pd

# List of all CSV files to combine
data_files = [
    'All60BladeDegree.csv',
    'All45BladeDegree.csv',
    'wind_30Degree_FrontAllUpdate.csv',
    'wind_30Degree_FrontHighAllUpdate.csv',
    'wind_15Degree_FrontAllUpdate.csv',
    'wind_15Degree_FrontHighAllUpdate.csv',  
    'wind_45Degree_FrontAllUpdate.csv',
    'wind_45Degree_FrontHighAllUpdate.csv',
    'wind_45Degree_AllUpdate.csv',
    'wind_45Degree_AllLow.csv',
    'wind_30Degree_AllUpdate.csv',
    'wind_30Degree_AllLow.csv', 
    'wind_15Degree_AllUpdate.csv',
    'wind_15Degree_AllLow.csv'
]

# Combine all CSV files into one DataFrame
combined_data = pd.concat([pd.read_csv(file) for file in data_files], ignore_index=True)

# Save the combined data to a new CSV file (optional)
combined_data.to_csv('Combine_All_RPM_data.csv', index=False)

# Display the first few rows of the combined DataFrame
print(combined_data.head())

# Display information about the combined DataFrame
print(combined_data.info())
