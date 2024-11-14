import pandas as pd

# Load the CSV data
df = pd.read_csv("cleaned_wind_turbine_data.csv", index_col=0)

# Display the column names to verify them
print(df.columns)
