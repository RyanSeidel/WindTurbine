#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Wind Turbine Digital Twin - Orientation Validation Model (from CSV Data)                                      #
# By [Wind Turbine Digital Twins]                                                                                 #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Description:                                                                                                    #
# This script validates the optimal orientation angle for a wind turbine based on experimental data loaded        #
# from CSV files. It filters data for specific wind directions (assumed to represent relative orientation        #
# angles: 0, 30, 45, 60 degrees), calculates the average RPM for each, and then uses Polynomial Regression       #
# to model the relationship and visualize the result. The goal is to show that RPM is maximized when the          #
# relative angle is close to 0 degrees.                                                                           #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import make_pipeline
from sklearn.metrics import r2_score
import joblib # Kept for potential model saving

#-----------------------------------------------------------------------------------------------------------------#
# 1. Configuration                                                                                                #
#-----------------------------------------------------------------------------------------------------------------#

# --- Files to Load ---
# !!! IMPORTANT !!! Uncomment the CSV file(s) containing the data for the 0, 30, 45, 60 degree tests.
data_files = [
    # Example: Uncomment the files relevant to your front-facing tests
    # 'wind_0_NoFanVib.csv',   # Assuming you have a file for 0 degrees
    # 'wind_30_NoFanVib.csv',
    # 'wind_45_NoFanVib.csv',
    # 'wind_60_NoFanVib.csv',
    # Or if angles are within a single file's 'wind_direction' column:
    'North_MedFan_0Degree.csv', # <--- !!! REPLACE OR UNCOMMENT APPROPRIATE FILES !!!
    'North_ZeroFan_0Degree.csv'
]

# --- Column Names ---
# !!! IMPORTANT !!! Verify these column names match your CSV files.
wind_direction_col = 'weatherstation_direction' # Column with the angle (0, 30, 45, 60)
rpm_col = 'rpm_value'                 # Column with the RPM measurements

# --- Target Angles ---
# The specific relative orientation angles we want to analyze
target_angles = np.array([0, 30, 45, 60])

# --- Angle Tolerance ---
# How close the 'wind_direction' value needs to be to the target angle
# E.g., 1.0 means angles between 29.0 and 31.0 will be considered '30'
angle_tolerance = 1.0

# --- Polynomial Degree ---
poly_degree = 3  # Degree for the Polynomial Regression (2 or 3 is usually good)

#-----------------------------------------------------------------------------------------------------------------#
# 2. Load and Process Data from CSV                                                                               #
#-----------------------------------------------------------------------------------------------------------------#
if not data_files or all(f.startswith('#') for f in data_files):
     raise ValueError("No data files specified. Please edit the 'data_files' list and uncomment the relevant CSV files.")

try:
    # Read and concatenate data from specified files
    data = pd.concat([pd.read_csv(f) for f in data_files if not f.startswith('#')], ignore_index=True)
    print(f"Loaded a total of {data.shape[0]} rows from {len(data_files)} file(s).")
except FileNotFoundError as e:
    print(f"Error loading file: {e}. Please ensure the file paths in 'data_files' are correct.")
    exit()
except Exception as e:
    print(f"An error occurred during data loading: {e}")
    exit()

# Check if required columns exist
if wind_direction_col not in data.columns or rpm_col not in data.columns:
    raise ValueError(f"Required columns '{wind_direction_col}' or '{rpm_col}' not found in the loaded data.")

# Calculate average RPM for each target angle
angle_rpm_map = {}
print("\nCalculating average RPM for target angles:")
for angle in target_angles:
    # Filter data for rows where wind_direction is close to the target angle
    angle_min = angle - angle_tolerance
    angle_max = angle + angle_tolerance
    filtered_data = data[(data[wind_direction_col] >= angle_min) & (data[wind_direction_col] <= angle_max)]

    if not filtered_data.empty:
        avg_rpm = filtered_data[rpm_col].mean()
        angle_rpm_map[angle] = avg_rpm
        print(f"  - Angle {angle}°: Found {len(filtered_data)} points, Average RPM = {avg_rpm:.2f}")
    else:
        print(f"  - Angle {angle}°: No data points found within tolerance ({angle_tolerance}°). Skipping this angle.")

# Prepare data for the model (only use angles where data was found)
if not angle_rpm_map:
    raise ValueError("No data found for any of the target angles. Cannot proceed.")

relative_angles_found = np.array(list(angle_rpm_map.keys())).reshape(-1, 1)
measured_rpms_avg = np.array(list(angle_rpm_map.values()))

print(f"\nUsing {len(relative_angles_found)} angle(s) with calculated average RPMs for validation.")
print("Angles Used (degrees):", relative_angles_found.flatten())
print("Average RPMs Used:", measured_rpms_avg)

#-----------------------------------------------------------------------------------------------------------------#
# 3. Define and Train the Polynomial Regression Model                                                             #
#-----------------------------------------------------------------------------------------------------------------#
# Create and train the pipeline
validation_model = make_pipeline(PolynomialFeatures(degree=poly_degree, include_bias=False),
                                 LinearRegression())

print(f"\nTraining Polynomial Regression model (Degree {poly_degree})...")
validation_model.fit(relative_angles_found, measured_rpms_avg)
print("Training complete.")

# Optional: Save the trained model
# joblib.dump(validation_model, 'orientation_validation_model_from_csv.pkl')
# print("Validation model saved as orientation_validation_model_from_csv.pkl.")

#-----------------------------------------------------------------------------------------------------------------#
# 4. Evaluate and Visualize the Model Fit                                                                         #
#-----------------------------------------------------------------------------------------------------------------#

# Generate a smooth range of angles for plotting the fitted curve
if len(relative_angles_found) > 1:
    angles_smooth = np.linspace(relative_angles_found.min(), relative_angles_found.max(), 300).reshape(-1, 1)
elif len(relative_angles_found) == 1:
     angles_smooth = relative_angles_found # Only one point, just use it
else:
     angles_smooth = np.array([]) # Should not happen due to earlier check

if angles_smooth.size > 0:
    rpms_predicted_smooth = validation_model.predict(angles_smooth)
else:
    rpms_predicted_smooth = np.array([])

# Predict RPMs for the original input angles to calculate metrics
rpms_predicted_original = validation_model.predict(relative_angles_found)

# Calculate R-squared score
r2 = r2_score(measured_rpms_avg, rpms_predicted_original)
print(f"\nR-squared score on the training data (average RPMs): {r2:.4f}")

# Find the angle corresponding to the maximum predicted RPM on the smooth curve
if rpms_predicted_smooth.size > 0:
    max_rpm_pred_index = np.argmax(rpms_predicted_smooth)
    max_rpm_angle_pred = angles_smooth[max_rpm_pred_index][0]
    max_rpm_pred_value = rpms_predicted_smooth[max_rpm_pred_index]

    print(f"\nAnalysis of the fitted curve (Degree {poly_degree}):")
    print(f"  - Predicted angle for maximum RPM: {max_rpm_angle_pred:.2f} degrees")
    print(f"  - Maximum predicted RPM value: {max_rpm_pred_value:.2f}")
else:
     print("\nCould not analyze fitted curve (not enough data points).")
     max_rpm_angle_pred = None
     max_rpm_pred_value = None


# Compare with actual maximum measured average point
max_rpm_actual_avg_index = np.argmax(measured_rpms_avg)
max_rpm_angle_actual_avg = relative_angles_found[max_rpm_actual_avg_index][0]
max_rpm_actual_avg_value = measured_rpms_avg[max_rpm_actual_avg_index]
print(f"  - Actual maximum average measured RPM: {max_rpm_actual_avg_value:.2f} at {max_rpm_angle_actual_avg} degrees")


# Plot the results
print("\nGenerating plot...")
plt.figure(figsize=(10, 6))

# Plot the average data points used for training
plt.scatter(relative_angles_found, measured_rpms_avg, color='red', s=100, label='Average Measured RPM per Angle', zorder=5)

# Plot the fitted polynomial curve
if angles_smooth.size > 0 and rpms_predicted_smooth.size > 0:
    plt.plot(angles_smooth, rpms_predicted_smooth, color='blue', label=f'Polynomial Fit (Degree {poly_degree})')

# Highlight the predicted maximum point on the curve
if max_rpm_angle_pred is not None and max_rpm_pred_value is not None:
    plt.scatter([max_rpm_angle_pred], [max_rpm_pred_value], color='green', s=150, marker='*', label=f'Predicted Max RPM ({max_rpm_pred_value:.1f})', zorder=6)

# Add labels, title, legend, and grid
plt.xlabel('Relative Orientation Angle (Degrees from Wind Direction)')
plt.ylabel('Average Measured RPM')
plt.title('Orientation Validation (from CSV): Average RPM vs. Relative Angle')
plt.legend()
plt.grid(True)
if relative_angles_found.size > 0:
    plt.xticks(np.arange(min(target_angles), max(target_angles) + 1, 15)) # Use target angles for ticks
    plt.xlim(min(target_angles) - 5, max(target_angles) + 5) # Add some padding based on target range
    plt.ylim(0, max(measured_rpms_avg) * 1.15) # Set y-axis limit slightly above max avg RPM
else:
    plt.xticks(np.arange(0, 61, 15))
    plt.xlim(-5, 65)
    plt.ylim(0, 1) # Default empty plot range


# Show the plot
plt.show()

print("\nScript finished.")
