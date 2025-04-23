#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Wind Turbine Digital Twin - Polynomial Regression Model for Predicting Weather Station Direction            #
# By [Wind Turbine Digital Twins]                                                                               #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Description:                                                                                                  #
# This script processes wind turbine data from specified CSV files, trains a Polynomial Regression model        #
# to predict weather station direction based on the turbine's orientation heading, weather speed, RPM,          # # <-- Updated Description
# linear acceleration, and accelerometer features. It evaluates the model's performance using a train-test      #
# split and visualizes the results.                                                                             #
#                                                                                                               #
# IMPORTANT LIMITATION: Weather direction is often a circular variable (0-360 degrees). Standard polynomial     #
# regression treats it linearly. This can lead to suboptimal predictions near the 0/360 boundary.               #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler, PolynomialFeatures
from sklearn.pipeline import Pipeline
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score
import matplotlib.pyplot as plt
import numpy as np
import joblib
# import seaborn as sns # Not used in this version

#-----------------------------------------------------------------------------------------------------------------#
# 1. Configuration                                                                                                #
#-----------------------------------------------------------------------------------------------------------------#

# --- Files to Load ---
# !!! IMPORTANT !!! Replace or add your actual CSV file names to this list.
data_files = [
    'North_HighFan_0Degree.csv',
    'North_MedFan_0Degree.csv',
    'North_LowFan_0Degree.csv',
    'North_ZeroFan_0Degree.csv',
    'NorthWest_HighFan_30Degree.csv',
    'NorthWest_MedFan_30Degree.csv',
    'NorthWest_LowFan_30Degree.csv',
    'NorthWest_HighFan_45Degree.csv',
    'NorthWest_MedFan_45Degree.csv',
    'NorthWest_LowFan_45Degree.csv',
    'NorthWest_HighFan_60Degree.csv',
    'NorthWest_MedFan_60Degree.csv',
    'NorthWest_LowFan_60Degree.csv',
    'West_HighFan_90Degree.csv',
    # 'SouthWest_MedFan_150Degree.csv',
    # 'West_LowFan_120Degree.csv',
    'South_HighFan_180Degree.csv', 
    'East_HighFan_270Degree.csv', # only trying every 90 angle
    'NorthEast_HighFan_300Degree.csv',
    #'NorthEast_HighFan_330Degree.csv', # i think it needs hot encode of low, med, high fan
    'NorthEast_MedFan_300Degree.csv',
    'NorthEast_LowFan_300Degree.csv',
    'NorthEast_HighFan_315Degree.csv',
    # 'NorthEast_MedFan_315Degree.csv',
    # 'NorthEast_LowFan_315Degree.csv',
    # 'NorthEast_HighFan_330Degree.csv',
    # 'NorthEast_LowFan_330Degree.csv',

]
# comment i know this is a lot of cvs but I want to be able to pin down which cvs is good and what is not needed I don't plan on using all of them!


# --- Feature, Target, and Model Configuration ---
# !!! IMPORTANT !!! Verify these column names match your CSV files.

# --- Define INPUT FEATURES ---
orientation_heading_col = 'orientation_heading' # Input feature
weather_speed_col = 'weatherstation_speed'      # Input feature
rpm_value_col = 'rpm_value'                     # Input feature
# Linear Acceleration Features
lin_accel_lx_col = 'linear_acceleration_lx'
lin_accel_ly_col = 'linear_acceleration_ly'
lin_accel_lz_col = 'linear_acceleration_lz'
# Accelerometer Features (NEW)
accel_ax_col = 'accelerometer_ax'
accel_ay_col = 'accelerometer_ay'
accel_az_col = 'accelerometer_az'


# --- Define TARGET VARIABLE ---
target_weather_direction_col = 'weatherstation_direction' # Target is Weather Direction

polynomial_degree = 2 # Degree of polynomial features (e.g., 2 for quadratic, 3 for cubic)

# --- Output Model/Pipeline Filename ---
# Keeping v2, but consider changing if this is a significant model update
output_model_filename = 'weather_direction_poly_model_v2.pkl'

#-----------------------------------------------------------------------------------------------------------------#
# 2. Load Data                                                                                                    #
#-----------------------------------------------------------------------------------------------------------------#
if not data_files or all(f.startswith('#') for f in data_files):
     raise ValueError("No data files specified. Please edit the 'data_files' list with your CSV file names.")

loaded_data = []
print("Loading data...")
for file_path in data_files:
    if file_path.startswith('#'): # Allow commenting out files in the list
        continue
    try:
        # Added low_memory=False as it can sometimes help with mixed types during load
        df = pd.read_csv(file_path, low_memory=False)
        loaded_data.append(df)
        print(f"  - Successfully loaded {file_path} ({df.shape[0]} rows)")
    except FileNotFoundError:
        print(f"Error: File not found - {file_path}. Please ensure the file exists and the path is correct.")
        exit()
    except Exception as e:
        print(f"An error occurred loading {file_path}: {e}")
        exit()

if not loaded_data:
    print("Error: No data could be loaded from the specified files.")
    exit()

# Combine data from all loaded files
data = pd.concat(loaded_data, ignore_index=True)
print(f"\nCombined dataset shape: {data.shape}")
# print(data.info()) # Uncomment to see column types and non-null counts

#-----------------------------------------------------------------------------------------------------------------#
# 3. Data Preprocessing & Feature Selection                                                                       #
#-----------------------------------------------------------------------------------------------------------------#
print("\nPreprocessing data...")

# --- Define Features (X) and Target (y) ---
# Define the list of input feature column names (Added Accelerometer)
features = [
    orientation_heading_col, # Input
    weather_speed_col,       # Input
    rpm_value_col,           # Input
    lin_accel_lx_col,        # Input
    lin_accel_ly_col,        # Input
    lin_accel_lz_col,        # Input
    accel_ax_col,            # Input (NEW)
    accel_ay_col,            # Input (NEW)
    accel_az_col             # Input (NEW)
]
target = target_weather_direction_col # Single target column

# Check if columns exist before selecting
missing_features = [col for col in features if col not in data.columns]
if target not in data.columns:
    missing_target = [target] # Target is a single string here
else:
    missing_target = []

if missing_features or missing_target:
    print(f"Error: The following required columns are missing from the data:")
    if missing_features: print(f"  Features: {missing_features}")
    if missing_target: print(f"  Target: {missing_target}")
    print(f"Available columns are: {data.columns.tolist()}")
    exit()

# --- Handle Missing Values (Example: Drop rows with NaNs in features/target) ---
initial_rows = data.shape[0]
data.dropna(subset=features + [target], inplace=True) # Check both features and target
rows_after_dropna = data.shape[0]
if initial_rows > rows_after_dropna:
    print(f"Removed {initial_rows - rows_after_dropna} rows with missing values in features or target.")

if data.empty:
    print("Error: No data remaining after handling missing values.")
    exit()

X = data[features]
y = data[target] # y is now a Series (single target)
print(f"Selected Features ({len(features)}): {features}") # Will now print 9 features
print(f"Selected Target: {target}") # Should now show weather direction

# --- Train-Test Split ---
# Split data into training and testing sets
# Using test_size=0.2 based on user's last provided code
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42)
print(f"Data split into training ({X_train.shape[0]} rows) and testing ({X_test.shape[0]} rows) sets.")

# Note: Scaling is handled *within* the pipeline.

#-----------------------------------------------------------------------------------------------------------------#
# 4. Model Training (Polynomial Regression Pipeline)                                                              #
#-----------------------------------------------------------------------------------------------------------------#
print(f"\nTraining Polynomial Regression model (degree={polynomial_degree}) to predict {target}...") # Updated print

# Create the pipeline: StandardScaler -> PolynomialFeatures -> LinearRegression
model_pipeline = Pipeline([
    ('scaler', StandardScaler()), # Step 1: Standardize features
    ('poly_features', PolynomialFeatures(degree=polynomial_degree, include_bias=False)), # Step 2: Create polynomial features
    ('linear_regression', LinearRegression()) # Step 3: Fit linear model on polynomial features
])

# Train the entire pipeline on the training data
model_pipeline.fit(X_train, y_train)
print("Training complete.")

# Save the trained pipeline (includes scaler, poly features, and model)
joblib.dump(model_pipeline, output_model_filename)
print(f"Trained pipeline saved as {output_model_filename}.") # Uses correct filename


#-----------------------------------------------------------------------------------------------------------------#
# 5. Model Prediction & Evaluation                                                                                #
#-----------------------------------------------------------------------------------------------------------------#
print("\nEvaluating model performance...")
# Predictions on both training and testing sets using the pipeline
y_train_pred = model_pipeline.predict(X_train)
y_test_pred = model_pipeline.predict(X_test)

# --- Evaluate the Model ---
# Calculate metrics on both training and testing data
mse_train = mean_squared_error(y_train, y_train_pred)
mse_test = mean_squared_error(y_test, y_test_pred)
rmse_train = np.sqrt(mse_train)
rmse_test = np.sqrt(mse_test)

r2_train = r2_score(y_train, y_train_pred)
r2_test = r2_score(y_test, y_test_pred)

accuracy_train = r2_train * 100
accuracy_test = r2_test * 100

# Print performance metrics
print(f"\n--- Performance Metrics (Predicting {target}) ---") # Updated print
print(f"Training MSE:   {mse_train:.4f}, Training RMSE:   {rmse_train:.4f}, Training R²:   {r2_train:.4f} ({accuracy_train:.2f}%)")
print(f"Testing MSE:    {mse_test:.4f}, Testing RMSE:    {rmse_test:.4f}, Testing R²:    {r2_test:.4f} ({accuracy_test:.2f}%)")
print(f"\nNote: R² can be low or negative for {target} prediction due to its circular nature and model limitations.") # Updated print

#-----------------------------------------------------------------------------------------------------------------#
# 6. Visualization                                                                                                #
#-----------------------------------------------------------------------------------------------------------------#
print("\nGenerating visualizations...")

# --- Plot 1: Predicted vs Actual Weather Direction (Test Set) --- # Updated title/labels
plt.figure(figsize=(8, 6))
plt.scatter(y_test, y_test_pred, alpha=0.6, edgecolors='k', s=50, label="Test Data Points")
# Add a y=x line for reference
min_val = min(y_test.min(), y_test_pred.min())
max_val = max(y_test.max(), y_test_pred.max())
plt.plot([min_val, max_val], [min_val, max_val], color='red', linestyle='--', linewidth=2, label="Perfect Fit Line (y=x)")
plt.xlabel(f"Actual {target_weather_direction_col} (Test Set)") # Updated label
plt.ylabel(f"Predicted {target_weather_direction_col} (Test Set)") # Updated label
plt.title(f"Polynomial Regression (Deg={polynomial_degree}): Predicted vs Actual Weather Direction") # Updated title
plt.legend()
plt.grid(True)
plt.savefig('Predicted_vs_Actual_Direction_Test.png', dpi=300, bbox_inches='tight') # Updated filename
print("Saved plot: Predicted_vs_Actual_Direction_Test.png")
plt.show()


# --- Plot 2: Residual Plot (Test Set) --- # Updated title/labels
# Residuals = Actual - Predicted
residuals_test = y_test - y_test_pred

plt.figure(figsize=(10, 6))
plt.scatter(y_test_pred, residuals_test, alpha=0.6, edgecolors='k', s=50, label='Test Set Residuals')
plt.axhline(y=0, color='red', linestyle='--', linewidth=2, label='Zero Error Line')
plt.xlabel(f"Predicted {target_weather_direction_col} (Test Set)") # Updated label
plt.ylabel("Residuals (Actual - Predicted)")
plt.title(f"Residual Plot: Errors vs Predicted Weather Direction on Test Data") # Updated title
plt.legend()
plt.grid(True)
plt.savefig('Residual_Plot_Direction_Test.png', dpi=300, bbox_inches='tight') # Updated filename
print("Saved plot: Residual_Plot_Direction_Test.png")
plt.show()

# --- (Optional) Plotting against individual features ---
# Plotting against multiple polynomial features is complex.
# We can plot the actual data against one feature for context.
# Note that predictions (y_test_pred) are based on *all* input features now.
# (Code for individual feature plots remains commented out as before)


print("\nScript finished.")
