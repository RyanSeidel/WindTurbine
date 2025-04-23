#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Wind Turbine Digital Twin - Polynomial Regression Model for Predicting Orientation Heading                  # # <-- Corrected Title
# By [Wind Turbine Digital Twins]                                                                               #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Description:                                                                                                  #
# This script processes wind turbine data from specified CSV files, trains a Polynomial Regression model        #
# to predict orientation heading based on weather station speed, direction, RPM, accelerometer,                 # # <-- Corrected Description
# linear acceleration, and voltage features. It evaluates the model's performance using a train-test split and  #
# visualizes the results.                                                                                       #
#                                                                                                               #
# IMPORTANT LIMITATION: Orientation heading is a circular variable (0-360 degrees). Standard polynomial       #
# regression treats it linearly, which means it doesn't understand that 359 degrees is close to 1 degree.       #
# This can lead to suboptimal predictions near the 0/360 boundary and potential predictions outside the range.  #
# For more accurate modeling of circular data, consider transformations (sin/cos) or specialized models.       #
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
    'NorthWest_LowFan_45Degree.csv'
    # 'your_data_file_2.csv',
]

# --- Feature, Target, and Model Configuration ---
# !!! IMPORTANT !!! Verify these column names match your CSV files.
# Input Features
weather_speed_col = 'weatherstation_speed'
weather_direction_col = 'weatherstation_direction' # Used as INPUT
rpm_value_col = 'rpm_value'
accel_ax_col = 'accelerometer_ax'
accel_ay_col = 'accelerometer_ay'
accel_az_col = 'accelerometer_az'
lin_accel_lx_col = 'linear_acceleration_lx'
lin_accel_ly_col = 'linear_acceleration_ly'
lin_accel_lz_col = 'linear_acceleration_lz'
voltage_value_col = 'voltage_value'
# Target Variable
target_orientation_col = 'orientation_heading' # <-- Correct Target

polynomial_degree = 2 # Degree of polynomial features (e.g., 2 for quadratic, 3 for cubic)

# --- Output Model/Pipeline Filename ---
output_model_filename = 'orientation_poly_regression_model.pkl' # <-- Correct Filename

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
# Define the list of input feature column names (10 features)
features = [
    weather_speed_col, weather_direction_col, rpm_value_col,
    accel_ax_col, accel_ay_col, accel_az_col,
    lin_accel_lx_col, lin_accel_ly_col, lin_accel_lz_col,
    voltage_value_col
]
target = target_orientation_col # Set the correct target

# Check if columns exist before selecting
missing_cols = [col for col in features if col not in data.columns]
if target not in data.columns:
    missing_cols.append(target)
if missing_cols:
    print(f"Error: The following required columns are missing from the data: {missing_cols}")
    print(f"Available columns are: {data.columns.tolist()}")
    exit()

# --- Handle Missing Values (Example: Drop rows with NaNs in features/target) ---
initial_rows = data.shape[0]
data.dropna(subset=features + [target], inplace=True)
rows_after_dropna = data.shape[0]
if initial_rows > rows_after_dropna:
    print(f"Removed {initial_rows - rows_after_dropna} rows with missing values in features or target.")

if data.empty:
    print("Error: No data remaining after handling missing values.")
    exit()

X = data[features]
y = data[target]
print(f"Selected Features ({len(features)}): {features}")
print(f"Selected Target: {target}") # Should now show orientation heading

# --- Train-Test Split ---
# Split data into training and testing sets
# Using test_size=0.3 as in the user's provided code
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.5, random_state=42)
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

# --- Plot 1: Predicted vs Actual Orientation Heading (Test Set) --- # Corrected title/labels
plt.figure(figsize=(8, 6))
plt.scatter(y_test, y_test_pred, alpha=0.6, edgecolors='k', s=50, label="Test Data Points")
# Add a y=x line for reference
min_val = min(y_test.min(), y_test_pred.min())
max_val = max(y_test.max(), y_test_pred.max())
plt.plot([min_val, max_val], [min_val, max_val], color='red', linestyle='--', linewidth=2, label="Perfect Fit Line (y=x)")
plt.xlabel(f"Actual {target_orientation_col} (Test Set)") # Corrected label
plt.ylabel(f"Predicted {target_orientation_col} (Test Set)") # Corrected label
plt.title(f"Polynomial Regression (Deg={polynomial_degree}): Predicted vs Actual Heading") # Corrected title
plt.legend()
plt.grid(True)
plt.savefig('Predicted_vs_Actual_Heading_Test.png', dpi=300, bbox_inches='tight') # Corrected filename
print("Saved plot: Predicted_vs_Actual_Heading_Test.png")
plt.show()


# --- Plot 2: Residual Plot (Test Set) --- # Corrected title/labels
# Residuals = Actual - Predicted
residuals_test = y_test - y_test_pred

plt.figure(figsize=(10, 6))
plt.scatter(y_test_pred, residuals_test, alpha=0.6, edgecolors='k', s=50, label='Test Set Residuals')
plt.axhline(y=0, color='red', linestyle='--', linewidth=2, label='Zero Error Line')
plt.xlabel(f"Predicted {target_orientation_col} (Test Set)") # Corrected label
plt.ylabel("Residuals (Actual - Predicted)")
plt.title(f"Residual Plot: Errors vs Predicted Heading on Test Data") # Corrected title
plt.legend()
plt.grid(True)
plt.savefig('Residual_Plot_Heading_Test.png', dpi=300, bbox_inches='tight') # Corrected filename
print("Saved plot: Residual_Plot_Heading_Test.png")
plt.show()

# --- (Optional) Plotting against individual features ---
# Plotting against multiple polynomial features is complex.
# We can plot the actual data against one feature for context.
# Note that predictions (y_test_pred) are based on *all* input features now.
# (Code for individual feature plots remains commented out as before)


print("\nScript finished.")
