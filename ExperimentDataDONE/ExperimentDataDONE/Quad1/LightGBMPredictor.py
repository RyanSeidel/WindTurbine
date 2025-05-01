#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Wind Turbine Digital Twin - Prediction Script using Saved LightGBM Model (Direct Input)                     #
# By [Wind Turbine Digital Twins]                                                                               #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Description:                                                                                                  #
# This script loads a previously trained LightGBM pipeline (saved as a .pkl file)                               #
# and uses it to predict RPM, Voltage, Power, and Current based on input values                                 #
# defined directly within the script. Only the predicted values are printed to the console.                     # # <-- Updated Description
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

import pandas as pd
import numpy as np
import joblib
import time
import os

#-----------------------------------------------------------------------------------------------------------------#
# 1. Configuration                                                                                                #
#-----------------------------------------------------------------------------------------------------------------#

# --- File Paths ---
# *** IMPORTANT: Specify the correct path to your saved model file ***
saved_model_filename = 'rpm_volt_pow_curr_multi_lgbm_model_fanflags_weather_linaccel.pkl'

# --- Output File (Optional - can be commented out if not needed) ---
# Specify the name for the output file containing inputs and predictions
# output_predictions_filename = 'lgbm_single_prediction_output.csv'

# --- Feature & Target Definitions (MUST MATCH the training script) ---
# Define the numerical features the model was trained on (in the exact same order)
numerical_features = [
    'weatherstation_speed',
    'weatherstation_direction',
    'weatherstation_humidity',
    'weatherstation_pressure',
    'weatherstation_altitude',
    'orientation_heading',
    'orientation_roll',
    'orientation_pitch',
    'low_fan',
    'med_fan',
    'high_fan',
    'linear_acceleration_lx',
    'linear_acceleration_ly',
    'linear_acceleration_lz'
]

# Define the names of the target variables predicted by the model
target_names = [
    'rpm_value',
    'voltage_value',
    'power_value',
    'current_value'
]

# Define the weather direction column if it needs radian conversion
weather_direction_col = 'weatherstation_direction' # Set to None if not used or already in radians

#-----------------------------------------------------------------------------------------------------------------#
# 2. Define Input Values for Prediction                                                                           #
#-----------------------------------------------------------------------------------------------------------------#

# *** IMPORTANT: Define the input values for the single prediction here ***
# Ensure all keys match the 'numerical_features' list above.

# NorthWest_HighFan_45Degree
input_data_dict = {
    'weatherstation_speed': 4.5,         # Example value
    'weatherstation_direction': 45.0,     # Example value (in degrees if conversion is enabled)
    'weatherstation_humidity': 65.39,      # Example value
    'weatherstation_pressure': 1020.68,    # Example value
    'weatherstation_altitude': -47.3,      # Example value
    'orientation_heading': 355.875,          # Example value
    'orientation_roll': -1.9375,              # Example value
    'orientation_pitch': -0.75,            # Example value
    'low_fan': 0,                         # Example value (0 or 1)
    'med_fan': 0,                         # Example value (0 or 1)
    'high_fan': 1,                        # Example value (0 or 1)
    'linear_acceleration_lx': -0.24,        # Example value
    'linear_acceleration_ly': -0.17,      # Example value
    'linear_acceleration_lz': -0.43         # Example value
}
# --- End Input Definition ---

#ZeroFan_0Degree
# input_data_dict = {
#     'weatherstation_speed': 0,         # Example value
#     'weatherstation_direction': 0,     # Example value (in degrees if conversion is enabled)
#     'weatherstation_humidity': 65.81,      # Example value
#     'weatherstation_pressure': 1021.01,    # Example value
#     'weatherstation_altitude': -49.83,      # Example value
#     'orientation_heading': 348.9375,          # Example value
#     'orientation_roll': -2.3125,              # Example value
#     'orientation_pitch': -0.75,            # Example value
#     'low_fan': 0,                         # Example value (0 or 1)
#     'med_fan': 0,                         # Example value (0 or 1)
#     'high_fan': 0,                        # Example value (0 or 1)
#     'linear_acceleration_lx': -0.07,        # Example value
#     'linear_acceleration_ly': -0.01,      # Example value
#     'linear_acceleration_lz': -0.35        # Example value
# }
# --- End Input Definition ---


#ZeroFan_0Degree
# input_data_dict = {
#     'weatherstation_speed': 0,         # Example value
#     'weatherstation_direction': 0,     # Example value (in degrees if conversion is enabled)
#     'weatherstation_humidity': 65.81,      # Example value
#     'weatherstation_pressure': 1021.01,    # Example value
#     'weatherstation_altitude': -49.83,      # Example value
#     'orientation_heading': 348.9375,          # Example value
#     'orientation_roll': -2.3125,              # Example value
#     'orientation_pitch': -0.75,            # Example value
#     'low_fan': 0,                         # Example value (0 or 1)
#     'med_fan': 0,                         # Example value (0 or 1)
#     'high_fan': 0,                        # Example value (0 or 1)
#     'linear_acceleration_lx': -0.07,        # Example value
#     'linear_acceleration_ly': -0.01,      # Example value
#     'linear_acceleration_lz': -0.35        # Example value
# }
# --- End Input Definition ---

# South_HighFan_180Degree
# input_data_dict = {
#     'weatherstation_speed': 5.4,         # Example value
#     'weatherstation_direction': 180,     # Example value (in degrees if conversion is enabled)
#     'weatherstation_humidity': 64.96,      # Example value
#     'weatherstation_pressure': 1020.47,    # Example value
#     'weatherstation_altitude': -44.97,      # Example value
#     'orientation_heading': 2,          # Example value
#     'orientation_roll': -4.0625,              # Example value
#     'orientation_pitch': -0.5,            # Example value
#     'low_fan': 0,                         # Example value (0 or 1)
#     'med_fan': 0,                         # Example value (0 or 1)
#     'high_fan': 1,                        # Example value (0 or 1)
#     'linear_acceleration_lx': -0.12,        # Example value
#     'linear_acceleration_ly': 0.32,      # Example value
#     'linear_acceleration_lz': -0.33        # Example value
# }
# --- End Input Definition ---

#-----------------------------------------------------------------------------------------------------------------#
# 3. Load Model                                                                                                   #
#-----------------------------------------------------------------------------------------------------------------#
print(f"Loading saved model pipeline from: {saved_model_filename}")
start_load_model_time = time.time()

if not os.path.exists(saved_model_filename):
    print(f"\n--- ERROR ---")
    print(f"Model file not found at: {saved_model_filename}")
    print(f"Please ensure the file exists and the path is correct.")
    exit()

try:
    # Load the entire pipeline object (includes preprocessor and model)
    model_pipeline = joblib.load(saved_model_filename)
    print("Model pipeline loaded successfully.")
except Exception as e:
    print(f"\n--- ERROR ---")
    print(f"An error occurred loading the model file: {e}")
    exit()

load_model_time = time.time() - start_load_model_time
print(f"Model loading took {load_model_time:.2f} seconds.")

#-----------------------------------------------------------------------------------------------------------------#
# 4. Prepare Input Data                                                                                           #
#-----------------------------------------------------------------------------------------------------------------#
print("\nPreparing direct input data...")
start_prepare_data_time = time.time()

# Validate that the dictionary contains all necessary keys
missing_keys = [key for key in numerical_features if key not in input_data_dict]
if missing_keys:
    print(f"\n--- ERROR ---")
    print(f"The following required features are missing from the 'input_data_dict':")
    for key in missing_keys:
        print(f"  - {key}")
    print(f"\nPlease define all features in the dictionary.")
    exit()

# Convert the dictionary to a pandas DataFrame (with a single row)
input_df = pd.DataFrame([input_data_dict])

# Ensure correct data types (attempt conversion)
print("Ensuring numeric data types...")
for col in numerical_features:
    try:
        input_df[col] = pd.to_numeric(input_df[col])
    except ValueError:
        print(f"\n--- ERROR ---")
        print(f"Could not convert value for '{col}' ('{input_df[col].iloc[0]}') to numeric.")
        exit()

# Check for any remaining NaN values after conversion
if input_df[numerical_features].isnull().values.any():
     print(f"\n--- ERROR ---")
     print(f"Missing or non-numeric values detected in input data after conversion.")
     print(input_df.info())
     exit()

# Convert Weather Direction to Radians if needed
if weather_direction_col and weather_direction_col in input_df.columns:
    print(f"Converting '{weather_direction_col}' from degrees to radians...")
    input_df[weather_direction_col] = np.radians(input_df[weather_direction_col])

prepare_data_time = time.time() - start_prepare_data_time
print(f"Data preparation took {prepare_data_time:.2f} seconds.")

#-----------------------------------------------------------------------------------------------------------------#
# 5. Make Prediction                                                                                              #
#-----------------------------------------------------------------------------------------------------------------#
print("\nMaking prediction using the loaded model...")
start_predict_time = time.time()

try:
    # Use the loaded pipeline to make predictions on the single input row
    predictions_array = model_pipeline.predict(input_df[numerical_features]) # Pass the DataFrame slice
except Exception as e:
    print(f"\n--- ERROR ---")
    print(f"An error occurred during prediction: {e}")
    exit()

predict_time = time.time() - start_predict_time
print(f"Prediction generated successfully. Took {predict_time:.2f} seconds.")

#-----------------------------------------------------------------------------------------------------------------#
# 6. Format and Display Results                                                                                   #
#-----------------------------------------------------------------------------------------------------------------#
print("\nFormatting results...")

# Create a DataFrame for the predictions
predictions_dict = {f'predicted_{name}': predictions_array[0, i] for i, name in enumerate(target_names)}
predictions_df_single = pd.DataFrame([predictions_dict]) # Create DF with one row

# --- Display Results in Console (Predictions Only) --- # <-- MODIFIED SECTION
print("\n--- Prediction Results ---")
# Set display options for better console output (optional)
pd.set_option('display.max_columns', None) # Show all columns
pd.set_option('display.width', 1000)       # Adjust width as needed
# Print only the predictions DataFrame
print(predictions_df_single.to_string(index=False)) # Use .to_string() for better formatting
print("-" * 50)
# --- End Display Section ---

# --- Optional: Save the single result to a CSV file ---
# print(f"\nSaving results to CSV...")
# try:
#     # Combine input and predictions if saving
#     input_df_display = pd.DataFrame([input_data_dict])
#     results_df = pd.concat([input_df_display.reset_index(drop=True), predictions_df_single.reset_index(drop=True)], axis=1)
#     # If the output file exists, append; otherwise, create it with headers
#     file_exists = os.path.exists(output_predictions_filename)
#     results_df.to_csv(output_predictions_filename, mode='a', header=not file_exists, index=False)
#     print(f"Results {'appended' if file_exists else 'saved'} successfully to: {output_predictions_filename}")
# except Exception as e:
#     print(f"\n--- ERROR ---")
#     print(f"An error occurred saving the results to CSV: {e}")
# --- End Optional Save Section ---

print("\nPrediction script finished.")
