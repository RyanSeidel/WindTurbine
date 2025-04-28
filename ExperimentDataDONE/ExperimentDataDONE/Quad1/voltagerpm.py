#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Wind Turbine Digital Twin - MLP Regressor Model for Predicting RPM, Voltage, Power, Current (Multi-Output)  # # <-- Updated Title
# By [Wind Turbine Digital Twins]                                                                               #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Description:                                                                                                  #
# This script processes wind turbine data, trains a Multi-layer Perceptron (MLP) Regressor                      #
# for MULTI-OUTPUT regression (using scikit-learn) to predict FOUR TARGETS: rpm_value, voltage_value,         # # <-- Updated Description
# power_value, AND current_value based on weather station (speed, direction, humidity, pressure),             # # <-- Updated Description
# orientation, and potentially linear acceleration features. It evaluates the model's performance FOR EACH      #
# TARGET using a train-test split and visualizes results, including a correlation matrix of inputs & targets.   #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from sklearn.neural_network import MLPRegressor
from sklearn.metrics import mean_squared_error, r2_score
import matplotlib.pyplot as plt
import numpy as np
import joblib
import seaborn as sns
import time

#-----------------------------------------------------------------------------------------------------------------#
# 1. Configuration                                                                                                #
#-----------------------------------------------------------------------------------------------------------------#

# --- Files to Load ---
# Ensure these files contain ALL required columns (inputs and all 4 targets)
data_files = [
    'North_HighFan_0Degree.csv',
    'North_MedFan_0Degree.csv',
    'North_LowFan_0Degree.csv',
    'North_ZeroFan_0Degree.csv',
    # 'NorthWest_HighFan_30Degree.csv',
    # 'NorthWest_MedFan_30Degree.csv',
    # 'NorthWest_LowFan_30Degree.csv',
    # 'NorthWest_HighFan_45Degree.csv',
    # 'NorthWest_MedFan_45Degree.csv',
    # 'NorthWest_LowFan_45Degree.csv',
    # 'NorthWest_HighFan_60Degree.csv',
    # 'NorthWest_MedFan_60Degree.csv',
    # 'NorthWest_LowFan_60Degree.csv',
    # 'West_HighFan_90Degree.csv',
    # 'SouthWest_MedFan_150Degree.csv',
    # 'West_LowFan_120Degree.csv',
    # 'South_HighFan_180Degree.csv',
    # 'East_HighFan_270Degree.csv',
    # 'NorthEast_HighFan_300Degree.csv',
    # i think it needs hot encode of low, med, high fan
    # 'NorthEast_MedFan_300Degree.csv',
    # 'NorthEast_LowFan_300Degree.csv',
    # 'NorthEast_HighFan_315Degree.csv',
    # 'NorthEast_MedFan_315Degree.csv',
    # 'NorthEast_LowFan_315Degree.csv',
    # 'NorthEast_HighFan_330Degree.csv',
    # 'NorthEast_LowFan_330Degree.csv',
    # 'NorthEast_MedFan_330Degree.csv', 
]

# --- Feature, Target, and Model Configuration ---
# Weather & Orientation (Inputs)
weather_speed_col = 'weatherstation_speed'
weather_direction_col = 'weatherstation_direction'
weather_humidity_col = 'weatherstation_humidity'
weather_pressure_col = 'weatherstation_pressure'
orientation_heading_col = 'orientation_heading'
orientation_roll_col = 'orientation_roll'
orientation_pitch_col = 'orientation_pitch'
# Linear Acceleration (Optional Input)
lin_accel_lx_col = 'linear_acceleration_lx'
lin_accel_ly_col = 'linear_acceleration_ly'
lin_accel_lz_col = 'linear_acceleration_lz'
# Electrical Values (Now Targets)
power_col = 'power_value'           # <--- Will be Target 3
current_col = 'current_value'       # <--- Will be Target 4
# Define TARGET Variable Names
target_rpm_col = 'rpm_value'          # Target 1
target_voltage_col = 'voltage_value'    # Target 2

# --- MLP Regressor Hyperparameters ---
# (Unchanged)
hidden_layer_config = (64, 32) # Might need larger network for 4 targets
activation_function = 'relu'
solver_algorithm = 'adam'
max_training_iterations = 500 # Might need more iterations
early_stopping_enabled = True
validation_set_fraction = 0.1
iterations_no_improvement = 10 # Might need to adjust patience

# --- Output Model/Pipeline Filename ---
# Filename constructed dynamically in Section 3

#-----------------------------------------------------------------------------------------------------------------#
# 2. Load Data (Unchanged - Ensure columns exist)                                                                 #
#-----------------------------------------------------------------------------------------------------------------#
start_load_time = time.time()
if not data_files or all(f.startswith('#') for f in data_files):
      raise ValueError("No data files specified. Please edit the 'data_files' list.")
loaded_data = []
print("Loading data...")
# (Loading loop remains the same)
for file_path in data_files:
    if file_path.startswith('#'): continue
    try:
        df = pd.read_csv(file_path, low_memory=False)
        loaded_data.append(df)
        print(f"  - Successfully loaded {file_path} ({df.shape[0]} rows)")
    except FileNotFoundError: print(f"Error: File not found - {file_path}."); exit()
    except Exception as e: print(f"An error occurred loading {file_path}: {e}"); exit()
if not loaded_data: print("Error: No data could be loaded."); exit()
data = pd.concat(loaded_data, ignore_index=True)
load_time = time.time() - start_load_time
print(f"\nCombined dataset shape: {data.shape}")
print(f"Data loading took {load_time:.2f} seconds.")

#-----------------------------------------------------------------------------------------------------------------#
# 3. Data Preprocessing & Feature/Target Selection                                                                #
#-----------------------------------------------------------------------------------------------------------------#
start_preprocess_time = time.time()
print("\nPreprocessing data...")

# --- Define INPUT Features (X) ---
# REMOVE power and current from features
features = [
    # Weather & Orientation
    weather_speed_col,
    weather_direction_col,   # (will be converted)
    weather_humidity_col,
    weather_pressure_col,
    orientation_heading_col,
    orientation_roll_col,
    orientation_pitch_col,

    # Linear Acceleration Features (Optional - Uncomment to include)
    lin_accel_lx_col,
    lin_accel_ly_col,
    lin_accel_lz_col
]

# --- Define TARGETS (y) ---
# ADD power and current to targets
targets = [
    target_rpm_col,
    target_voltage_col,
    power_col,          # <--- ADDED Target 3
    current_col         # <--- ADDED Target 4
]

# --- Determine included feature groups and construct output filename ---
include_lin_accel = any(col in features for col in [lin_accel_lx_col, lin_accel_ly_col, lin_accel_lz_col])
include_extra_weather = any(col in features for col in [weather_humidity_col, weather_pressure_col])

# Update base filename for 4 targets, remove elec_in logic
output_filename_parts = ['rpm_volt_pow_curr_multi_mlp_model'] # <--- Updated base name
if include_extra_weather:
    output_filename_parts.append('weather')
    print("Including extra Weather features (humidity, pressure) as INPUTS.")
else:
    print("Excluding extra Weather features from INPUTS.")
# REMOVED check for electrical inputs as they are now targets
if include_lin_accel:
    output_filename_parts.append('linaccel')
    print("Including Linear Acceleration features as INPUTS.")
else:
    print("Excluding Linear Acceleration features from INPUTS.")

output_model_filename = "_".join(output_filename_parts) + '.pkl'
print(f"Output model filename set to: {output_model_filename}")


# --- Define all columns needed (features + targets) ---
all_cols = features + targets # Correctly combines reduced features and expanded targets

# --- Check if ALL required columns exist before selecting ---
print(f"Checking for required columns: {all_cols}")
missing_cols = [col for col in all_cols if col not in data.columns]
if missing_cols:
    print(f"\nError: The following required columns are missing from the loaded data:")
    print(f"  {missing_cols}")
    print(f"\nPlease ensure your CSV files contain these columns OR adjust the 'features'/'targets' lists.")
    print(f"Available columns in loaded data: {data.columns.tolist()}")
    exit()

# --- Convert Weather Direction input feature to Radians ---
# (Preprocessing logic remains the same)
if weather_direction_col in features:
    print(f"Converting '{weather_direction_col}' from degrees to radians...")
    data[weather_direction_col] = pd.to_numeric(data[weather_direction_col], errors='coerce')
    rows_before_dropna_direction = data.shape[0]
    data.dropna(subset=[weather_direction_col], inplace=True)
    rows_after_dropna_direction = data.shape[0]
    if rows_before_dropna_direction > rows_after_dropna_direction:
        print(f"  Removed {rows_before_dropna_direction - rows_after_dropna_direction} rows due to non-numeric '{weather_direction_col}'.")
    data[weather_direction_col] = np.radians(data[weather_direction_col])
else:
     print(f"Skipping radian conversion as '{weather_direction_col}' is not in selected features.")

# --- Handle Other Missing Values & Ensure Numeric Types ---
initial_rows = data.shape[0]
print(f"Ensuring required columns ({len(all_cols)}) are numeric and handling missing values...")
for col in all_cols:
    if col in data.columns:
        if not pd.api.types.is_numeric_dtype(data[col]):
             print(f"  Converting column '{col}' to numeric.")
             data[col] = pd.to_numeric(data[col], errors='coerce')
        # No need to print about existing NaNs here, dropna handles it.

rows_before_final_dropna = data.shape[0]
data.dropna(subset=all_cols, inplace=True)
rows_after_dropna = data.shape[0]

additional_removed = rows_before_final_dropna - rows_after_dropna
if additional_removed > 0:
    print(f"Removed {additional_removed} rows due to missing/non-numeric values in required columns.")

if data.empty:
    print("Error: No data remaining after cleaning.")
    exit()

# --- Define X (features) and y (targets) AFTER cleaning ---
X = data[features]
y = data[targets] # y is now a DataFrame with 4 columns
preprocess_time = time.time() - start_preprocess_time
print(f"\nSelected Features ({len(features)}): {features}")
print(f"Selected Targets ({len(targets)}): {targets}")
print(f"Final dataset size for modeling: {X.shape[0]} rows")
print(f"Data preprocessing took {preprocess_time:.2f} seconds.")


# --- Train-Test Split ---
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
print(f"\nData split into training ({X_train.shape[0]} rows) and testing ({X_test.shape[0]} rows) sets.")
print(f"Shape of y_train: {y_train.shape}, Shape of y_test: {y_test.shape}") # Verify y shapes (n_samples, 4)


#-----------------------------------------------------------------------------------------------------------------#
# 4. Model Training (MLP Regressor Pipeline - Handles Multi-Output)                                               #
#-----------------------------------------------------------------------------------------------------------------#
start_train_time = time.time()
print(f"\nTraining Multi-Output MLP Regressor model (Predicting {len(targets)} targets)...")
model_pipeline = Pipeline([
    ('scaler', StandardScaler()),
    ('mlp_regressor', MLPRegressor(
        hidden_layer_sizes=hidden_layer_config, activation=activation_function, solver=solver_algorithm,
        max_iter=max_training_iterations, early_stopping=early_stopping_enabled,
        validation_fraction=validation_set_fraction, n_iter_no_change=iterations_no_improvement,
        random_state=42, verbose=True
    ))
])
model_pipeline.fit(X_train, y_train) # y_train now has 4 columns
train_time = time.time() - start_train_time
print(f"\nTraining complete. Took {train_time:.2f} seconds.")
joblib.dump(model_pipeline, output_model_filename)
print(f"Trained pipeline saved as {output_model_filename}.")


#-----------------------------------------------------------------------------------------------------------------#
# 5. Model Prediction & Evaluation (Evaluation loop adapts automatically)                                         #
#-----------------------------------------------------------------------------------------------------------------#
print("\nEvaluating model performance (per target)...")
start_eval_time = time.time()
# predict will return an array with shape (n_samples, 4)
y_train_pred = model_pipeline.predict(X_train)
y_test_pred = model_pipeline.predict(X_test)

# --- Evaluate the Model FOR EACH TARGET ---
mse_train_raw = mean_squared_error(y_train, y_train_pred, multioutput='raw_values')
mse_test_raw = mean_squared_error(y_test, y_test_pred, multioutput='raw_values')
rmse_train_raw = np.sqrt(mse_train_raw)
rmse_test_raw = np.sqrt(mse_test_raw)
r2_train_raw = r2_score(y_train, y_train_pred, multioutput='raw_values')
r2_test_raw = r2_score(y_test, y_test_pred, multioutput='raw_values')
eval_time = time.time() - start_eval_time

# Print performance metrics for each target (loop now runs 4 times)
print(f"\n--- Performance Metrics (Evaluation took {eval_time:.2f} seconds) ---")
for i, target_name in enumerate(targets): # Iterates through all 4 target names
    print(f"--- Target: {target_name} ---")
    print(f"  Training MSE:   {mse_train_raw[i]:.4f}, RMSE: {rmse_train_raw[i]:.4f}, R²: {r2_train_raw[i]:.4f} ({r2_train_raw[i]*100:.2f}%)")
    print(f"  Testing MSE:    {mse_test_raw[i]:.4f}, RMSE: {rmse_test_raw[i]:.4f}, R²: {r2_test_raw[i]:.4f} ({r2_test_raw[i]*100:.2f}%)")


#-----------------------------------------------------------------------------------------------------------------#
# 6. Visualization (Plotting loop adapts automatically)                                                          #
#-----------------------------------------------------------------------------------------------------------------#
print("\nGenerating visualizations (per target)...")
start_viz_time = time.time()

# --- Plot 0: Feature and Target Correlation Matrix ---
print("Calculating and plotting inputs & targets correlation matrix...")
correlation_matrix = data[all_cols].corr() # Includes all features and 4 targets

num_corr_cols = len(all_cols)
fig_width = max(8, num_corr_cols * 1.0); fig_height = max(6, num_corr_cols * 0.8)
plt.figure(figsize=(fig_width, fig_height))
sns.heatmap(correlation_matrix, annot=True, cmap='coolwarm', fmt=".2f", linewidths=.5, annot_kws={"size": 7})
plt.title(f'Correlation Matrix: Inputs and Targets ({len(targets)} Targets)') # Updated title slightly
plt.xticks(rotation=45, ha='right'); plt.yticks(rotation=0)
plt.tight_layout()
plt.savefig('Inputs_Targets_Correlation_Matrix.png', dpi=300, bbox_inches='tight') # Generic filename
print("Saved plot: Inputs_Targets_Correlation_Matrix.png")
plt.show()

# --- Create plots for EACH target variable ---
# This loop now iterates 4 times, creating plots for RPM, Voltage, Power, Current
for i, target_name in enumerate(targets):
    print(f"\nGenerating plots for target: {target_name}")

    # Extract actual and predicted values for the current target
    y_test_target_actual = y_test[target_name]
    y_train_target_actual = y_train[target_name]
    # Index the prediction array (shape n_samples, 4) using the loop variable i
    y_test_target_pred = y_test_pred[:, i]
    y_train_target_pred = y_train_pred[:, i]

    # --- Plot 1: Predicted vs Actual (for this target) ---
    plt.figure(figsize=(8, 6))
    plt.scatter(y_test_target_actual, y_test_target_pred, alpha=0.6, edgecolors='k', s=50, label="Test Data Points")
    min_val = min(y_test_target_actual.min(), y_test_target_pred.min())
    max_val = max(y_test_target_actual.max(), y_test_target_pred.max())
    plt.plot([min_val, max_val], [min_val, max_val], color='red', linestyle='--', linewidth=2, label="Perfect Fit Line (y=x)")
    plt.xlabel(f"Actual {target_name} (Test Set)")
    plt.ylabel(f"Predicted {target_name} (Test Set)")
    plt.title(f"MLP Multi-Output: Predicted vs Actual - {target_name}")
    plt.legend(); plt.grid(True)
    plt.savefig(f'MLP_Predicted_vs_Actual_{target_name}_Test.png', dpi=300, bbox_inches='tight')
    print(f"Saved plot: MLP_Predicted_vs_Actual_{target_name}_Test.png")
    plt.show()

    # --- Plot 2: Residual Plot (for this target) ---
    residuals_test_target = y_test_target_actual - y_test_target_pred
    plt.figure(figsize=(10, 6))
    plt.scatter(y_test_target_pred, residuals_test_target, alpha=0.6, edgecolors='k', s=50, label=f'{target_name} Test Set Residuals')
    plt.axhline(y=0, color='red', linestyle='--', linewidth=2, label='Zero Error Line')
    plt.xlabel(f"Predicted {target_name} (Test Set)")
    plt.ylabel("Residuals (Actual - Predicted)")
    plt.title(f"Residual Plot - {target_name} (MLP Multi-Output)")
    plt.legend(); plt.grid(True)
    plt.savefig(f'MLP_Residual_Plot_{target_name}_Test.png', dpi=300, bbox_inches='tight')
    print(f"Saved plot: MLP_Residual_Plot_{target_name}_Test.png")
    plt.show()

    # --- Plot 3: Actual vs Predicted over Training Sample Index (for this target) ---
    plt.figure(figsize=(15, 6))
    if isinstance(y_train_target_actual, pd.Series): y_train_values_target = y_train_target_actual.values
    else: y_train_values_target = y_train_target_actual
    y_train_pred_values_target = y_train_target_pred

    plot_subset = min(len(y_train_values_target), 500); indices = np.arange(plot_subset)
    plt.plot(indices, y_train_values_target[:plot_subset], label='Actual', marker='.', linestyle='-', alpha=0.7)
    plt.plot(indices, y_train_pred_values_target[:plot_subset], label='Predicted', marker='x', linestyle='--', alpha=0.7)
    plt.title(f'{target_name}: Actual vs Predicted (Training Sample - MLP Multi-Output)')
    plt.xlabel(f'Sample Index (First {plot_subset} Samples)'); plt.ylabel(target_name)
    plt.legend(); plt.grid(True)
    plt.savefig(f'MLP_Actual_vs_Predicted_{target_name}_Train_Index.png', dpi=300, bbox_inches='tight')
    print(f"Saved plot: MLP_Actual_vs_Predicted_{target_name}_Train_Index.png")
    plt.show()

viz_time = time.time() - start_viz_time
print(f"\nVisualizations generated. Took {viz_time:.2f} seconds.")
total_time = time.time() - start_load_time
print(f"\nScript finished. Total execution time: {total_time:.2f} seconds.")