#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Wind Turbine Digital Twin - MLP Regressor Model for Predicting RPM, Voltage, Power, Current (Multi-Output)  #
# By [Wind Turbine Digital Twins]                                                                               #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Description:                                                                                                  #
# This script processes wind turbine data, trains a Multi-layer Perceptron (MLP) Regressor                      #
# for MULTI-OUTPUT regression (using scikit-learn) to predict FOUR TARGETS: rpm_value, voltage_value,           #
# power_value, AND current_value based on weather station (speed, direction, humidity, pressure),               #
# orientation, FAN LEVEL FLAGS (low_fan, med_fan, high_fan columns), and potentially linear acceleration features.# # <-- Updated Description
# It evaluates the model's performance FOR EACH TARGET using a train-test split and visualizes results          #
# (correlation matrix, RPM & Voltage plots only).                                                               #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler # Removed OneHotEncoder
from sklearn.compose import ColumnTransformer
from sklearn.pipeline import Pipeline
from sklearn.neural_network import MLPRegressor
from sklearn.metrics import mean_squared_error, r2_score
import matplotlib.pyplot as plt
import numpy as np
import joblib
import seaborn as sns
import time
# import re # Removed regex import, no longer needed for filenames

#-----------------------------------------------------------------------------------------------------------------#
# 1. Configuration                                                                                                #
#-----------------------------------------------------------------------------------------------------------------#

# --- Files to Load ---
# Ensure these files contain ALL required columns (inputs including fan flags, and all 4 targets)
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
    'SouthWest_MedFan_150Degree.csv',
    'West_LowFan_120Degree.csv',
    'South_HighFan_180Degree.csv',
    'East_HighFan_270Degree.csv',
    'NorthEast_HighFan_300Degree.csv',
    #i think it needs hot encode of low, med, high fan
    'NorthEast_MedFan_300Degree.csv',
    'NorthEast_LowFan_300Degree.csv',
    'NorthEast_HighFan_315Degree.csv',
    'NorthEast_MedFan_315Degree.csv',
    'NorthEast_LowFan_315Degree.csv',
    'NorthEast_HighFan_330Degree.csv',
    'NorthEast_LowFan_330Degree.csv',
    'NorthEast_MedFan_330Degree.csv',
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
# Linear Acceleration (Optional Input - Uncomment features list below to include)
lin_accel_lx_col = 'linear_acceleration_lx'
lin_accel_ly_col = 'linear_acceleration_ly'
lin_accel_lz_col = 'linear_acceleration_lz'
# Fan Flags (Inputs - Assumed to be columns in the CSV, likely 0/1)
low_fan_col = 'low_fan'     # NEW INPUT
med_fan_col = 'med_fan'     # NEW INPUT
high_fan_col = 'high_fan'   # NEW INPUT

altitude_col = 'weatherstation_altitude'
# Electrical Values (Now Targets)
power_col = 'power_value'           # Target 3
current_col = 'current_value'       # Target 4
# Define TARGET Variable Names
target_rpm_col = 'rpm_value'        # Target 1
target_voltage_col = 'voltage_value'    # Target 2
# Removed fan_level_col

# --- MLP Regressor Hyperparameters ---
hidden_layer_config = (64, 32)
activation_function = 'relu'
solver_algorithm = 'adam'
max_training_iterations = 400
early_stopping_enabled = True
validation_set_fraction = 0.1
iterations_no_improvement = 10

# --- Output Model/Pipeline Filename ---
# Filename constructed dynamically in Section 3

#-----------------------------------------------------------------------------------------------------------------#
# 2. Load Data (SIMPLIFIED - No fan level extraction needed)                                                      #
#-----------------------------------------------------------------------------------------------------------------#
start_load_time = time.time()
if not data_files or all(f.startswith('#') for f in data_files):
      raise ValueError("No data files specified. Please edit the 'data_files' list.")
loaded_data = []
print("Loading data...") # Simplified message

# Removed fan_level_pattern

for file_path in data_files:
    if file_path.startswith('#'): continue
    try:
        df = pd.read_csv(file_path, low_memory=False)
        # No fan level extraction needed from filename
        print(f"  - Successfully loaded {file_path} ({df.shape[0]} rows)")
        loaded_data.append(df)
        # Check if expected fan columns exist in the first loaded file (optional check)
        # if not loaded_data: # Only check first time
        #    expected_fan_cols = [low_fan_col, med_fan_col, high_fan_col]
        #    missing_fan_cols = [col for col in expected_fan_cols if col not in df.columns]
        #    if missing_fan_cols:
        #        print(f"  - WARNING: Expected fan columns {missing_fan_cols} not found in {file_path}. Ensure they exist in all files or script might fail later.")

    except FileNotFoundError: print(f"Error: File not found - {file_path}."); exit()
    except Exception as e: print(f"An error occurred loading {file_path}: {e}"); exit()

if not loaded_data: print("Error: No data could be loaded."); exit()
data = pd.concat(loaded_data, ignore_index=True)
load_time = time.time() - start_load_time
print(f"\nCombined dataset shape: {data.shape}")
print(f"Data loading took {load_time:.2f} seconds.")
# Removed value counts for 'fan_level'

#-----------------------------------------------------------------------------------------------------------------#
# 3. Data Preprocessing & Feature/Target Selection (MODIFIED for fan flag columns)                                #
#-----------------------------------------------------------------------------------------------------------------#
start_preprocess_time = time.time()
print("\nPreprocessing data...")

# --- Define NUMERICAL INPUT Features ---
# These will be scaled. Includes original numerical features AND the new fan flags.
numerical_features = [
    # Weather & Orientation
    weather_speed_col,
    weather_direction_col,   # (will be converted to radians before scaling)
    weather_humidity_col,
    weather_pressure_col,
    orientation_heading_col,
    orientation_roll_col,
    orientation_pitch_col,

    altitude_col,

    # Fan Flags (treated as numerical, likely 0/1)
    low_fan_col,
    med_fan_col,
    high_fan_col,

    # Linear Acceleration Features (Optional - Uncomment to include)
    lin_accel_lx_col,
    lin_accel_ly_col,
    lin_accel_lz_col
]

# --- Removed CATEGORICAL INPUT Features ---
# categorical_features = [fan_level_col] # REMOVED

# --- Define TARGETS (y) ---
targets = [
    target_rpm_col,
    target_voltage_col,
    power_col,        # Target 3
    current_col       # Target 4
]

# --- Determine included feature groups and construct output filename ---
include_lin_accel = any(col in numerical_features for col in [lin_accel_lx_col, lin_accel_ly_col, lin_accel_lz_col])
include_extra_weather = any(col in numerical_features for col in [weather_humidity_col, weather_pressure_col])

# Updated filename to reflect direct fan flags
output_filename_parts = ['rpm_volt_pow_curr_multi_mlp_model', 'fanflags'] # Changed 'fanlevel' to 'fanflags'
if include_extra_weather:
    output_filename_parts.append('weather')
    print("Including extra Weather features (humidity, pressure) as INPUTS.")
else:
    print("Excluding extra Weather features from INPUTS.")
if include_lin_accel:
    output_filename_parts.append('linaccel')
    print("Including Linear Acceleration features as INPUTS.")
else:
    print("Excluding Linear Acceleration features from INPUTS.")

output_model_filename = "_".join(output_filename_parts) + '.pkl'
print(f"Output model filename set to: {output_model_filename}")

# --- Define all columns needed (numerical features + targets) ---
all_required_cols = numerical_features + targets # Simplified, no categorical

# --- Check if ALL required columns exist before selecting ---
print(f"Checking for required columns: {all_required_cols}")
missing_cols = [col for col in all_required_cols if col not in data.columns]
if missing_cols:
    print(f"\nError: The following required columns are missing from the loaded data:")
    for col in missing_cols:
        print(f"  - {col}")
    print(f"\nPlease ensure your CSV files contain these columns OR adjust the feature/target lists.")
    print(f"Available columns in loaded data: {data.columns.tolist()}")
    exit()

# --- Convert Weather Direction input feature to Radians ---
if weather_direction_col in numerical_features:
    print(f"Converting '{weather_direction_col}' from degrees to radians...")
    data[weather_direction_col] = pd.to_numeric(data[weather_direction_col], errors='coerce')
    rows_before_dropna_direction = data.shape[0]
    data.dropna(subset=[weather_direction_col], inplace=True)
    rows_after_dropna_direction = data.shape[0]
    if rows_before_dropna_direction > rows_after_dropna_direction:
        print(f"  Removed {rows_before_dropna_direction - rows_after_dropna_direction} rows due to non-numeric '{weather_direction_col}'.")
    data[weather_direction_col] = np.radians(data[weather_direction_col])
else:
     print(f"Skipping radian conversion as '{weather_direction_col}' is not in selected numerical features.")

# --- Handle Other Missing Values & Ensure Numeric Types (for ALL required columns) ---
initial_rows = data.shape[0]
print(f"Ensuring required columns ({len(all_required_cols)}) are numeric and handling missing values...")
for col in all_required_cols: # Check all required (features + targets)
    if col in data.columns:
        if not pd.api.types.is_numeric_dtype(data[col]):
              print(f"  Converting column '{col}' to numeric.")
              # Be careful coercing potentially non-numeric fan flags if they aren't 0/1
              data[col] = pd.to_numeric(data[col], errors='coerce')

rows_before_final_dropna = data.shape[0]
# Drop rows if *any* required column is missing
data.dropna(subset=all_required_cols, inplace=True)
rows_after_dropna = data.shape[0]

additional_removed = rows_before_final_dropna - rows_after_dropna
if additional_removed > 0:
    print(f"Removed {additional_removed} rows due to missing/non-numeric values in required columns.")

if data.empty:
    print("Error: No data remaining after cleaning.")
    exit()

# --- Define X (features) and y (targets) AFTER cleaning ---
X = data[numerical_features] # X now only contains numerical_features
y = data[targets]
preprocess_time = time.time() - start_preprocess_time
print(f"\nSelected Numerical Features ({len(numerical_features)}): {numerical_features}")
# print(f"Selected Categorical Features ({len(categorical_features)}): {categorical_features}") # Removed
print(f"Selected Targets ({len(targets)}): {targets}")
print(f"Final dataset size for modeling: {X.shape[0]} rows")
print(f"Data preprocessing took {preprocess_time:.2f} seconds.")

# --- Train-Test Split ---
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
print(f"\nData split into training ({X_train.shape[0]} rows) and testing ({X_test.shape[0]} rows) sets.")
print(f"Shape of y_train: {y_train.shape}, Shape of y_test: {y_test.shape}")

#-----------------------------------------------------------------------------------------------------------------#
# 4. Model Training (MLP Regressor Pipeline - SIMPLIFIED Preprocessor)                                            #
#-----------------------------------------------------------------------------------------------------------------#
start_train_time = time.time()
print(f"\nTraining Multi-Output MLP Regressor model (Predicting {len(targets)} targets)...")

# --- Create Preprocessor using ColumnTransformer ---
# This now ONLY applies StandardScaler to the numerical features (which include fan flags)
# If fan flags were strings or needed different treatment, this would need adjustment.
preprocessor = ColumnTransformer(
    transformers=[
        ('num', StandardScaler(), numerical_features),
        # ('cat', OneHotEncoder(handle_unknown='ignore'), categorical_features) # REMOVED
    ],
    remainder='passthrough' # Should not have remainders if X only has numerical_features
)

# --- Create the Full Pipeline ---
# 1. Preprocessor (handles scaling)
# 2. MLP Regressor
model_pipeline = Pipeline([
    ('preprocessor', preprocessor),
    ('mlp_regressor', MLPRegressor(
        hidden_layer_sizes=hidden_layer_config, activation=activation_function, solver=solver_algorithm,
        max_iter=max_training_iterations, early_stopping=early_stopping_enabled,
        validation_fraction=validation_set_fraction, n_iter_no_change=iterations_no_improvement,
        random_state=42, verbose=True
    ))
])

# --- Train the pipeline ---
# The pipeline automatically applies preprocessing steps before training the MLP
model_pipeline.fit(X_train, y_train)

train_time = time.time() - start_train_time
print(f"\nTraining complete. Took {train_time:.2f} seconds.")
joblib.dump(model_pipeline, output_model_filename)
print(f"Trained pipeline saved as {output_model_filename}.")

# --- Get Feature Names After Transformation (for potential debugging/inspection) ---
try:
    # Access the fitted ColumnTransformer to get feature names (will just be numerical features)
    feature_names_out = model_pipeline.named_steps['preprocessor'].get_feature_names_out()
    print(f"\nFeature names after preprocessing (used by MLP):")
    print(feature_names_out)
except Exception as e:
    print(f"\nCould not retrieve feature names after preprocessing: {e}")


#-----------------------------------------------------------------------------------------------------------------#
# 5. Model Prediction & Evaluation (Unchanged - Pipeline handles prediction)                                      #
#-----------------------------------------------------------------------------------------------------------------#
print("\nEvaluating model performance (per target)...")
start_eval_time = time.time()
# Use the pipeline directly for predictions; it handles the preprocessing
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

# Print performance metrics for each target
print(f"\n--- Performance Metrics (Evaluation took {eval_time:.2f} seconds) ---")
for i, target_name in enumerate(targets):
    print(f"--- Target: {target_name} ---")
    print(f"  Training MSE:   {mse_train_raw[i]:.4f}, RMSE: {rmse_train_raw[i]:.4f}, R²: {r2_train_raw[i]:.4f} ({r2_train_raw[i]*100:.2f}%)")
    print(f"  Testing MSE:    {mse_test_raw[i]:.4f}, RMSE: {rmse_test_raw[i]:.4f}, R²: {r2_test_raw[i]:.4f} ({r2_test_raw[i]*100:.2f}%)")

#-----------------------------------------------------------------------------------------------------------------#
# 6. Visualization (MODIFIED - Correlation Matrix includes fan flags)                                           #
#-----------------------------------------------------------------------------------------------------------------#
print("\nGenerating visualizations (Correlation Matrix, RPM & Voltage plots only)...")
start_viz_time = time.time()

# --- Plot 0: Feature and Target Correlation Matrix ---
# Now includes the fan flag columns along with other numerical inputs and targets
print("Calculating and plotting correlation matrix (All Numerical Inputs & Targets)...")

# Select all numerical features (including fan flags) and targets for the correlation matrix
cols_for_corr = numerical_features + targets
correlation_matrix = data[cols_for_corr].corr() # Use the cleaned dataframe

num_corr_cols = len(cols_for_corr)
fig_width = max(10, num_corr_cols * 1.0); fig_height = max(8, num_corr_cols * 0.8) # Adjusted size slightly
plt.figure(figsize=(fig_width, fig_height))
sns.heatmap(correlation_matrix, annot=True, cmap='coolwarm', fmt=".2f", linewidths=.5, annot_kws={"size": 7})
plt.title(f'Correlation Matrix: All Numerical Inputs (incl. Fan Flags) and Targets ({len(targets)} Targets)') # Updated title
plt.xticks(rotation=45, ha='right'); plt.yticks(rotation=0)
plt.tight_layout()
plt.savefig('All_Inputs_Targets_Correlation_Matrix.png', dpi=300, bbox_inches='tight') # Updated filename
print("Saved plot: All_Inputs_Targets_Correlation_Matrix.png")
plt.show()


# --- Create plots ONLY for RPM and Voltage targets ---
# Define which targets to plot
targets_to_plot = [target_rpm_col, target_voltage_col] # Specify targets here

for i, target_name in enumerate(targets):
    # Check if the current target is one we want to plot
    if target_name in targets_to_plot: # ADDED CONDITION
        print(f"\nGenerating plots for target: {target_name}")

        # Extract actual and predicted values for the current target
        y_test_target_actual = y_test[target_name]
        y_train_target_actual = y_train[target_name]
        y_test_target_pred = y_test_pred[:, i]
        y_train_target_pred = y_train_pred[:, i]

        # --- Plot 1: Predicted vs Actual (for this target) ---
        plt.figure(figsize=(8, 6))
        plt.scatter(y_test_target_actual, y_test_target_pred, alpha=0.6, edgecolors='k', s=50, label="Test Data Points")
        try:
            min_val = min(y_test_target_actual.min(), y_test_target_pred.min())
            max_val = max(y_test_target_actual.max(), y_test_target_pred.max())
            plt.plot([min_val, max_val], [min_val, max_val], color='red', linestyle='--', linewidth=2, label="Perfect Fit Line (y=x)")
        except ValueError:
            print(f"Warning: Could not plot y=x line for {target_name}, possibly due to empty data.")
        plt.xlabel(f"Actual {target_name} (Test Set)")
        plt.ylabel(f"Predicted {target_name} (Test Set)")
        plt.title(f"MLP Multi-Output (Fan Flags): Predicted vs Actual - {target_name}") # Updated title
        plt.legend(); plt.grid(True)
        plt.savefig(f'MLP_FanFlags_Predicted_vs_Actual_{target_name}_Test.png', dpi=300, bbox_inches='tight') # Updated filename
        print(f"Saved plot: MLP_FanFlags_Predicted_vs_Actual_{target_name}_Test.png")
        plt.show()

        # --- Plot 2: Residual Plot (REMOVED) ---
        # (Code for residual plot is deleted/commented out)

        # --- Plot 3: Actual vs Predicted over Training Sample Index (for this target) ---
        plt.figure(figsize=(15, 6))
        if isinstance(y_train_target_actual, pd.Series): y_train_values_target = y_train_target_actual.values
        else: y_train_values_target = y_train_target_actual

        y_train_pred_values_target = y_train_target_pred

        if len(y_train_values_target) > 0:
            plot_subset = min(len(y_train_values_target), 500); indices = np.arange(plot_subset)
            plt.plot(indices, y_train_values_target[:plot_subset], label='Actual', marker='.', linestyle='-', alpha=0.7)
            plt.plot(indices, y_train_pred_values_target[:plot_subset], label='Predicted', marker='x', linestyle='--', alpha=0.7)
            plt.title(f'{target_name}: Actual vs Predicted (Training Sample - MLP Multi-Output - Fan Flags)') # Updated title
            plt.xlabel(f'Sample Index (First {plot_subset} Samples)'); plt.ylabel(target_name)
            plt.legend(); plt.grid(True)
            plt.savefig(f'MLP_FanFlags_Actual_vs_Predicted_{target_name}_Train_Index.png', dpi=300, bbox_inches='tight') # Updated filename
            print(f"Saved plot: MLP_FanFlags_Actual_vs_Predicted_{target_name}_Train_Index.png")
            plt.show()
        else:
             print(f"Warning: Skipping Actual vs Predicted plot for {target_name} due to insufficient training data points.")
    else:
        print(f"\nSkipping visualization plots for target: {target_name}")


viz_time = time.time() - start_viz_time
print(f"\nVisualizations generated. Took {viz_time:.2f} seconds.")
total_time = time.time() - start_load_time
print(f"\nScript finished. Total execution time: {total_time:.2f} seconds.")

