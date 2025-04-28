#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Wind Turbine Digital Twin - LightGBM Regressor Model for Predicting RPM, Voltage, Power, Current            #
# By [Wind Turbine Digital Twins]                                                                               #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Description:                                                                                                  #
# This script processes wind turbine data, trains a LightGBM Regressor wrapped in MultiOutputRegressor          # # <-- Updated Description
# for MULTI-OUTPUT regression (using lightgbm library and sklearn) to predict FOUR TARGETS: rpm_value,          # # <-- Updated Description
# voltage_value, power_value, AND current_value based on weather station (speed, direction, humidity, pressure, #
# altitude), orientation, FAN LEVEL FLAGS (low_fan, med_fan, high_fan columns), and potentially linear          #
# acceleration features. It evaluates the model's performance FOR EACH TARGET using a train-test split          #
# and visualizes results (correlation matrix, RPM & Voltage plots only).                                        #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.compose import ColumnTransformer
from sklearn.pipeline import Pipeline
import lightgbm as lgb
from sklearn.multioutput import MultiOutputRegressor # <-- ADDED IMPORT
from sklearn.metrics import mean_squared_error, r2_score
import matplotlib.pyplot as plt
import numpy as np
import joblib
import seaborn as sns
import time

#-----------------------------------------------------------------------------------------------------------------#
# 1. Configuration (Unchanged)                                                                                    #
#-----------------------------------------------------------------------------------------------------------------#

# --- Files to Load ---
data_files = [
    'North_HighFan_0Degree.csv',
    'North_MedFan_0Degree.csv',
    'North_LowFan_0Degree.csv',
    'North_ZeroFan_0Degree.csv', # Ensure this has appropriate 0/1 for fan flags
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
weather_speed_col = 'weatherstation_speed'
weather_direction_col = 'weatherstation_direction'
weather_humidity_col = 'weatherstation_humidity'
weather_pressure_col = 'weatherstation_pressure'
altitude_col = 'weatherstation_altitude'
orientation_heading_col = 'orientation_heading'
orientation_roll_col = 'orientation_roll'
orientation_pitch_col = 'orientation_pitch'
lin_accel_lx_col = 'linear_acceleration_lx'
lin_accel_ly_col = 'linear_acceleration_ly'
lin_accel_lz_col = 'linear_acceleration_lz'
low_fan_col = 'low_fan'
med_fan_col = 'med_fan'
high_fan_col = 'high_fan'
power_col = 'power_value'
current_col = 'current_value'
target_rpm_col = 'rpm_value'
target_voltage_col = 'voltage_value'

# --- LightGBM Regressor Hyperparameters ---
lgbm_n_estimators = 100
lgbm_learning_rate = 0.1
lgbm_num_leaves = 31
lgbm_max_depth = -1

# --- Output Model/Pipeline Filename ---
# Filename constructed dynamically in Section 3

#-----------------------------------------------------------------------------------------------------------------#
# 2. Load Data (Unchanged)                                                                                        #
#-----------------------------------------------------------------------------------------------------------------#
start_load_time = time.time()
if not data_files or all(f.startswith('#') for f in data_files):
      raise ValueError("No data files specified. Please edit the 'data_files' list.")
loaded_data = []
print("Loading data...")
for file_path in data_files:
    if file_path.startswith('#'): continue
    try:
        df = pd.read_csv(file_path, low_memory=False)
        print(f"  - Successfully loaded {file_path} ({df.shape[0]} rows)")
        loaded_data.append(df)
    except FileNotFoundError: print(f"Error: File not found - {file_path}."); exit()
    except Exception as e: print(f"An error occurred loading {file_path}: {e}"); exit()
if not loaded_data: print("Error: No data could be loaded."); exit()
data = pd.concat(loaded_data, ignore_index=True)
load_time = time.time() - start_load_time
print(f"\nCombined dataset shape: {data.shape}")
print(f"Data loading took {load_time:.2f} seconds.")

#-----------------------------------------------------------------------------------------------------------------#
# 3. Data Preprocessing & Feature/Target Selection (Unchanged Logic, Filename consistent)                         #
#-----------------------------------------------------------------------------------------------------------------#
start_preprocess_time = time.time()
print("\nPreprocessing data...")
numerical_features = [
    weather_speed_col, weather_direction_col, weather_humidity_col, weather_pressure_col,
    altitude_col, orientation_heading_col, orientation_roll_col, orientation_pitch_col,
    low_fan_col, med_fan_col, high_fan_col,
    lin_accel_lx_col, lin_accel_ly_col, lin_accel_lz_col # Optional
]
targets = [target_rpm_col, target_voltage_col, power_col, current_col]
include_lin_accel = any(col in numerical_features for col in [lin_accel_lx_col, lin_accel_ly_col, lin_accel_lz_col])
include_extra_weather = any(col in numerical_features for col in [weather_humidity_col, weather_pressure_col, altitude_col])

output_filename_parts = ['rpm_volt_pow_curr_multi_lgbm_model', 'fanflags']
if include_extra_weather: output_filename_parts.append('weather')
if include_lin_accel: output_filename_parts.append('linaccel')
output_model_filename = "_".join(output_filename_parts) + '.pkl'
print(f"Output model filename set to: {output_model_filename}")

all_required_cols = numerical_features + targets
print(f"Checking for required columns: {all_required_cols}")
missing_cols = [col for col in all_required_cols if col not in data.columns]
if missing_cols:
    print(f"\nError: Missing columns: {missing_cols}")
    print(f"Available columns: {data.columns.tolist()}")
    exit()

if weather_direction_col in numerical_features:
    print(f"Converting '{weather_direction_col}' from degrees to radians...")
    data[weather_direction_col] = pd.to_numeric(data[weather_direction_col], errors='coerce')
    rows_before_dropna_direction = data.shape[0]
    data.dropna(subset=[weather_direction_col], inplace=True)
    rows_after_dropna_direction = data.shape[0]
    if rows_before_dropna_direction > rows_after_dropna_direction:
        print(f"  Removed {rows_before_dropna_direction - rows_after_dropna_direction} rows due to non-numeric '{weather_direction_col}'.")
    data[weather_direction_col] = np.radians(data[weather_direction_col])

print(f"Ensuring required columns ({len(all_required_cols)}) are numeric and handling missing values...")
for col in all_required_cols:
    if col in data.columns:
        if not pd.api.types.is_numeric_dtype(data[col]):
              print(f"  Converting column '{col}' to numeric.")
              data[col] = pd.to_numeric(data[col], errors='coerce')
rows_before_final_dropna = data.shape[0]
data.dropna(subset=all_required_cols, inplace=True)
rows_after_dropna = data.shape[0]
additional_removed = rows_before_final_dropna - rows_after_dropna
if additional_removed > 0: print(f"Removed {additional_removed} rows due to missing/non-numeric values.")
if data.empty: print("Error: No data remaining after cleaning."); exit()

X = data[numerical_features]
y = data[targets]
preprocess_time = time.time() - start_preprocess_time
print(f"\nSelected Numerical Features ({len(numerical_features)}): {numerical_features}")
print(f"Selected Targets ({len(targets)}): {targets}")
print(f"Final dataset size for modeling: {X.shape[0]} rows")
print(f"Data preprocessing took {preprocess_time:.2f} seconds.")

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
print(f"\nData split into training ({X_train.shape[0]} rows) and testing ({X_test.shape[0]} rows) sets.")
print(f"Shape of y_train: {y_train.shape}, Shape of y_test: {y_test.shape}")

#-----------------------------------------------------------------------------------------------------------------#
# 4. Model Training (CHANGED to use MultiOutputRegressor with LightGBM)                                           #
#-----------------------------------------------------------------------------------------------------------------#
start_train_time = time.time()
print(f"\nTraining Multi-Output LightGBM Regressor model (Predicting {len(targets)} targets)...")

# --- Create Preprocessor (Unchanged) ---
preprocessor = ColumnTransformer(
    transformers=[('num', StandardScaler(), numerical_features)],
    remainder='passthrough'
)

# --- Create the base LGBM Regressor ---
base_lgbm = lgb.LGBMRegressor(
        n_estimators=lgbm_n_estimators,
        learning_rate=lgbm_learning_rate,
        num_leaves=lgbm_num_leaves,
        max_depth=lgbm_max_depth,
        random_state=42,
        n_jobs=-1,
        # verbose=-1 # Suppress LightGBM verbosity if desired
    )

# --- Create the Full Pipeline with MultiOutputRegressor wrapping LGBMRegressor ---
# 1. Preprocessor (handles scaling)
# 2. MultiOutputRegressor wrapping LightGBM Regressor
model_pipeline = Pipeline([
    ('preprocessor', preprocessor),
    # Wrap the base LGBM model with MultiOutputRegressor
    ('multioutput_lgbm', MultiOutputRegressor(base_lgbm, n_jobs=-1)) # Use n_jobs=-1 for parallel fitting of models per target
])

# --- Train the pipeline ---
# MultiOutputRegressor will fit one LGBM model per target column in y_train
model_pipeline.fit(X_train, y_train)

train_time = time.time() - start_train_time
print(f"\nTraining complete. Took {train_time:.2f} seconds.")
joblib.dump(model_pipeline, output_model_filename)
print(f"Trained pipeline saved as {output_model_filename}.")

# --- Get Feature Names After Transformation (Unchanged) ---
try:
    feature_names_out = model_pipeline.named_steps['preprocessor'].get_feature_names_out()
    print(f"\nFeature names after preprocessing (used by LGBM):")
    print(feature_names_out)
except Exception as e:
    print(f"\nCould not retrieve feature names after preprocessing: {e}")

# --- Optional: Feature Importances from LightGBM (Average across models) ---
# Note: With MultiOutputRegressor, we have one model per target.
# We can average the importances or look at them individually.
try:
    # Access the MultiOutputRegressor which contains individual estimators
    multioutput_model = model_pipeline.named_steps['multioutput_lgbm']
    all_importances = []
    for i, estimator in enumerate(multioutput_model.estimators_):
        print(f"\n--- Feature Importances (from LightGBM - Target: {targets[i]}) ---")
        importances = estimator.feature_importances_
        all_importances.append(importances)
        feature_importance_df = pd.DataFrame({'Feature': feature_names_out, 'Importance': importances})
        feature_importance_df = feature_importance_df.sort_values(by='Importance', ascending=False)
        print(feature_importance_df.to_string(index=False))

    # Calculate and plot average importances
    if all_importances:
        avg_importances = np.mean(all_importances, axis=0)
        avg_feature_importance_df = pd.DataFrame({'Feature': feature_names_out, 'Importance': avg_importances})
        avg_feature_importance_df = avg_feature_importance_df.sort_values(by='Importance', ascending=False)
        print("\n--- Average Feature Importances (LightGBM across all targets) ---")
        print(avg_feature_importance_df.to_string(index=False))

        plt.figure(figsize=(10, max(6, len(feature_names_out) * 0.4)))
        sns.barplot(x='Importance', y='Feature', data=avg_feature_importance_df)
        plt.title('LightGBM Average Feature Importances (Across Targets)') # Updated title
        plt.tight_layout()
        plt.savefig('LGBM_Avg_Feature_Importances.png', dpi=300, bbox_inches='tight') # Updated filename
        print("\nSaved plot: LGBM_Avg_Feature_Importances.png")
        plt.show()

except Exception as e:
    print(f"\nCould not retrieve or plot feature importances: {e}")


#-----------------------------------------------------------------------------------------------------------------#
# 5. Model Prediction & Evaluation (Unchanged Logic)                                                              #
#-----------------------------------------------------------------------------------------------------------------#
print("\nEvaluating model performance (per target)...")
start_eval_time = time.time()
y_train_pred = model_pipeline.predict(X_train)
y_test_pred = model_pipeline.predict(X_test)
mse_train_raw = mean_squared_error(y_train, y_train_pred, multioutput='raw_values')
mse_test_raw = mean_squared_error(y_test, y_test_pred, multioutput='raw_values')
rmse_train_raw = np.sqrt(mse_train_raw)
rmse_test_raw = np.sqrt(mse_test_raw)
r2_train_raw = r2_score(y_train, y_train_pred, multioutput='raw_values')
r2_test_raw = r2_score(y_test, y_test_pred, multioutput='raw_values')
eval_time = time.time() - start_eval_time
print(f"\n--- Performance Metrics (Evaluation took {eval_time:.2f} seconds) ---")
for i, target_name in enumerate(targets):
    print(f"--- Target: {target_name} ---")
    print(f"  Training MSE:   {mse_train_raw[i]:.4f}, RMSE: {rmse_train_raw[i]:.4f}, R²: {r2_train_raw[i]:.4f} ({r2_train_raw[i]*100:.2f}%)")
    print(f"  Testing MSE:    {mse_test_raw[i]:.4f}, RMSE: {rmse_test_raw[i]:.4f}, R²: {r2_test_raw[i]:.4f} ({r2_test_raw[i]*100:.2f}%)")

#-----------------------------------------------------------------------------------------------------------------#
# 6. Visualization (Unchanged Logic, Titles/Filenames updated)                                                    #
#-----------------------------------------------------------------------------------------------------------------#
print("\nGenerating visualizations (Correlation Matrix, RPM & Voltage plots only)...")
start_viz_time = time.time()
print("Calculating and plotting correlation matrix (All Numerical Inputs & Targets)...")
cols_for_corr = numerical_features + targets
correlation_matrix = data[cols_for_corr].corr()
num_corr_cols = len(cols_for_corr)
fig_width = max(10, num_corr_cols * 1.0); fig_height = max(8, num_corr_cols * 0.8)
plt.figure(figsize=(fig_width, fig_height))
sns.heatmap(correlation_matrix, annot=True, cmap='coolwarm', fmt=".2f", linewidths=.5, annot_kws={"size": 7})
plt.title(f'Correlation Matrix: All Numerical Inputs (incl. Fan Flags) and Targets ({len(targets)} Targets)')
plt.xticks(rotation=45, ha='right'); plt.yticks(rotation=0)
plt.tight_layout()
plt.savefig('All_Inputs_Targets_Correlation_Matrix.png', dpi=300, bbox_inches='tight')
print("Saved plot: All_Inputs_Targets_Correlation_Matrix.png")
plt.show()

targets_to_plot = [target_rpm_col, target_voltage_col]
for i, target_name in enumerate(targets):
    if target_name in targets_to_plot:
        print(f"\nGenerating plots for target: {target_name}")
        y_test_target_actual = y_test[target_name]
        y_train_target_actual = y_train[target_name]
        y_test_target_pred = y_test_pred[:, i]
        y_train_target_pred = y_train_pred[:, i]

        plt.figure(figsize=(8, 6))
        plt.scatter(y_test_target_actual, y_test_target_pred, alpha=0.6, edgecolors='k', s=50, label="Test Data Points")
        try:
            min_val = min(y_test_target_actual.min(), y_test_target_pred.min())
            max_val = max(y_test_target_actual.max(), y_test_target_pred.max())
            plt.plot([min_val, max_val], [min_val, max_val], color='red', linestyle='--', linewidth=2, label="Perfect Fit Line (y=x)")
        except ValueError: print(f"Warning: Could not plot y=x line for {target_name}, possibly due to empty data.")
        plt.xlabel(f"Actual {target_name} (Test Set)")
        plt.ylabel(f"Predicted {target_name} (Test Set)")
        plt.title(f"LightGBM Multi-Output (Fan Flags): Predicted vs Actual - {target_name}")
        plt.legend(); plt.grid(True)
        plt.savefig(f'LGBM_FanFlags_Predicted_vs_Actual_{target_name}_Test.png', dpi=300, bbox_inches='tight')
        print(f"Saved plot: LGBM_FanFlags_Predicted_vs_Actual_{target_name}_Test.png")
        plt.show()

        plt.figure(figsize=(15, 6))
        if isinstance(y_train_target_actual, pd.Series): y_train_values_target = y_train_target_actual.values
        else: y_train_values_target = y_train_target_actual
        y_train_pred_values_target = y_train_target_pred
        if len(y_train_values_target) > 0:
            plot_subset = min(len(y_train_values_target), 500); indices = np.arange(plot_subset)
            plt.plot(indices, y_train_values_target[:plot_subset], label='Actual', marker='.', linestyle='-', alpha=0.7)
            plt.plot(indices, y_train_pred_values_target[:plot_subset], label='Predicted', marker='x', linestyle='--', alpha=0.7)
            plt.title(f'{target_name}: Actual vs Predicted (Training Sample - LightGBM - Fan Flags)')
            plt.xlabel(f'Sample Index (First {plot_subset} Samples)'); plt.ylabel(target_name)
            plt.legend(); plt.grid(True)
            plt.savefig(f'LGBM_FanFlags_Actual_vs_Predicted_{target_name}_Train_Index.png', dpi=300, bbox_inches='tight')
            print(f"Saved plot: LGBM_FanFlags_Actual_vs_Predicted_{target_name}_Train_Index.png")
            plt.show()
        else: print(f"Warning: Skipping Actual vs Predicted plot for {target_name} due to insufficient training data points.")
    else: print(f"\nSkipping visualization plots for target: {target_name}")

viz_time = time.time() - start_viz_time
print(f"\nVisualizations generated. Took {viz_time:.2f} seconds.")
total_time = time.time() - start_load_time
print(f"\nScript finished. Total execution time: {total_time:.2f} seconds.")
