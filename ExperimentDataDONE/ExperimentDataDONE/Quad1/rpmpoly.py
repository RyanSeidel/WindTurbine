#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Wind Turbine Digital Twin - MLP Regressor Model for Predicting RPM                                          #
# By [Wind Turbine Digital Twins]                                                                               #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Description:                                                                                                  #
# This script processes wind turbine data from specified CSV files, trains a Multi-layer Perceptron (MLP)       #
# Regressor model (using scikit-learn) to predict rpm_value based on weather station speed, direction         #
# (converted to radians), orientation, electrical signals (voltage, power, current), and potentially          # # <-- Updated Description
# linear acceleration features. It evaluates the model's performance using a train-test split and visualizes    #
# the results, including a correlation matrix of input features and the target variable.                        #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler # Keep scaler for inputs
from sklearn.pipeline import Pipeline
from sklearn.neural_network import MLPRegressor # <-- Import MLPRegressor
# from sklearn.linear_model import LinearRegression # Replaced by MLPRegressor
# from sklearn.preprocessing import PolynomialFeatures # MLP handles non-linearity
from sklearn.metrics import mean_squared_error, r2_score
import matplotlib.pyplot as plt
import numpy as np
import joblib
import seaborn as sns # <--- IMPORT SEABORN for correlation matrix

#-----------------------------------------------------------------------------------------------------------------#
# 1. Configuration                                                                                                #
#-----------------------------------------------------------------------------------------------------------------#

# --- Files to Load ---
# !!! IMPORTANT !!! Replace or add your actual CSV file names to this list.
# Ensure these files contain ALL the required columns, including the new electrical ones.
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
    'East_HighFan_270Degree.csv', # only trying every 90 angle
    'NorthEast_HighFan_300Degree.csv',
    'NorthEast_HighFan_330Degree.csv', # i think it needs hot encode of low, med, high fan
    'NorthEast_MedFan_300Degree.csv',
    'NorthEast_LowFan_300Degree.csv',
    'NorthEast_HighFan_315Degree.csv',
    'NorthEast_MedFan_315Degree.csv',
    'NorthEast_LowFan_315Degree.csv',
    'NorthEast_HighFan_330Degree.csv',
    'NorthEast_LowFan_330Degree.csv',
]

# --- Feature, Target, and Model Configuration ---
# !!! IMPORTANT !!! Verify these column names match your CSV files.

# --- Define INPUT FEATURES ---
# Weather & Orientation
weather_speed_col = 'weatherstation_speed'
weather_direction_col = 'weatherstation_direction'# (will be converted to radians)
orientation_heading_col = 'orientation_heading'
orientation_roll_col = 'orientation_roll'
orientation_pitch_col = 'orientation_pitch'
# Linear Acceleration (Optional - controlled by commenting/uncommenting in Section 3)
lin_accel_lx_col = 'linear_acceleration_lx'
lin_accel_ly_col = 'linear_acceleration_ly'
lin_accel_lz_col = 'linear_acceleration_lz'
# Electrical Features (NEW - controlled by commenting/uncommenting in Section 3)
voltage_col = 'voltage_value' # <--- NEW FEATURE COLUMN NAME
power_col = 'power_value'     # <--- NEW FEATURE COLUMN NAME
current_col = 'current_value' # <--- NEW FEATURE COLUMN NAME


# --- Define TARGET VARIABLE ---
target_rpm_col = 'rpm_value' # Target is RPM

# --- MLP Regressor Hyperparameters ---
# (Unchanged)
hidden_layer_config = (64, 32)
activation_function = 'relu'
solver_algorithm = 'adam'
max_training_iterations = 500
early_stopping_enabled = True
validation_set_fraction = 0.1
iterations_no_improvement = 10

# --- Output Model/Pipeline Filename ---
# Filename will be constructed dynamically in Section 3 based on included features

#-----------------------------------------------------------------------------------------------------------------#
# 2. Load Data (Assumed unchanged - requires new columns to be present in CSVs)                                   #
#-----------------------------------------------------------------------------------------------------------------#
if not data_files or all(f.startswith('#') for f in data_files):
      raise ValueError("No data files specified. Please edit the 'data_files' list with your CSV file names.")
loaded_data = []
print("Loading data...")
# (Loading loop remains the same)
for file_path in data_files:
    if file_path.startswith('#'): continue
    try:
        # Ensure low_memory=False if columns have mixed types initially, though preprocessing should handle it
        df = pd.read_csv(file_path, low_memory=False)
        loaded_data.append(df)
        print(f"  - Successfully loaded {file_path} ({df.shape[0]} rows)")
    except FileNotFoundError:
        print(f"Error: File not found - {file_path}. Please ensure the file exists and the path is correct."); exit()
    except Exception as e:
        print(f"An error occurred loading {file_path}: {e}"); exit()
if not loaded_data: print("Error: No data could be loaded from the specified files."); exit()
data = pd.concat(loaded_data, ignore_index=True)
print(f"\nCombined dataset shape: {data.shape}")


#-----------------------------------------------------------------------------------------------------------------#
# 3. Data Preprocessing & Feature Selection                                                                       #
#-----------------------------------------------------------------------------------------------------------------#
print("\nPreprocessing data...")

# --- Define Features (X) and Target (y) ---
# Define the list of input feature column names
features = [
    # Weather & Orientation
    weather_speed_col,
    weather_direction_col,   # (will be converted)
    # orientation_heading_col,
    # orientation_roll_col,
    # orientation_pitch_col,

    # Electrical Features (NEW - Uncomment to include)
    voltage_col,
    power_col,
    current_col,

    # Linear Acceleration Features (Optional - Uncomment to include)
    # lin_accel_lx_col,
    # lin_accel_ly_col,
    # lin_accel_lz_col
]
target = target_rpm_col

# --- Determine included feature groups and construct output filename ---
include_lin_accel = any(col in features for col in [lin_accel_lx_col, lin_accel_ly_col, lin_accel_lz_col])
include_electrical = any(col in features for col in [voltage_col, power_col, current_col])

output_filename_parts = ['rpm_mlp_model']
if include_electrical:
    output_filename_parts.append('elec')
    print("Including Electrical features (voltage, power, current).")
else:
    print("Excluding Electrical features.")

if include_lin_accel:
    output_filename_parts.append('linaccel')
    print("Including Linear Acceleration features.")
else:
    print("Excluding Linear Acceleration features.")

output_model_filename = "_".join(output_filename_parts) + '.pkl'
print(f"Output model filename set to: {output_model_filename}")


# --- Define all columns needed (features + target) ---
all_cols = features + [target]

# --- Check if ALL required columns exist before selecting ---
print(f"Checking for required columns: {all_cols}")
missing_cols = [col for col in all_cols if col not in data.columns]
if missing_cols:
    print(f"\nError: The following required columns are missing from the loaded data:")
    print(f"  {missing_cols}")
    print(f"\nPlease ensure your CSV files contain these columns OR comment them out in the 'features' list.")
    print(f"Available columns in loaded data: {data.columns.tolist()}")
    exit()

# --- Convert Weather Direction input feature to Radians ---
if weather_direction_col in features: # Check if it's actually included
    print(f"Converting input feature '{weather_direction_col}' from degrees to radians...")
    data[weather_direction_col] = pd.to_numeric(data[weather_direction_col], errors='coerce')
    rows_before_dropna_direction = data.shape[0]
    data.dropna(subset=[weather_direction_col], inplace=True)
    rows_after_dropna_direction = data.shape[0]
    if rows_before_dropna_direction > rows_after_dropna_direction:
        print(f"  Removed {rows_before_dropna_direction - rows_after_dropna_direction} rows due to non-numeric values in '{weather_direction_col}'.")
    data[weather_direction_col] = np.radians(data[weather_direction_col])
else:
     print(f"Skipping radian conversion as '{weather_direction_col}' is not in the selected features.")

# --- Handle Other Missing Values & Ensure Numeric Types ---
initial_rows = data.shape[0]
print("Ensuring required columns are numeric and handling missing values...")
for col in all_cols: # Use all_cols
    if col in data.columns: # Should always be true after check above, but safe practice
        data[col] = pd.to_numeric(data[col], errors='coerce') # Force numeric, turn errors into NaN

# Recalculate row count before final dropna if direction drop occurred
rows_before_final_dropna = rows_after_dropna_direction if weather_direction_col in features else initial_rows

data.dropna(subset=all_cols, inplace=True) # Drop rows if *any* needed column is NaN after coercion
rows_after_dropna = data.shape[0]

# Report rows dropped by the final dropna step
additional_removed = rows_before_final_dropna - rows_after_dropna
if additional_removed > 0:
    print(f"Removed {additional_removed} rows due to missing/non-numeric values in required columns: {all_cols}")

if data.empty:
    print("Error: No data remaining after handling missing values and ensuring numeric types.")
    exit()

# --- Define X and y AFTER cleaning ---
X = data[features]
y = data[target]
print(f"\nSelected Features ({len(features)}): {features}")
print(f"Selected Target: {target}")
print(f"Final dataset size for modeling: {X.shape[0]} rows")


# --- Train-Test Split ---
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
print(f"Data split into training ({X_train.shape[0]} rows) and testing ({X_test.shape[0]} rows) sets.")

#-----------------------------------------------------------------------------------------------------------------#
# 4. Model Training (MLP Regressor Pipeline) (Unchanged - Adapts Automatically)                                   #
#-----------------------------------------------------------------------------------------------------------------#
print(f"\nTraining MLP Regressor model...")
model_pipeline = Pipeline([
    ('scaler', StandardScaler()),
    ('mlp_regressor', MLPRegressor(
        hidden_layer_sizes=hidden_layer_config, activation=activation_function, solver=solver_algorithm,
        max_iter=max_training_iterations, early_stopping=early_stopping_enabled,
        validation_fraction=validation_set_fraction, n_iter_no_change=iterations_no_improvement,
        random_state=42, verbose=True
    ))
])
model_pipeline.fit(X_train, y_train)
print("Training complete.")
joblib.dump(model_pipeline, output_model_filename)
print(f"Trained pipeline saved as {output_model_filename}.")


#-----------------------------------------------------------------------------------------------------------------#
# 5. Model Prediction & Evaluation (Unchanged - Adapts Automatically)                                             #
#-----------------------------------------------------------------------------------------------------------------#
print("\nEvaluating model performance...")
y_train_pred = model_pipeline.predict(X_train)
y_test_pred = model_pipeline.predict(X_test)
mse_train = mean_squared_error(y_train, y_train_pred); rmse_train = np.sqrt(mse_train); r2_train = r2_score(y_train, y_train_pred)
mse_test = mean_squared_error(y_test, y_test_pred); rmse_test = np.sqrt(mse_test); r2_test = r2_score(y_test, y_test_pred)
accuracy_train = r2_train * 100; accuracy_test = r2_test * 100
print(f"\n--- Performance Metrics (Predicting {target}) ---")
print(f"Training MSE:   {mse_train:.4f}, Training RMSE:   {rmse_train:.4f}, Training R²:   {r2_train:.4f} ({accuracy_train:.2f}%)")
print(f"Testing MSE:    {mse_test:.4f}, Testing RMSE:    {rmse_test:.4f}, Testing R²:    {r2_test:.4f} ({accuracy_test:.2f}%)")


#-----------------------------------------------------------------------------------------------------------------#
# 6. Visualization                                                                                                #
#-----------------------------------------------------------------------------------------------------------------#
print("\nGenerating visualizations...")

# --- Plot 0: Feature and Target Correlation Matrix --- <--- ADJUSTED SIZE
print("Calculating and plotting feature & target correlation matrix...")
correlation_matrix = data[all_cols].corr() # Uses all_cols which now includes new features if added

# Adjust figsize based on the number of features + target
num_corr_cols = len(all_cols)
fig_width = max(8, num_corr_cols * 1.1) # Adjust multiplier as needed
fig_height = max(6, num_corr_cols * 0.9) # Adjust multiplier as needed
plt.figure(figsize=(fig_width, fig_height))

sns.heatmap(correlation_matrix, annot=True, cmap='coolwarm', fmt=".2f", linewidths=.5, annot_kws={"size": 8}) # Smaller font if many features
plt.title(f'Correlation Matrix of Input Features and Target ({target_rpm_col})')
plt.xticks(rotation=45, ha='right')
plt.yticks(rotation=0)
plt.tight_layout()
plt.savefig('Feature_Target_Correlation_Matrix.png', dpi=300, bbox_inches='tight')
print("Saved plot: Feature_Target_Correlation_Matrix.png")
plt.show()


# --- Plot 1: Predicted vs Actual RPM (Test Set) (Unchanged) ---
plt.figure(figsize=(8, 6))
plt.scatter(y_test, y_test_pred, alpha=0.6, edgecolors='k', s=50, label="Test Data Points")
min_val = min(y_test.min(), y_test_pred.min()); max_val = max(y_test.max(), y_test_pred.max())
plt.plot([min_val, max_val], [min_val, max_val], color='red', linestyle='--', linewidth=2, label="Perfect Fit Line (y=x)")
plt.xlabel(f"Actual {target_rpm_col} (Test Set)"); plt.ylabel(f"Predicted {target_rpm_col} (Test Set)")
plt.title(f"MLP Regressor: Predicted vs Actual RPM"); plt.legend(); plt.grid(True)
plt.savefig('MLP_Predicted_vs_Actual_RPM_Test.png', dpi=300, bbox_inches='tight')
print("Saved plot: MLP_Predicted_vs_Actual_RPM_Test.png")
plt.show()


# --- Plot 2: Residual Plot (Test Set) (Unchanged) ---
residuals_test = y_test - y_test_pred
plt.figure(figsize=(10, 6))
plt.scatter(y_test_pred, residuals_test, alpha=0.6, edgecolors='k', s=50, label='Test Set Residuals')
plt.axhline(y=0, color='red', linestyle='--', linewidth=2, label='Zero Error Line')
plt.xlabel(f"Predicted {target_rpm_col} (Test Set)"); plt.ylabel("Residuals (Actual - Predicted)")
plt.title("Residual Plot: Errors vs Predicted RPM on Test Data (MLP)"); plt.legend(); plt.grid(True)
plt.savefig('MLP_Residual_Plot_RPM_Test.png', dpi=300, bbox_inches='tight')
print("Saved plot: MLP_Residual_Plot_RPM_Test.png")
plt.show()

# --- Plot 3: Actual vs Predicted RPM over Training Sample Index (Unchanged) ---
plt.figure(figsize=(15, 6))
if isinstance(y_train, pd.DataFrame): y_train_values = y_train.iloc[:, 0].values
elif isinstance(y_train, pd.Series): y_train_values = y_train.values
else: y_train_values = y_train
if y_train_pred.ndim > 1: y_train_pred_values = y_train_pred[:, 0]
else: y_train_pred_values = y_train_pred
plot_subset = min(len(y_train_values), 500); indices = np.arange(plot_subset)
plt.plot(indices, y_train_values[:plot_subset], label='Actual', marker='.', linestyle='-', alpha=0.7)
plt.plot(indices, y_train_pred_values[:plot_subset], label='Predicted', marker='x', linestyle='--', alpha=0.7)
plt.title(f'{target_rpm_col}: Actual vs Predicted (Training Set Sample)')
plt.xlabel(f'Sample Index (First {plot_subset} Samples)'); plt.ylabel(target_rpm_col)
plt.legend(); plt.grid(True)
plt.savefig('MLP_Actual_vs_Predicted_RPM_Train_Index.png', dpi=300, bbox_inches='tight')
print("Saved plot: MLP_Actual_vs_Predicted_RPM_Train_Index.png")
plt.show()


print("\nScript finished.")