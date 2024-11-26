import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.multioutput import MultiOutputRegressor
from xgboost import XGBRegressor, plot_importance
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import joblib


# This is only for RPM and Voltage 

# List of CSV files for all 8 directions
data_files = [
    # North Directions!
    'wind_turbine_North_Zero_Degree_HighFan.csv',
    'wind_turbine_North_Zero_Degree_LowFan.csv',
    'wind_turbine_North_Zero_Degree_NoFan.csv',
#     'wind_turbine_North_45_Degree_HighFan.csv',
#     'wind_turbine_North_45_Degree_LowFan.csv',
#     'wind_turbine_North_45_Degree_NoFan.csv',
#     'wind_turbine_North_90_Degree_HighFan.csv',
#     'wind_turbine_North_90_Degree_LowFan.csv',
#     'wind_turbine_North_90_Degree_NoFan.csv',
    
#     #  North Blades Alternate
#     # 'wind_turbine_North_1Blade45_HighFan.csv',
#     # 'wind_turbine_North_1Blade45_LowFan.csv',
#     # 'wind_turbine_North_1Blade45_Turn45_HighFan.csv',
#     # 'wind_turbine_North_1Blade45_Turn45_LowFan.csv',
#     # 'wind_turbine_North_2Blade45_HighFan.csv',
#     # 'wind_turbine_North_2Blade45_LowFan.csv',
#     # 'wind_turbine_North_2Blade45_Turn45_HighFan.csv',
#     # 'wind_turbine_North_2Blade45_Turn45_LowFan.csv',
#     # 'wind_turbine_North_3Blade45_Turn45_HighFan.csv',
#     # 'wind_turbine_North_3Blade45_Turn45_LowFan.csv',
#     # 'wind_turbine_North_Blade45_HighFan.csv',
#     # 'wind_turbine_North_Blade45_LowFan.csv',
     
#     # West
    'wind_turbine_West_Zero_Degree_NoFan.csv',
    'wind_turbine_West_Zero_Degree_LowFan.csv',
    'wind_turbine_West_Zero_Degree_HighFan.csv',
    
#     # East
    'wind_turbine_East_Zero_Degree_HighFan.csv',
    'wind_turbine_East_Zero_Degree_LowFan.csv',
    'wind_turbine_East_Zero_Degree_NoFan.csv',
    
#     # North West
    'wind_turbine_NorthWest_Zero_Degree_NoFan.csv',
    'wind_turbine_NorthWest_Zero_Degree_HighFan.csv',
    'wind_turbine_NorthWest_Zero_Degree_LowFan.csv',
    
#     #South
    'wind_turbine_South_Zero_Degree_LowFan.csv',
    'wind_turbine_South_Zero_Degree_HighFan.csv',
    'wind_turbine_South_Zero_Degree_NoFan.csv',
#     # 'wind_turbine_South_45_Degree_LowFan.csv',
#     # 'wind_turbine_South_45_Degree_HighFan.csv',
#     # 'wind_turbine_South_90_Degree_HighFan.csv',
#     # 'wind_turbine_South_90_Degree_LowFan.csv',
    
#     #SouthEast
    'wind_turbine_SouthEast_Zero_Degree_NoFan.csv',
    'wind_turbine_SouthEast_Zero_Degree_HighFan.csv',
    'wind_turbine_SouthEast_Zero_Degree_LowFan.csv',
#     # 'wind_turbine_SouthEast_45_Degree_LowFan.csv',
#     # 'wind_turbine_SouthEast_90_Degree_HighFan.csv',
    
#     #SouthWest
    'wind_turbine_SouthWest_Zero_Degree_NoFan.csv',
    'wind_turbine_SouthWest_Zero_Degree_LowFan.csv',
    'wind_turbine_SouthWest_Zero_Degree_HighFan1.csv',
    
#     #NorthEast 
    'wind_turbine_NorthEast_Zero_Degree_HighFan.csv',
    'wind_turbine_NorthEast_Zero_Degree_LowFan.csv',
    'wind_turbine_NorthEast_Zero_Degree_NoFan.csv'
]

# Load and combine all datasets
data = pd.concat([pd.read_csv(f) for f in data_files], ignore_index=True)
print(f"Combined dataset contains {data.shape[0]} rows.")



# Convert RPM to RPS
data['rps_value'] = data['rpm_value'] / 60  # Convert RPM to RPS

# Features (X) and Targets (y)
# Optimize data types for memory efficiency
X = data[['speed_value', 'servo_value', 'direction_value', 'pressure_value', 'humidity_value',
          'orientation_heading', 'orientation_roll', 'orientation_pitch', 
          'rpm_blade_1', 'rpm_blade_2', 'rpm_blade_3']]

# Downcast to smaller numerical types
X = X.astype({
    'speed_value': 'float16',
    'servo_value': 'float32',
    'direction_value': 'int16',
    'pressure_value': 'float32',
    'servo_value': 'int8',  # Assuming values like 0-255
    'humidity_value': 'float32',
    'orientation_heading': 'float16',
    'orientation_roll': 'float16',
    'orientation_pitch': 'float16',
    'rpm_blade_1': 'int16',  # Assuming RPM values are within 16-bit range
    'rpm_blade_2': 'int16',
    'rpm_blade_3': 'int16'
})

# Print memory usage before and after optimization
print(f"Memory usage after optimization: {X.memory_usage(deep=True).sum() / 1024**2:.2f} MB")

y = data[['rps_value']]  # Use 'rps_value' instead of 'rpm_value'

# Train-test split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Attempt to load the existing model
try:
    multi_model = joblib.load('wind_turbine_model.pkl')
    print("Loaded saved model.")
    # Retrain the model with the combined dataset
    print("Retraining the model with combined data...")
    multi_model.fit(X_train, y_train)
    print("Retraining complete.")
except FileNotFoundError:
    print("No saved model found. Training a new model...")
    multi_model = MultiOutputRegressor(XGBRegressor())
    multi_model.fit(X_train, y_train)
    print("Training complete.")

# Save Updated Model
joblib.dump(multi_model, 'wind_turbine_model.pkl')
print("Updated model saved as wind_turbine_model.pkl.")

# Predict on Test Data
y_pred = multi_model.predict(X_test)

# Evaluate the Model
mse_rps = mean_squared_error(y_test['rps_value'], y_pred[:, 0])

print(f"Mean Squared Error for RPS: {mse_rps}")


# Plot Feature Importance
print("Plotting feature importance for RPS prediction model:")
plot_importance(multi_model.estimators_[0])  # Plot importance for the first target (RPS)
plt.title("Feature Importance for RPS Prediction")
plt.show()
