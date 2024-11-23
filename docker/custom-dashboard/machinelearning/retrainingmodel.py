import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.multioutput import MultiOutputRegressor
from xgboost import XGBRegressor, plot_importance
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import joblib

# List of CSV files for all 8 directions
data_files = [
    'wind_turbine_datasouth.csv',
]

# Load and combine all datasets
data = pd.concat([pd.read_csv(f) for f in data_files], ignore_index=True)
print(f"Combined dataset contains {data.shape[0]} rows.")

# Convert RPM to RPS
data['rps_value'] = data['rpm_value'] / 60  # Convert RPM to RPS

# Features (X) and Targets (y)
# Optimize data types for memory efficiency
X = data[['speed_value', 'direction_value', 'pressure_value', 'servo_value', 'humidity_value',
          'orientation_heading', 'orientation_roll', 'orientation_pitch', 'temperature_value',
          'rpm_blade_1', 'rpm_blade_2', 'rpm_blade_3', 'current_value', 'power_value',
          'magnetometer_mx', 'magnetometer_my', 'magnetometer_mz',
          'gyroscope_gx', 'gyroscope_gy', 'gyroscope_gz', 'accelerometer_ax', 
          'accelerometer_ay', 'accelerometer_az', 'linear_acceleration_lx', 
          'linear_acceleration_ly', 'linear_acceleration_lz', 'gravity_grx', 
          'gravity_gry', 'gravity_grz', 'altitude_value']]

# Downcast to smaller numerical types
X = X.astype({
    'speed_value': 'float32',
    'direction_value': 'float32',
    'pressure_value': 'float32',
    'servo_value': 'int8',  # Assuming values like 0-255
    'humidity_value': 'float32',
    'orientation_heading': 'float32',
    'orientation_roll': 'float32',
    'orientation_pitch': 'float32',
    'temperature_value': 'float32',
    'rpm_blade_1': 'int16',  # Assuming RPM values are within 16-bit range
    'rpm_blade_2': 'int16',
    'rpm_blade_3': 'int16',
    'current_value': 'float32',
    'power_value': 'float32',
    'magnetometer_mx': 'float32',
    'magnetometer_my': 'float32',
    'magnetometer_mz': 'float32',
    'gyroscope_gx': 'float32',
    'gyroscope_gy': 'float32',
    'gyroscope_gz': 'float32',
    'accelerometer_ax': 'float32',
    'accelerometer_ay': 'float32',
    'accelerometer_az': 'float32',
    'linear_acceleration_lx': 'float32',
    'linear_acceleration_ly': 'float32',
    'linear_acceleration_lz': 'float32',
    'gravity_grx': 'float32',
    'gravity_gry': 'float32',
    'gravity_grz': 'float32',
    'altitude_value': 'float32'
})

# Print memory usage before and after optimization
print(f"Memory usage after optimization: {X.memory_usage(deep=True).sum() / 1024**2:.2f} MB")

y = data[['rps_value', 'voltage_value']]  # Use 'rps_value' instead of 'rpm_value'

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
mse_voltage = mean_squared_error(y_test['voltage_value'], y_pred[:, 1])

print(f"Mean Squared Error for RPS: {mse_rps}")
print(f"Mean Squared Error for Voltage: {mse_voltage}")

# Plot Feature Importance
print("Plotting feature importance for RPS prediction model:")
plot_importance(multi_model.estimators_[0])  # Plot importance for the first target (RPS)
plt.title("Feature Importance for RPS Prediction")
plt.show()

print("Plotting feature importance for Voltage prediction model:")
plot_importance(multi_model.estimators_[1])  # Plot importance for the second target (Voltage)
plt.title("Feature Importance for Voltage Prediction")
plt.show()
