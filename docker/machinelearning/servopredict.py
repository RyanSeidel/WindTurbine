import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.multioutput import MultiOutputRegressor
from xgboost import XGBRegressor, plot_importance
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import joblib

# This is for servo value prediction (with wind direction and RPM correlation)

# List of CSV files for all 8 directions (same as before)
data_files = [
    'wind_turbine_North_Zero_Degree_HighFan.csv',
    'wind_turbine_North_Zero_Degree_LowFan.csv',
    'wind_turbine_North_Zero_Degree_NoFan.csv',
    'wind_turbine_West_Zero_Degree_NoFan.csv',
    'wind_turbine_West_Zero_Degree_LowFan.csv',
    'wind_turbine_West_Zero_Degree_HighFan.csv',
    'wind_turbine_East_Zero_Degree_HighFan.csv',
    'wind_turbine_East_Zero_Degree_LowFan.csv',
    'wind_turbine_East_Zero_Degree_NoFan.csv',
    'wind_turbine_NorthWest_45_Degree_HighFan.csv',
    'wind_turbine_NorthWest_45_Degree_LowFan.csv',
    'wind_turbine_NorthWest_Zero_Degree_NoFan.csv',
    'wind_turbine_South_Zero_Degree_LowFan.csv',
    'wind_turbine_South_Zero_Degree_HighFan.csv',
    'wind_turbine_South_Zero_Degree_NoFan.csv',
    'wind_turbine_SouthEast_Zero_Degree_NoFan.csv',
    'wind_turbine_SouthEast_Zero_Degree_HighFan.csv',
    'wind_turbine_SouthEast_Zero_Degree_LowFan.csv',
    'wind_turbine_SouthWest_Zero_Degree_NoFan.csv',
    'wind_turbine_SouthWest_Zero_Degree_LowFan.csv',
    'wind_turbine_SouthWest_Zero_Degree_HighFan1.csv',
    'wind_turbine_NorthEast_Zero_Degree_HighFan.csv',
    'wind_turbine_NorthEast_Zero_Degree_LowFan.csv',
    'wind_turbine_NorthEast_Zero_Degree_NoFan.csv'
]

# Load and combine all datasets
data = pd.concat([pd.read_csv(f) for f in data_files], ignore_index=True)
print(f"Combined dataset contains {data.shape[0]} rows.")

# Convert RPM to RPS (for potentially improved feature correlation)
data['rps_value'] = data['rpm_value'] / 60  # Convert RPM to RPS

# Features (X) and Target (y)
X = data[['speed_value', 'servo_value', 'direction_value', 'pressure_value', 'humidity_value',
          'orientation_heading', 'orientation_roll', 'orientation_pitch', 
          'rpm_blade_1', 'rpm_blade_2', 'rpm_blade_3', 'rpm_value']]  # Include rpm_value as a feature

# Optimize data types for memory efficiency
X = X.astype({
    'speed_value': 'float16',
    'servo_value': 'float32',  # servo_value is now a target
    'direction_value': 'int16',
    'pressure_value': 'float32',
    'humidity_value': 'float32',
    'orientation_heading': 'float16',
    'orientation_roll': 'float16',
    'orientation_pitch': 'float16',
    'rpm_blade_1': 'int16',
    'rpm_blade_2': 'int16',
    'rpm_blade_3': 'int16',
    'rpm_value': 'int16'  # Include RPM value as an input feature
})

# Target variable: servo_value (we want to predict servo)
y = data[['servo_value']]

# Train-test split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Attempt to load the existing model
try:
    multi_model = joblib.load('wind_turbine_model_servo.pkl')
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
joblib.dump(multi_model, 'wind_turbine_model_servo.pkl')
print("Updated model saved as wind_turbine_model_servo.pkl.")

# Predict on Test Data
y_pred = multi_model.predict(X_test)

# Evaluate the Model with MSE
mse_servo = mean_squared_error(y_test['servo_value'], y_pred[:, 0])
print(f"Mean Squared Error for Servo Value: {mse_servo}")

# Plot Feature Importance
print("Plotting feature importance for Servo Value prediction model:")
plot_importance(multi_model.estimators_[0])  # Plot importance for the servo value
plt.title("Feature Importance for Servo Value Prediction")
plt.show()

# Check predictions for Wind Direction 1 (North) to ensure servo is predicted as 0
north_data = X_test[X_test['direction_value'] == 1]  # Filter data for North (direction_value = 1)
north_predictions = multi_model.predict(north_data)

# Check predictions when RPM is 50 (to ensure the model handles specific RPM values)
rpm_50_data = X_test[X_test['rpm_value'] == 50]  # Filter data for RPM 50
rpm_50_predictions = multi_model.predict(rpm_50_data)

print(f"Predictions for North direction (servo_value should be 0):")
print(north_predictions[:10])  # Display the first 10 predictions for North direction

print(f"Predictions for RPM 50 (servo_value predictions at RPM 50):")
print(rpm_50_predictions[:10])  # Display the first 10 predictions for RPM = 50
