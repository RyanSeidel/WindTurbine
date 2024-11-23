import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.multioutput import MultiOutputRegressor
from xgboost import XGBRegressor, plot_importance
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import joblib

# THIS IS IMPORTANT BC IT GETS YOU STARTED CREATING THE MODEL

# Load the CSV
file_name = 'wind_turbine_datasouth.csv'
data = pd.read_csv(file_name)

# Verify column names
print(data.columns)

# Convert RPM to RPS
data['rps_value'] = data['rpm_value'] / 60  # Convert RPM to RPS

# Features (X) and Targets (y)
X = data[['speed_value', 'direction_value', 'pressure_value', 'servo_value', 'humidity_value',
          'orientation_heading', 'orientation_roll', 'orientation_pitch', 'temperature_value',
          'rpm_blade_1', 'rpm_blade_2', 'rpm_blade_3', 'current_value', 'power_value','magnetometer_mx', 'magnetometer_my', 'magnetometer_mz',
          'gyroscope_gx', 'gyroscope_gy', 'gyroscope_gz', 'accelerometer_ax',	'accelerometer_ay',	'accelerometer_az', 'linear_acceleration_lx', 'linear_acceleration_ly',	'linear_acceleration_lz', 'gravity_grx', 'gravity_gry', 'gravity_grz', 'altitude_value'
]]
y = data[['rps_value', 'voltage_value']]  # Use 'rps_value' instead of 'rpm_value'

# Train-test split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Define Multi-Output Model
multi_model = MultiOutputRegressor(XGBRegressor())

# Train the Model
multi_model.fit(X_train, y_train)

# Predict on Test Data
y_pred = multi_model.predict(X_test)

# Evaluate the Model
mse_rps = mean_squared_error(y_test['rps_value'], y_pred[:, 0])
mse_voltage = mean_squared_error(y_test['voltage_value'], y_pred[:, 1])

print(f"Mean Squared Error for RPS: {mse_rps}")
print(f"Mean Squared Error for Voltage: {mse_voltage}")

# Save the model
joblib.dump(multi_model, 'wind_turbine_model.pkl')
print("Model saved as wind_turbine_model.pkl")

# Plot Feature Importance
# Feature importance is available for each estimator (one for each target)
print("Plotting feature importance for RPS prediction model:")
plot_importance(multi_model.estimators_[0])  # Plot importance for the first target (RPS)
plt.title("Feature Importance for RPS Prediction")
plt.show()

print("Plotting feature importance for Voltage prediction model:")
plot_importance(multi_model.estimators_[1])  # Plot importance for the second target (Voltage)
plt.title("Feature Importance for Voltage Prediction")
plt.show()
