import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.multioutput import MultiOutputRegressor
from xgboost import XGBRegressor, plot_importance
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import joblib

# Load the CSV
file_name = 'wind_turbine_datasouth.csv'
data = pd.read_csv(file_name)

# Verify column names
print("Dataset columns:", data.columns)

# Convert RPM to RPS
data['rps_value'] = data['rpm_value'] / 60  # Convert RPM to RPS

# Features (X) and Targets (y)
X = data[['speed_value', 'direction_value', 'pressure_value', 'servo_value', 'humidity_value',
          'orientation_heading', 'orientation_roll', 'orientation_pitch', 'temperature_value',
          'rpm_blade_1', 'rpm_blade_2', 'rpm_blade_3', 'current_value', 'power_value', 
          'magnetometer_mx', 'magnetometer_my', 'magnetometer_mz', 'gravity_grx', 
          'gravity_gry', 'gravity_grz', 'altitude_value', 'rpm_value', 'voltage_value']]
y = data[['accelerometer_ax', 'accelerometer_ay', 'accelerometer_az', 
          'gyroscope_gx', 'gyroscope_gy', 'gyroscope_gz', 
          'linear_acceleration_lx', 'linear_acceleration_ly', 'linear_acceleration_lz']]

# Train-test split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Define Multi-Output Model
multi_model = MultiOutputRegressor(XGBRegressor())

# Train the Model
print("Training the model...")
multi_model.fit(X_train, y_train)
print("Model training complete.")

# Predict on Test Data
y_pred = multi_model.predict(X_test)

# Evaluate the Model
# Compute Mean Squared Errors for each target
mse_values = {}
for i, col in enumerate(y.columns):
    mse = mean_squared_error(y_test.iloc[:, i], y_pred[:, i])
    mse_values[col] = mse
    print(f"Mean Squared Error for {col}: {mse}")

# Save the model
joblib.dump(multi_model, 'wind_turbine_model.pkl')
print("Model saved as wind_turbine_model.pkl")

# Plot Feature Importance
# for i, col in enumerate(y.columns):
#     print(f"Plotting feature importance for {col} prediction model:")
#     plot_importance(multi_model.estimators_[i])  # Plot importance for each target
#     plt.title(f"Feature Importance for {col} Prediction")
#     plt.show()
