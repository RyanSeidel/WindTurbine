import joblib
import pandas as pd
import numpy as np

# Load the saved models
scaler = joblib.load('60BladeModel_gyro_components.pkl')
poly = joblib.load('60BladeModelVibration.pkl')
poly_model = joblib.load('60BladeModel_components_model.pkl')

# Prepare a test dataset (Example: Using the test_data you defined earlier)
speed_values = [5, 5, 5, 5]
test_data = pd.DataFrame({
    'speed_value': speed_values,
    'rpm_value': [1] * len(speed_values),  # Example values, replace as needed
    'blade_60': [1] * len(speed_values),
    'blade_45': [0] * len(speed_values),
    'blade_30': [0] * len(speed_values),
    'linear_acceleration_lx': [0.03] * len(speed_values),
    'linear_acceleration_ly': [0.01] * len(speed_values),
    'linear_acceleration_lz': [-0.02] * len(speed_values),
    'gyroscope_gx': [0.1] * len(speed_values),
    'gyroscope_gy': [0.2] * len(speed_values),
    'gyroscope_gz': [-0.1] * len(speed_values),
    'gravity_grx': [9.8] * len(speed_values),
    'gravity_gry': [0.2] * len(speed_values),
    'gravity_grz': [-0.1] * len(speed_values),
    'magnetometer_mx': [-15.3] * len(speed_values),
    'magnetometer_my': [-14.75] * len(speed_values),
    'magnetometer_mz': [-20.06] * len(speed_values),
    'alignment_0': [1] * len(speed_values),
    'alignment_45': [0] * len(speed_values),
    'accelerometer_ax': [0.3] * len(speed_values),
    'accelerometer_ay': [-0.04] * len(speed_values),
    'accelerometer_az': [9.41] * len(speed_values),
})

# Calculate actual magnitude from test data
test_data['accel_magnitude'] = np.sqrt(
    test_data['accelerometer_ax']**2 +
    test_data['accelerometer_ay']**2 +
    test_data['accelerometer_az']**2
)

# Standardize and transform the test data
test_data_scaled = scaler.transform(test_data.drop(columns=['accel_magnitude']))
test_data_poly = poly.transform(test_data_scaled)

# Make predictions
predictions = poly_model.predict(test_data_poly)

# Calculate residuals and threshold for anomaly detection
residuals = np.abs(predictions.flatten() - test_data['accel_magnitude'].values)
threshold = np.mean(residuals) + 1.5 * np.std(residuals)
anomalies = residuals > threshold

# Display results
predicted_data = pd.DataFrame({
    'Speed Value': test_data['speed_value'],
    'Predicted Acceleration Magnitude': predictions.flatten(),
    'Actual Acceleration Magnitude': test_data['accel_magnitude'],
    'Residuals': residuals,
    'Is Anomaly': anomalies,
})

print(predicted_data)

# Optional: Visualize results
import matplotlib.pyplot as plt

plt.figure(figsize=(8, 6))
plt.plot(test_data['speed_value'], predictions, label="Predicted Acceleration Magnitude", marker='o')
plt.scatter(test_data['speed_value'], predictions, c=anomalies, cmap="coolwarm", label="Anomalies")
plt.xlabel('Speed Value')
plt.ylabel('Acceleration Magnitude')
plt.title('Predicted Acceleration Magnitude vs Speed (with Anomalies)')
plt.legend()
plt.grid(True)
plt.show()
