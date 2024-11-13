import pandas as pd
import joblib

# Load the saved model and scaler
scaler = joblib.load("scaler.pkl")
model = joblib.load("voltage_prediction_model.pkl")

# Define sample input for prediction (changing RPM values)
sample_data = pd.DataFrame({
    'rpm_value': [100, 200, 300, 400],  # Change RPM values here
    'orientation_heading': [0] * 4,
    'orientation_roll': [0] * 4,
    'orientation_pitch': [0] * 4,
    'temperature_value': [25] * 4,
    'magnetometer_mx': [0] * 4,
    'magnetometer_my': [0] * 4,
    'magnetometer_mz': [0] * 4,
    'gyroscope_gx': [0] * 4,
    'gyroscope_gy': [0] * 4,
    'gyroscope_gz': [0] * 4,
    'accelerometer_ax': [0] * 4,
    'accelerometer_ay': [0] * 4,
    'accelerometer_az': [0] * 4,
    'linear_acceleration_lx': [0] * 4,
    'linear_acceleration_ly': [0] * 4,
    'linear_acceleration_lz': [0] * 4,
    'gravity_grx': [0] * 4,
    'gravity_gry': [0] * 4,
    'gravity_grz': [0] * 4,
    'current_value': [0] * 4,
    'power_value': [0] * 4,
    'servo_value': [0] * 4
})

# Scale the sample data with the loaded scaler
sample_data_scaled = scaler.transform(sample_data)

# Make predictions for different RPM values
predictions = model.predict(sample_data_scaled)

# Print the predictions for each RPM value
for rpm, voltage in zip([100, 200, 300, 400], predictions):
    print(f"Predicted Voltage for RPM={rpm}: {voltage}")
