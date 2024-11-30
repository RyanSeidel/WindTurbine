# Predict on Specific Speed Values
speed_values = [0, 5, 10, 15]
test_data = pd.DataFrame({
    'speed_value': speed_values,
    'rpm_value': [X['rpm_value'].mean()] * len(speed_values),
    "blade_60": [1 if blade_angle == 60 else 0],
    "blade_45": [1 if blade_angle == 45 else 0],
    "blade_30": [1 if blade_angle == 30 else 0],
    'linear_acceleration_lx': [X['linear_acceleration_lx'].mean()] * len(speed_values),
    'linear_acceleration_ly': [X['linear_acceleration_ly'].mean()] * len(speed_values),
    'linear_acceleration_lz': [X['linear_acceleration_lz'].mean()] * len(speed_values),
    'gyroscope_gx': [X['gyroscope_gx'].mean()] * len(speed_values),
    'gyroscope_gy': [X['gyroscope_gy'].mean()] * len(speed_values),
    'gyroscope_gz': [X['gyroscope_gz'].mean()] * len(speed_values),
    'gravity_grx': [X['gravity_grx'].mean()] * len(speed_values),
    'gravity_gry': [X['gravity_gry'].mean()] * len(speed_values),
    'gravity_grz': [X['gravity_grz'].mean()] * len(speed_values),
    'magnetometer_mx': [X['magnetometer_mx'].mean()] * len(speed_values),
    'magnetometer_my': [X['magnetometer_my'].mean()] * len(speed_values),
    'magnetometer_mz': [X['magnetometer_mz'].mean()] * len(speed_values),
    'alignment_0': [X['alignment_0'].mean()] * len(speed_values),
    'alignment_45': [X['alignment_45'].mean()] * len(speed_values),
    'accelerometer_ax': [X['accelerometer_ax'].mean()] * len(speed_values),
    'accelerometer_ay': [X['accelerometer_ay'].mean()] * len(speed_values),
    'accelerometer_az': [X['accelerometer_az'].mean()] * len(speed_values),
})


# Standardize and transform the test data
test_data_scaled = scaler.transform(test_data)
test_data_poly = poly.transform(test_data_scaled)

# Predict acceleration magnitude
predictions = poly_model.predict(test_data_poly)

# Calculate actual magnitude from test data
test_data['accel_magnitude'] = np.sqrt(
    test_data['accelerometer_ax']**2 +
    test_data['accelerometer_ay']**2 +
    test_data['accelerometer_az']**2
)

# Calculate residuals for anomalies
new_residuals = np.abs(predictions.flatten() - test_data['accel_magnitude'].values)

# Calculate residuals for the test data
residuals = np.abs(y_test.values.flatten() - y_test_pred.flatten())

# Check for anomalies
threshold = np.mean(residuals) + 3 * np.std(residuals)  # Ensure threshold is defined
anomalies = new_residuals > threshold

# Display predictions with anomaly flags
predicted_data = pd.DataFrame({
    'Predicted Acceleration Magnitude': predictions.flatten(),
    'Speed Value': speed_values,
    'Actual Acceleration Magnitude': test_data['accel_magnitude'].values,
    'Residuals': new_residuals,
    'Is Anomaly': anomalies,
})
print(predicted_data)

# Plot predictions with anomaly markers
plt.figure(figsize=(8, 6))
plt.plot(speed_values, predictions, label="Predicted Acceleration Magnitude", marker='o')
plt.scatter(speed_values, predictions, c=anomalies, cmap="coolwarm", label="Anomalies")
plt.xlabel('Speed Value')
plt.ylabel('Acceleration Magnitude')
plt.title('Predicted Acceleration Magnitude vs Speed (with Anomalies)')
plt.legend()
plt.grid(True)
plt.show()