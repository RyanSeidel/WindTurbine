import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import PolynomialFeatures, StandardScaler
from sklearn.multioutput import MultiOutputRegressor
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score
import matplotlib.pyplot as plt
import joblib

# Load dataset
data_files = [
    
    # 97 Percent
    'wind_60_HighFanVib.csv',
    'wind_60_NoFanVib.csv',
    'wind_60_2HighFanVib.csv',
    'wind_60_45Degree_1HighFanVib.csv',
    'wind_60_45Degree_2HighFanVib.csv',
    'wind_60_45DegreeNoFanVib.csv'
    
    # 97 Percent
    # 'wind_45_1HighFanVib.csv',
    # 'wind_45_2HighFanVib.csv',
    # 'wind_45_45Degree_1HighFanVib.csv',
    # 'wind_45_45Degree_2HighFanVib.csv',
    # 'wind_45_NoFanVib.csv'
            
    # 88 Percent
    # 'wind_30_1HighFanVib.csv',
    # 'wind_30_2HighFanVib.csv',
    # 'wind_30_45Degree_1HighFanVib.csv',
    # 'wind_30_45Degree_2HighFanVib.csv',
    # 'wind_30_NoFanVib.csv'
    
    # 96 Percent Accuracy
    # 'wind_15_HighFanVib.csv',
    # 'wind_15_2HighFanVib.csv',
    # 'wind_15_45Degree_1HighFanVib.csv',
    # 'wind_15_45Degree_2HighFanVib.csv',
    # 'wind_15_NoFanVib.csv'
       

]
data = pd.concat([pd.read_csv(f) for f in data_files], ignore_index=True)
print(f"Combined dataset contains {data.shape[0]} rows.")

# Add magnitude as a new column (if not already added)
data['accel_magnitude'] = np.sqrt(
    data['accelerometer_ax']**2 +
    data['accelerometer_ay']**2 +
    data['accelerometer_az']**2
)


# Features (excluding magnitude)
# Features (excluding `rpm_value` and magnitude)
X = data[['speed_value',
          'linear_acceleration_lx', 'linear_acceleration_ly', 'linear_acceleration_lz',
          'gyroscope_gx', 'gyroscope_gy', 'gyroscope_gz',
          'gravity_grx', 'gravity_gry', 'gravity_grz',
          'magnetometer_mx', 'magnetometer_my', 'magnetometer_mz',
          'alignment_0', 'alignment_45',
          'accelerometer_ax', 'accelerometer_ay', 'accelerometer_az'
          ]]

# Target variables
y = data[['accel_magnitude', 'rpm_value']]

# Train-test split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Standardize the features
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

# Save the scaler for future use
joblib.dump(scaler, '60BladeModel_gyro_components.pkl')

# Create Polynomial Features
poly_degree = 1  # Linear relationship for now
poly = PolynomialFeatures(degree=poly_degree, include_bias=False)
X_train_poly = poly.fit_transform(X_train_scaled)
X_test_poly = poly.transform(X_test_scaled)

# Save the polynomial feature transformer for future use
joblib.dump(poly, '60BladeModelVibration.pkl')

# Train Multi-Output Polynomial Regression Model
try:
    poly_model = joblib.load('polynomial_accel_gyro_components_model.pkl')
    print("Loaded saved Polynomial Regression model.")
    print("Retraining the model with combined data...")
    poly_model.fit(X_train_poly, y_train)
    print("Retraining complete.")
except FileNotFoundError:
    print("No saved model found. Training a new Polynomial Regression model...")
    poly_model = MultiOutputRegressor(LinearRegression())
    poly_model.fit(X_train_poly, y_train)
    print("Training complete.")

# Save the Updated Model
joblib.dump(poly_model, '60BladeModel_components_model.pkl')
print("Updated model saved as polynomial_accel_gyro_components_model.pkl.")

# Predict on Train and Test Data
y_train_pred = poly_model.predict(X_train_poly)
y_test_pred = poly_model.predict(X_test_poly)

# Evaluate the Model
mse_train = mean_squared_error(y_train, y_train_pred, multioutput='uniform_average')
mse_test = mean_squared_error(y_test, y_test_pred, multioutput='uniform_average')

r2_train = r2_score(y_train, y_train_pred, multioutput='uniform_average')
r2_test = r2_score(y_test, y_test_pred, multioutput='uniform_average')

# Convert R² to accuracy percentage
accuracy_train = r2_train * 100
accuracy_test = r2_test * 100

print(f"Training MSE: {mse_train}")
print(f"Testing MSE: {mse_test}")
print(f"Training RMSE: {np.sqrt(mse_train)}, Testing RMSE: {np.sqrt(mse_test)}")
print(f"Training R² Score: {r2_train:.4f} ({accuracy_train:.2f}%)")
print(f"Testing R² Score: {r2_test:.4f} ({accuracy_test:.2f}%)")

# Predict on Specific Speed Values
speed_values = [12]  # Example speed values for predictions
test_data = pd.DataFrame({
    'speed_value': speed_values,
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
    'accelerometer_az': [X['accelerometer_az'].mean()] * len(speed_values)
})

# Predict on Test Data
y_test_pred = poly_model.predict(X_test_poly)

# Calculate Residuals for `accel_magnitude` and `rpm_value`
accel_residuals = np.abs(y_test['accel_magnitude'].values - y_test_pred[:, 0])
rpm_residuals = np.abs(y_test['rpm_value'].values - y_test_pred[:, 1])

# Define Separate Thresholds for Anomalies
accel_threshold = np.mean(accel_residuals) + 2 * np.std(accel_residuals)  # 3 standard deviations
rpm_threshold = np.mean(rpm_residuals) + 4 * np.std(rpm_residuals)        # 3 standard deviations

# Flag Anomalies Separately
accel_anomalies = accel_residuals > accel_threshold
rpm_anomalies = rpm_residuals > rpm_threshold

# Combine Results into a DataFrame
anomaly_results = pd.DataFrame({
    'Actual Acceleration Magnitude': y_test['accel_magnitude'].values,
    'Predicted Acceleration Magnitude': y_test_pred[:, 0],
    'Accel Residuals': accel_residuals,
    'Accel Is Anomaly': accel_anomalies,
    'Actual RPM': y_test['rpm_value'].values,
    'Predicted RPM': y_test_pred[:, 1],
    'RPM Residuals': rpm_residuals,
    'RPM Is Anomaly': rpm_anomalies
})

# Display Anomaly Results
print(anomaly_results)

# Separate Analysis for Magnitude Anomalies
magnitude_anomalies = anomaly_results[anomaly_results['Accel Is Anomaly']]
print("\nMagnitude Anomalies:")
print(magnitude_anomalies)

# Separate Analysis for RPM Anomalies
rpm_anomalies = anomaly_results[anomaly_results['RPM Is Anomaly']]
print("\nRPM Anomalies:")
print(rpm_anomalies)

# Plot Predictions and Anomalies for `accel_magnitude`
plt.figure(figsize=(8, 6))
plt.plot(y_test['accel_magnitude'].values, label='Actual Acceleration Magnitude', linestyle='--')
plt.plot(y_test_pred[:, 0], label='Predicted Acceleration Magnitude', linestyle=':')
plt.scatter(np.where(accel_anomalies), y_test_pred[:, 0][accel_anomalies], color='red', label='Anomalies')
plt.xlabel('Sample Index')
plt.ylabel('Acceleration Magnitude')
plt.title('Anomaly Detection for Acceleration Magnitude')
plt.legend()
plt.grid(True)
plt.show()

# Plot Predictions and Anomalies for `rpm_value`
plt.figure(figsize=(8, 6))
plt.plot(y_test['rpm_value'].values, label='Actual RPM', linestyle='--')
plt.plot(y_test_pred[:, 1], label='Predicted RPM', linestyle=':')
plt.scatter(np.where(rpm_anomalies), y_test_pred[:, 1][rpm_anomalies], color='red', label='Anomalies')
plt.xlabel('Sample Index')
plt.ylabel('RPM')
plt.title('Anomaly Detection for RPM')
plt.legend()
plt.grid(True)
plt.show()