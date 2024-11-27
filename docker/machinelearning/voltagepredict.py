import pandas as pd
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.preprocessing import PolynomialFeatures, StandardScaler
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import numpy as np
import joblib

# Load the datasets
data_files = [
    'AllDataUpdated3_filtered_data.csv'
]

# Combine datasets
data = pd.concat([pd.read_csv(f) for f in data_files], ignore_index=True)
print(f"Combined dataset contains {data.shape[0]} rows.")

# Select features and target
X = data[['rpm_value', 'speed_value', 'servo_value', 
          'orientation_heading', 'orientation_roll', 'orientation_pitch']]
y = data['voltage_value']  # Voltage as the target

# Train-test split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Standardize features
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)
joblib.dump(scaler, 'voltage_scaler.pkl')

# Polynomial features
poly = PolynomialFeatures(degree=1, include_bias=False)
X_train_poly = poly.fit_transform(X_train_scaled)
X_test_poly = poly.transform(X_test_scaled)
joblib.dump(poly, 'voltage_poly_features.pkl')

# Train a Polynomial Regression Model
try:
    voltage_model = joblib.load('voltage_wind_turbine_model.pkl')
    print("Loaded saved Voltage Prediction model.")
    print("Retraining the model with combined data...")
    voltage_model.fit(X_train_poly, y_train)
    print("Retraining complete.")
except FileNotFoundError:
    print("No saved model found. Training a new Voltage Prediction model...")
    voltage_model = LinearRegression()
    voltage_model.fit(X_train_poly, y_train)
    print("Training complete.")

# Save the updated model
joblib.dump(voltage_model, 'voltage_wind_turbine_model.pkl')
print("Updated model saved as voltage_wind_turbine_model.pkl.")

# Predictions
y_pred = voltage_model.predict(X_test_poly)

# Evaluate the model
mse_train = mean_squared_error(y_train, voltage_model.predict(X_train_poly))
mse_test = mean_squared_error(y_test, y_pred)

print(f"Training MSE: {mse_train}")
print(f"Testing MSE: {mse_test}")
print(f"Training RMSE: {np.sqrt(mse_train)}, Testing RMSE: {np.sqrt(mse_test)}")

# Cross-validation
cv_scores = cross_val_score(voltage_model, X_train_poly, y_train, cv=5, scoring='neg_mean_squared_error')
mean_cv_mse = -np.mean(cv_scores)
print(f"Cross-Validation Mean MSE: {mean_cv_mse}")

# Specific RPM Values for Voltage Prediction
specific_rpm_values = [30, 48, 60]
test_data = pd.DataFrame({
    'rpm_value': specific_rpm_values,
    'speed_value': [X['speed_value'].mean()] * len(specific_rpm_values),  # Fixed mean value
    'servo_value': [X['servo_value'].mean()] * len(specific_rpm_values),
    'orientation_heading': [X['orientation_heading'].mean()] * len(specific_rpm_values),
    'orientation_roll': [X['orientation_roll'].mean()] * len(specific_rpm_values),
    'orientation_pitch': [X['orientation_pitch'].mean()] * len(specific_rpm_values)
})

# Standardize the specific test data
test_data_scaled = scaler.transform(test_data)
test_data_poly = poly.transform(test_data_scaled)

# Predict Voltage for Specific RPM Values
specific_voltage_predictions = voltage_model.predict(test_data_poly)

# Print Predictions
print("\nVoltage Predictions for Specific RPM Values:")
for rpm, voltage in zip(specific_rpm_values, specific_voltage_predictions):
    print(f"RPM: {rpm}, Predicted Voltage: {voltage:.4f}")

# Visualize Polynomial Fit for RPM and Voltage
plt.scatter(X['rpm_value'], y, label='Actual', alpha=0.5)
plt.scatter(specific_rpm_values, specific_voltage_predictions, color='red', label='Predicted for Specific RPM')
rpm_values = np.linspace(X['rpm_value'].min(), X['rpm_value'].max(), 100)  # Generate 100 RPM values
test_data = pd.DataFrame({
    'rpm_value': rpm_values,
    'speed_value': [X['speed_value'].mean()] * len(rpm_values),
    'servo_value': [X['servo_value'].mean()] * len(rpm_values),
    'orientation_heading': [X['orientation_heading'].mean()] * len(rpm_values),
    'orientation_roll': [X['orientation_roll'].mean()] * len(rpm_values),
    'orientation_pitch': [X['orientation_pitch'].mean()] * len(rpm_values)
})

# Standardize and Predict for Fit Line
test_data_scaled = scaler.transform(test_data)
test_data_poly = poly.transform(test_data_scaled)
rpm_voltage_predictions = voltage_model.predict(test_data_poly)

# Plot Polynomial Fit
plt.plot(rpm_values, rpm_voltage_predictions, label='Polynomial Fit', color='blue')
plt.xlabel('RPM Value')
plt.ylabel('Voltage')
plt.legend()
plt.title('Polynomial Fit for RPM and Voltage')
plt.show()
