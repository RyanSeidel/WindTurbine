import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.preprocessing import PolynomialFeatures, StandardScaler
from sklearn.multioutput import MultiOutputRegressor
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import joblib

# Load dataset
data_files = [
    'AllDataUpdated3_filtered_data.csv',  # Replace with your actual file(s)
]
data = pd.concat([pd.read_csv(f) for f in data_files], ignore_index=True)
print(f"Combined dataset contains {data.shape[0]} rows.")

# Select features and targets
X = data[['speed_value', 'servo_value', 'direction_value',
          'orientation_heading', 'orientation_roll', 'orientation_pitch',
          'linear_acceleration_lx', 'linear_acceleration_ly', 'linear_acceleration_lz']]

y = data[['gyroscope_gx', 'gyroscope_gy', 'gyroscope_gz',
          'accelerometer_ax', 'accelerometer_ay', 'accelerometer_az']]  # Individual components

# Train-test split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Standardize the features to improve numerical stability
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

# Save the scaler for future use
joblib.dump(scaler, 'scaler_accel_gyro_components.pkl')

# Create Polynomial Features
poly_degree = 1  # Linear relationship for now
poly = PolynomialFeatures(degree=poly_degree, include_bias=False)
X_train_poly = poly.fit_transform(X_train_scaled)
X_test_poly = poly.transform(X_test_scaled)

# Save the polynomial feature transformer for future use
joblib.dump(poly, 'poly_features_accel_gyro_components.pkl')

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
joblib.dump(poly_model, 'polynomial_accel_gyro_components_model.pkl')
print("Updated model saved as polynomial_accel_gyro_components_model.pkl.")

# Predict on Test Data
y_pred = poly_model.predict(X_test_poly)

# Evaluate the Model
mse_train = mean_squared_error(y_train, poly_model.predict(X_train_poly))
mse_test = mean_squared_error(y_test, y_pred)

print(f"Training MSE: {mse_train}")
print(f"Testing MSE: {mse_test}")
print(f"Training RMSE: {np.sqrt(mse_train)}, Testing RMSE: {np.sqrt(mse_test)}")

# Cross-validation to validate the polynomial degree
cv_scores = cross_val_score(poly_model, X_train_poly, y_train, cv=5, scoring='neg_mean_squared_error')
mean_cv_mse = -np.mean(cv_scores)
print(f"Cross-Validation Mean MSE: {mean_cv_mse}")

# Visualize Predictions vs Actuals for Accelerometer Ax
plt.scatter(y_test['accelerometer_ax'], y_pred[:, 3], alpha=0.5)
plt.plot([y_test['accelerometer_ax'].min(), y_test['accelerometer_ax'].max()],
         [y_test['accelerometer_ax'].min(), y_test['accelerometer_ax'].max()], color='red', linestyle='--')
plt.xlabel("Actual Accelerometer Ax")
plt.ylabel("Predicted Accelerometer Ax")
plt.title("Actual vs Predicted Accelerometer Ax")
plt.show()

# Visualize Predictions vs Actuals for Gyroscope Gx
plt.scatter(y_test['gyroscope_gx'], y_pred[:, 0], alpha=0.5)
plt.plot([y_test['gyroscope_gx'].min(), y_test['gyroscope_gx'].max()],
         [y_test['gyroscope_gx'].min(), y_test['gyroscope_gx'].max()], color='red', linestyle='--')
plt.xlabel("Actual Gyroscope Gx")
plt.ylabel("Predicted Gyroscope Gx")
plt.title("Actual vs Predicted Gyroscope Gx")
plt.show()

# Predict on Specific Speed Values
speed_values = [0, 30, 100]
test_data = pd.DataFrame({
    'speed_value': speed_values,
    'servo_value': [X['servo_value'].mean()] * len(speed_values),  # Fixed mean value for servo
    'direction_value': [X['direction_value'].mode()[0]] * len(speed_values),  # Most common value
    'orientation_heading': [X['orientation_heading'].mean()] * len(speed_values),  # Fixed mean
    'orientation_roll': [X['orientation_roll'].mean()] * len(speed_values),  # Fixed mean
    'orientation_pitch': [X['orientation_pitch'].mean()] * len(speed_values),  # Fixed mean
    'linear_acceleration_lx': [X['linear_acceleration_lx'].mean()] * len(speed_values),
    'linear_acceleration_ly': [X['linear_acceleration_ly'].mean()] * len(speed_values),
    'linear_acceleration_lz': [X['linear_acceleration_lz'].mean()] * len(speed_values)
})

# Standardize the test data
test_data_scaled = scaler.transform(test_data)

# Apply polynomial transformation
test_data_poly = poly.transform(test_data_scaled)

# Predict individual components
predictions = poly_model.predict(test_data_poly)

# Display predictions
predicted_data = pd.DataFrame(predictions, columns=['Gyroscope Gx', 'Gyroscope Gy', 'Gyroscope Gz',
                                                    'Accelerometer Ax', 'Accelerometer Ay', 'Accelerometer Az'])
predicted_data['Speed Value'] = speed_values
print(predicted_data)

# Plot the predictions
plt.figure(figsize=(8, 6))
for i, label in enumerate(['Gyroscope Gx', 'Gyroscope Gy', 'Gyroscope Gz']):
    plt.plot(speed_values, predictions[:, i], label=label, marker='o')
plt.xlabel('Speed Value')
plt.ylabel('Gyroscope Components')
plt.title('Predicted Gyroscope Components vs Speed')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(8, 6))
for i, label in enumerate(['Accelerometer Ax', 'Accelerometer Ay', 'Accelerometer Az']):
    plt.plot(speed_values, predictions[:, i + 3], label=label, marker='o')
plt.xlabel('Speed Value')
plt.ylabel('Accelerometer Components')
plt.title('Predicted Accelerometer Components vs Speed')
plt.legend()
plt.grid(True)
plt.show()
