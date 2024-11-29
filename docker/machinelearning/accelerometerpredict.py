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
    'wind_turbine_NorthWest_HighFan.csv',
    'wind_turbine_NorthWest_Zero_Degree_LowFan.csv'

]
data = pd.concat([pd.read_csv(f) for f in data_files], ignore_index=True)
print(f"Combined dataset contains {data.shape[0]} rows.")

# Select features and targets
X = data[['speed_value', 
          'orientation_heading', 'orientation_roll', 'orientation_pitch',
          'linear_acceleration_lx', 'linear_acceleration_ly', 'linear_acceleration_lz',
          'gyroscope_gx', 'gyroscope_gy', 'gyroscope_gz', 'magnetometer_mx', 
          'magnetometer_my', 'magnetometer_mz']]

y = data[['accelerometer_ax', 'accelerometer_ay', 'accelerometer_az']]  # Individual components

# Train-test split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Standardize the features
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

# Save the scaler for future use
joblib.dump(scaler, 'scaler_accel_gyro_components.pkl')

# Create Polynomial Features
poly_degree = 1 # Linear relationship for now
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

# Visualize Predictions vs Actuals for Accelerometer Ax
plt.scatter(y_test['accelerometer_ax'], y_test_pred[:, 0], alpha=0.5)
plt.plot([y_test['accelerometer_ax'].min(), y_test['accelerometer_ax'].max()],
         [y_test['accelerometer_ax'].min(), y_test['accelerometer_ax'].max()], color='red', linestyle='--')
plt.xlabel("Actual Accelerometer Ax")
plt.ylabel("Predicted Accelerometer Ax")
plt.title("Actual vs Predicted Accelerometer Ax")
plt.show()

# Predict on Specific Speed Values
speed_values = [0, 5, 10, 15]
test_data = pd.DataFrame({
    'speed_value': speed_values,
    'orientation_heading': [X['orientation_heading'].mean()] * len(speed_values),
    'orientation_roll': [X['orientation_roll'].mean()] * len(speed_values),
    'orientation_pitch': [X['orientation_pitch'].mean()] * len(speed_values),
    'linear_acceleration_lx': [X['linear_acceleration_lx'].mean()] * len(speed_values),
    'linear_acceleration_ly': [X['linear_acceleration_ly'].mean()] * len(speed_values),
    'linear_acceleration_lz': [X['linear_acceleration_lz'].mean()] * len(speed_values),
    'gyroscope_gx': [X['gyroscope_gx'].mean()] * len(speed_values),
    'gyroscope_gy': [X['gyroscope_gy'].mean()] * len(speed_values),
    'gyroscope_gz': [X['gyroscope_gz'].mean()] * len(speed_values),
    'magnetometer_mx': [X['gyroscope_gx'].mean()] * len(speed_values),
    'magnetometer_my': [X['gyroscope_gy'].mean()] * len(speed_values),
    'magnetometer_mz': [X['gyroscope_gz'].mean()] * len(speed_values)
})

# Standardize the test data
test_data_scaled = scaler.transform(test_data)

# Apply polynomial transformation
test_data_poly = poly.transform(test_data_scaled)

# Predict individual components
predictions = poly_model.predict(test_data_poly)

# Display predictions
predicted_data = pd.DataFrame(predictions, columns=['Accelerometer Ax', 'Accelerometer Ay', 'Accelerometer Az'])
predicted_data['Speed Value'] = speed_values
print(predicted_data)

# Plot the predictions
plt.figure(figsize=(8, 6))
for i, label in enumerate(['Accelerometer Ax', 'Accelerometer Ay', 'Accelerometer Az']):
    plt.plot(speed_values, predictions[:, i], label=label, marker='o')
plt.xlabel('Speed Value')
plt.ylabel('Accelerometer Components')
plt.title('Predicted Accelerometer Components vs Speed')
plt.legend()
plt.grid(True)
plt.show()
