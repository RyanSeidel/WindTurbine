import pandas as pd
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.preprocessing import PolynomialFeatures, StandardScaler
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import numpy as np
import joblib

# This is only for RPM and Voltage

# List of CSV files for all 8 directions
data_files = [
        # North Directions!        

    # 'AllDataUpdated3_filtered_data.csv', 
    'AllDataWithAlignmentCategory.csv',

    

]

# Load and combine all datasets
data = pd.concat([pd.read_csv(f) for f in data_files], ignore_index=True)
print(f"Combined dataset contains {data.shape[0]} rows.")

# Convert RPM to RPS
data['rps_value'] = data['rpm_value'] / 60  # Convert RPM to RPS

# Select relevant features and the target
X = data[['speed_value', 'servo_value', 'direction_value', 
          'orientation_heading', 'orientation_roll', 'orientation_pitch',
          'alignment_category']]

y = data['rps_value']  # Use 'rps_value' instead of 'rpm_value'

# Train-test split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Standardize the features to improve numerical stability
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

# Save the scaler for future use
joblib.dump(scaler, 'scaler.pkl')

# Create Polynomial Features
poly_degree = 2  # Adjust degree as needed
poly = PolynomialFeatures(degree=1, include_bias=False)
X_train_poly = poly.fit_transform(X_train_scaled)
X_test_poly = poly.transform(X_test_scaled)

# Save the polynomial feature transformer for future use
joblib.dump(poly, 'poly_features.pkl')

# Train Polynomial Regression Model
try:
    poly_model = joblib.load('polynomial_wind_turbine_model.pkl')
    print("Loaded saved Polynomial Regression model.")
    print("Retraining the model with combined data...")
    poly_model.fit(X_train_poly, y_train)
    print("Retraining complete.")
except FileNotFoundError:
    print("No saved model found. Training a new Polynomial Regression model...")
    poly_model = LinearRegression()
    poly_model.fit(X_train_poly, y_train)
    print("Training complete.")

# Save the Updated Model
joblib.dump(poly_model, 'polynomial_wind_turbine_model.pkl')
print("Updated model saved as polynomial_wind_turbine_model.pkl.")

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

# Visualize Predictions vs Actuals
plt.scatter(y_test, y_pred, alpha=0.5)
plt.plot([y_test.min(), y_test.max()], [y_test.min(), y_test.max()], color='red', linestyle='--')
plt.xlabel("Actual RPS")
plt.ylabel("Predicted RPS")
plt.title("Actual vs Predicted RPS")
plt.show()

# Visualize Polynomial Fit for a Single Feature
plt.scatter(X['speed_value'], y, label='Actual', alpha=0.5)
# Generate test data for visualization
speed_values = np.linspace(X['speed_value'].min(), X['speed_value'].max(), 100)  # Generate 100 speed values
test_data = pd.DataFrame({
    'speed_value': speed_values,
    'servo_value': [X['servo_value'].mean()] * len(speed_values),  # Fixed mean value
    'direction_value': [X['direction_value'].mode()[0]] * len(speed_values),  # Most common value
    'orientation_heading': [X['orientation_heading'].mean()] * len(speed_values),  # Fixed mean value
    'orientation_roll': [X['orientation_roll'].mean()] * len(speed_values),  # Fixed mean value
    'orientation_pitch': [X['orientation_pitch'].mean()] * len(speed_values), # Fixed mean value
    'alignment_category': [X['alignment_category'].mean()] * len(speed_values)
})

# Standardize the full test data
test_data_scaled = scaler.transform(test_data)  # Scale all features

# Apply polynomial transformation
test_data_poly = poly.transform(test_data_scaled)

# Predict RPS values
speed_predictions = poly_model.predict(test_data_poly)

# Plot the polynomial fit for speed_value
plt.plot(speed_values, speed_predictions, label='Polynomial Fit', color='red')
plt.scatter(X['speed_value'], y, alpha=0.5, label='Actual Data')
plt.xlabel('Speed Value')
plt.ylabel('RPS')
plt.legend()
plt.title('Polynomial Fit for Speed Value')
plt.show()