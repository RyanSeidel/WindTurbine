import pandas as pd
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
from sklearn.metrics import r2_score
import numpy as np
import joblib

# Load dataset
data_files = [  
    'Combine_All_RPM_data.csv',
              
              ]  # Add your dataset paths here
data = pd.concat([pd.read_csv(f) for f in data_files], ignore_index=True)
print(f"Combined dataset contains {data.shape[0]} rows.")

# Data Preprocessing
# Convert RPM to RPS
data['rps_value'] = data['rpm_value'] / 60  # Convert RPM to RPS

# Select features and target
X = data[['speed_value', 
          'blade_60', 
          'blade_45', 
          'blade_30', 
          'blade_15', 
          'alignment_0', 
          'alignment_1', 
          'alignment_2', 
          'orientation_heading', 
          'orientation_roll', 
          'orientation_pitch',
          'servo_value'
]]
y = data['rps_value']

# Train-test split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Standardize features
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

# Save the scaler for future use
joblib.dump(scaler, 'scaler.pkl')

# Train Linear Regression Model
model = LinearRegression()
model.fit(X_train_scaled, y_train)

# Save the trained model
joblib.dump(model, 'linear_regression_model.pkl')

# Predictions
y_train_pred = model.predict(X_train_scaled)
y_test_pred = model.predict(X_test_scaled)

# Evaluate the Model
mse_train = mean_squared_error(y_train, y_train_pred)
mse_test = mean_squared_error(y_test, y_test_pred)
print(f"Training MSE: {mse_train}")
print(f"Testing MSE: {mse_test}")
print(f"Training RMSE: {np.sqrt(mse_train)}, Testing RMSE: {np.sqrt(mse_test)}")

# Calculate R² for training and testing data
r2_train = r2_score(y_train, y_train_pred)
r2_test = r2_score(y_test, y_test_pred)

# Convert R² to accuracy percentage
accuracy_train = r2_train * 100
accuracy_test = r2_test * 100

print(f"Training Accuracy: {accuracy_train:.2f}%")
print(f"Testing Accuracy: {accuracy_test:.2f}%")

# Visualize Predictions vs Actual
plt.scatter(y_test, y_test_pred, alpha=0.5, label="Predicted vs Actual")
plt.plot([y_test.min(), y_test.max()], [y_test.min(), y_test.max()], color='red', linestyle='--', label="Perfect Fit")
plt.xlabel("Actual RPS")
plt.ylabel("Predicted RPS")
plt.title("Linear Regression: Predicted vs Actual RPS")
plt.legend()
plt.show()

# Generate Test Data for Visualization
speed_values = np.linspace(X['speed_value'].min(), X['speed_value'].max(), 100)  # Generate 100 speed values
test_data = pd.DataFrame({
    'speed_value': speed_values,
    'blade_60': [1] * len(speed_values),  # Assume 0-degree alignment
    'blade_45': [0] * len(speed_values),
    'blade_30': [0] * len(speed_values),
    'blade_15': [0] * len(speed_values),
    'alignment_0': [1] * len(speed_values),  # Assume 0-degree alignment
    'alignment_1': [0] * len(speed_values),
    'alignment_2': [0] * len(speed_values),
    'orientation_heading': [X['orientation_heading'].mean()] * len(speed_values),  # Use mean value
    'orientation_roll': [X['orientation_roll'].mean()] * len(speed_values),        # Use mean value
    'orientation_pitch': [X['orientation_pitch'].mean()] * len(speed_values),      # Use mean value
    'servo_value': [X['servo_value'].mean()] * len(speed_values),
})



# Standardize test data
test_data_scaled = scaler.transform(test_data)

# Predict RPS for test data
speed_predictions = model.predict(test_data_scaled)

# Plot Speed vs Predicted RPS
plt.plot(speed_values, speed_predictions, label='Linear Fit', color='red')
plt.scatter(X['speed_value'], y, alpha=0.5, label='Actual Data')
plt.xlabel('Speed Value')
plt.ylabel('RPS')
plt.legend()
plt.title('Linear Fit for Speed Value with Alignment Mode')
plt.show()
