import numpy as np
import pandas as pd
import joblib  # For loading saved models
import matplotlib.pyplot as plt  # Ensure matplotlib.pyplot is imported

# Load the saved model and scaler
linear_model = joblib.load('linear_regression_model.pkl')  # Linear regression model
scaler = joblib.load('scaler.pkl')  # Scaler used during training

# Define specific wind speeds for testing
test_wind_speeds = [0, 5, 6, 10]

# Create a dataframe for testing (match features used in training)
test_data = pd.DataFrame({
    'speed_value': test_wind_speeds,                 # Wind speeds to test
    'blade_60': [1] * len(test_wind_speeds),        # Assume blade_60 is active
    'blade_45': [0] * len(test_wind_speeds),
    'blade_30': [0] * len(test_wind_speeds),
    'blade_15': [0] * len(test_wind_speeds),
    'alignment_0': [1] * len(test_wind_speeds),     # Assume alignment_0
    'alignment_1': [0] * len(test_wind_speeds),
    'alignment_2': [0] * len(test_wind_speeds),
    'orientation_heading': [360] * len(test_wind_speeds),  # Replace with mean or fixed values
    'orientation_roll': [0] * len(test_wind_speeds),
    'orientation_pitch': [0] * len(test_wind_speeds),
    'servo_value': [0] * len(test_wind_speeds),         # Replace with mean or fixed values
})

# Standardize test data
test_data_scaled = scaler.transform(test_data)  # Apply the same scaler used during training

# Make predictions
predictions = linear_model.predict(test_data_scaled)

# Display results
results = pd.DataFrame({
    'Wind Speed': test_wind_speeds,
    'Predicted RPS': predictions
})
print(results)

# Visualize predictions
plt.figure(figsize=(8, 6))
plt.plot(test_wind_speeds, predictions, label='Predicted RPS', color='blue', marker='o')
plt.xlabel('Wind Speed')
plt.ylabel('Predicted RPS')
plt.title('Predicted RPS vs Wind Speed')
plt.grid()
plt.legend()
plt.show()
