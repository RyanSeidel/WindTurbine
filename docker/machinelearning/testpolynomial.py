import numpy as np
import pandas as pd
import joblib  # For loading saved models

# Load the saved model, scaler, and polynomial feature transformer
poly_model = joblib.load('polynomial_wind_turbine_model.pkl')
scaler = joblib.load('scaler.pkl')  # Load the scaler used during training
poly = joblib.load('poly_features.pkl')  # Load the polynomial feature transformer

# Example: Specific wind speeds to test
test_wind_speeds = [0, 5, 6, 7]

# Create a dataframe for testing (repeat values to match test_wind_speeds length)
test_data = pd.DataFrame({
    'speed_value': test_wind_speeds,                # Wind speeds you want to test
    'servo_value': [90] * len(test_wind_speeds),    # Repeat values to match test_wind_speeds length
    'direction_value': [1] * len(test_wind_speeds),  
    'orientation_heading': [270] * len(test_wind_speeds),
    'orientation_roll': [1] * len(test_wind_speeds),
    'orientation_pitch': [-1] * len(test_wind_speeds),
})

# Apply scaling and polynomial transformation
test_data_scaled = scaler.transform(test_data)           # Apply StandardScaler
test_data_poly = poly.transform(test_data_scaled)        # Apply PolynomialFeatures

# Make predictions
predictions = poly_model.predict(test_data_poly)

# Display results
results = pd.DataFrame({
    'Wind Speed': test_wind_speeds,
    'Predicted RPS': predictions.flatten()  # Flatten predictions if it's 2D
})
print(results)
