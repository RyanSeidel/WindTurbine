import pandas as pd
import numpy as np
import joblib

# Load Pre-trained Model
MODEL_FILE = 'wind_turbine_model.pkl'

try:
    multi_model = joblib.load(MODEL_FILE)
    print(f"Loaded pre-trained model from {MODEL_FILE}.")
except FileNotFoundError:
    print(f"Model file {MODEL_FILE} not found. Please train the model first.")
    exit()

# Example Input Data
example_data = pd.DataFrame({
    'speed_value': [16],
    'direction_value': [180],
    'pressure_value': [1013],
    'servo_value': [90],
    'humidity_value': [55.0],
    'orientation_heading': [1],
    'orientation_roll': [0],
    'orientation_pitch': [0],
    'rpm_blade_1': [60],
    'rpm_blade_2': [60],
    'rpm_blade_3': [60],
}, dtype='float32')

# Ensure Feature Order Matches Model
expected_features = multi_model.estimators_[0].get_booster().feature_names
example_data = example_data[expected_features]

# Predict
predictions = multi_model.predict(example_data)

# Adjust Predictions as Needed
predictions[0][0] = predictions[0][0] * 60  # Convert RPS to RPM

# Output Predictions
print("Predicted RPS:", predictions[0][0])
