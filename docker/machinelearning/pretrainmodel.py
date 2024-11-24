import pandas as pd
import numpy as np
import joblib
from sklearn.metrics import mean_squared_error

# Load Pre-trained Model
MODEL_FILE = 'wind_turbine_model.pkl'

try:
    multi_model = joblib.load(MODEL_FILE)
    print(f"Loaded pre-trained model from {MODEL_FILE}.")
except FileNotFoundError:
    print(f"Model file {MODEL_FILE} not found. Please train the model first.")
    exit()

# Define Function to Predict
def predict(data):
    """
    Predict using the pre-trained model.
    
    Args:
        data (pd.DataFrame): Input features for prediction.
        
    Returns:
        np.array: Predicted values for RPS and Voltage.
    """
    return multi_model.predict(data)

# Example Input Data
example_data = pd.DataFrame({
    'speed_value': [10.5],
    'direction_value': [180],
    'pressure_value': [1013],
    'servo_value': [90],
    'humidity_value': [55.0],
    'orientation_heading': [0],
    'orientation_roll': [0],
    'orientation_pitch': [0],
    'temperature_value': [25.0],
    'rpm_blade_1': [60],
    'rpm_blade_2': [60],
    'rpm_blade_3': [60],
    'current_value': [10.5],
    'power_value': [100.0],
    'magnetometer_mx': [0.3],
    'magnetometer_my': [0.2],
    'magnetometer_mz': [0.1],
    'gyroscope_gx': [0.0],
    'gyroscope_gy': [0.0],
    'gyroscope_gz': [0.0],
    'accelerometer_ax': [0.01],
    'accelerometer_ay': [0.01],
    'accelerometer_az': [-0.01],
    'linear_acceleration_lx': [0.0],
    'linear_acceleration_ly': [0.0],
    'linear_acceleration_lz': [0.0],
    'gravity_grx': [0.0],
    'gravity_gry': [0.0],
    'gravity_grz': [1.0],
    'altitude_value': [10.0]
}, dtype='float32')

# Predict
predictions = predict(example_data)

predictions[0][0] = predictions[0][0] * 60

# Output Predictions
print("Predicted RPS:", predictions[0][0])
print("Predicted Voltage:", predictions[0][1])
