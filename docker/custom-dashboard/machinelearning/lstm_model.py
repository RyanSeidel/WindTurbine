# 1. LSTM (Long Short-Term Memory) with Multi-Input for Time-Series Forecasting

#     Best For: Handling temporal dependencies across multiple features in time-series data, particularly effective for predicting RPM and volts based on historical and influencing data.
#     Why: LSTM networks are designed to capture patterns in sequential data, making them ideal for scenarios where the past values of RPM, volts, and environmental features (like wind speed and direction) influence future values.
#     Application:
#         Use a multi-input LSTM network where each feature (blade orientation, wind speed, wind direction, historical RPM, and volts) is treated as a separate input to the model.
#         Train the model to predict both RPM and volts as outputs.
#     Benefits:
#         Highly effective for sequential dependencies, making it well-suited to handle historical sensor data.
#         Can simultaneously capture complex relationships between features and target variables (RPM and volts).


# lstm_model.py

import numpy as np
from tensorflow.keras.models import load_model

# Load the model (assumes it is already trained and saved as "lstm_model.h5")
model = load_model("lstm_model.h5")

def predict_rpm_volts(wind_speed, wind_direction, temperature, orientation, n_timesteps=30, n_features=6):
    # Prepare input array with the last timestep populated with user inputs
    input_data = np.zeros((1, n_timesteps, n_features))
    input_data[0, -1, 0] = wind_speed
    input_data[0, -1, 1] = wind_direction
    input_data[0, -1, 2] = temperature
    input_data[0, -1, 3] = orientation

    # Get the prediction from the model
    predicted_rpm, predicted_volts = model.predict(input_data)[0]
    return predicted_rpm, predicted_volts
