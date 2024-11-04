# 1. LSTM (Long Short-Term Memory) with Multi-Input for Time-Series Forecasting

#     Best For: Handling temporal dependencies across multiple features in time-series data, particularly effective for predicting RPM and volts based on historical and influencing data.
#     Why: LSTM networks are designed to capture patterns in sequential data, making them ideal for scenarios where the past values of RPM, volts, and environmental features (like wind speed and direction) influence future values.
#     Application:
#         Use a multi-input LSTM network where each feature (blade orientation, wind speed, wind direction, historical RPM, and volts) is treated as a separate input to the model.
#         Train the model to predict both RPM and volts as outputs.
#     Benefits:
#         Highly effective for sequential dependencies, making it well-suited to handle historical sensor data.
#         Can simultaneously capture complex relationships between features and target variables (RPM and volts).


import numpy as np
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense
from tensorflow.keras.models import load_model

# -------------------------------
# Step 1: Data Preparation
# -------------------------------

# Example parameters (customize these for your data)
n_samples = 1000       # Number of training samples (sequences)
n_timesteps = 30       # Number of timesteps in each sequence
n_features = 6         # Number of features (e.g., orientation, wind speed, etc.)
n_outputs = 2          # Number of outputs: RPM and volts

# Generating random data for demonstration (replace with actual data)
X_train = np.random.rand(n_samples, n_timesteps, n_features)
y_train = np.random.rand(n_samples, n_outputs)  # Output data: RPM and volts

# -------------------------------
# Step 2: Define and Compile the LSTM Model
# -------------------------------

# Initialize the LSTM model
model = Sequential()
model.add(LSTM(64, activation='relu', input_shape=(n_timesteps, n_features)))
model.add(Dense(32, activation='relu'))
model.add(Dense(n_outputs))  # Output layer for RPM and volts

# Compile the model
model.compile(optimizer='adam', loss='mse')

# -------------------------------
# Step 3: Train the Model
# -------------------------------

# Train the model with your training data
history = model.fit(X_train, y_train, epochs=50, batch_size=32, validation_split=0.2)

# Save the model for future use
model.save("lstm_model.h5")

# -------------------------------
# Step 4: Real-Time Prediction (After Model Training)
# -------------------------------

# Load the saved model (optional, to demonstrate reloading in a real-time app)
model = load_model("lstm_model.h5")

# Example new input for real-time prediction (use real data here)
new_input = np.random.rand(1, n_timesteps, n_features)  # Shape: (1, timesteps, features)

# Get predictions
predicted_rpm, predicted_volts = model.predict(new_input)[0]
print(f"Predicted RPM: {predicted_rpm}")
print(f"Predicted Volts: {predicted_volts}")

# -------------------------------
# Optional: Integrate with InfluxDB or Real-Time System
# -------------------------------

# Example function to simulate real-time data retrieval
def get_real_time_data():
    # Replace with actual logic to pull the last 'n_timesteps' data points
    return np.random.rand(1, n_timesteps, n_features)

# Example of using model in a loop for real-time predictions
def real_time_prediction_loop():
    while True:
        real_time_input = get_real_time_data()  # Get the latest data sequence
        predicted_rpm, predicted_volts = model.predict(real_time_input)[0]
        print(f"Predicted RPM: {predicted_rpm}, Predicted Volts: {predicted_volts}")
        
        # Add code to send predictions to InfluxDB, MQTT, or other system as needed
        # time.sleep(1)  # Sleep for a second or adjust as per real-time requirements

# Uncomment the line below to start real-time predictions
# real_time_prediction_loop()
