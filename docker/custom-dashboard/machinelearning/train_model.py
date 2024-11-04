from influxdb_client import InfluxDBClient
import numpy as np
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense

# InfluxDB Configuration
INFLUXDB_URL = "http://localhost:8086"  # Replace with your InfluxDB URL
INFLUXDB_TOKEN = "YOUR_INFLUXDB_TOKEN"
INFLUXDB_ORG = "YOUR_ORG"
INFLUXDB_BUCKET = "YOUR_BUCKET"

# Parameters for LSTM
n_timesteps = 30  # Number of timesteps per sequence
n_features = 14    # Number of features: orientation, magnetometer, temperature, volts, current, blade orientation
n_outputs = 2     # Target outputs: RPM and Volts

# Initialize InfluxDB Client
client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
query_api = client.query_api()

def fetch_data_from_influxdb(field):
    """
    Fetch data for a specific field from InfluxDB and return as a list.
    """
    query = f'''
        from(bucket: "{INFLUXDB_BUCKET}")
        |> range(start: -7d)  // Adjust the time range as needed
        |> filter(fn: (r) => r["_measurement"] == "wind_turbine" and r["_field"] == "{field}")
        |> keep(columns: ["_time", "_value"])
        |> sort(columns: ["_time"])
    '''
    tables = query_api.query(query)
    values = [record.get_value() for table in tables for record in table.records]
    return values

# ---------------
# Fetch Data for Each Feature
# ---------------

# Fetch each feature from InfluxDB

#Hall Effect Sensor
rpm = fetch_data_from_influxdb("rpm")
#BMEO055 Sensor
orientation = fetch_data_from_influxdb("orientation")
magnetometer = fetch_data_from_influxdb("magnetometer")
temperature = fetch_data_from_influxdb("temperature")
gyroscope = fetch_data_from_influxdb("gyroscope")
accelerometer = fetch_data_from_influxdb("accelerometer")
linear_acceleration = fetch_data_from_influxdb("linear_acceleration")
gravity = fetch_data_from_influxdb("gravity")
calibration = fetch_data_from_influxdb("calibration")

#INA260
volts = fetch_data_from_influxdb("volts")
current = fetch_data_from_influxdb("current")
power = fetch_data_from_influxdb("power")

#WeatherStation
windSpeed = fetch_data_from_influxdb("windspeed")
winddirection = fetch_data_from_influxdb("winddirection")

#user input
blade_orientation = fetch_data_from_influxdb("blade_orientation")

# Fetch target data
target_rpm = fetch_data_from_influxdb("rpm")
target_volts = fetch_data_from_influxdb("volts_output")  # Adjust field name if different

# # ---------------
# # Format Data for LSTM
# # ---------------

# # Combine features into a single array (X) for input to the model
# # Ensure that each list has enough data to create sequences of length n_timesteps
# min_length = min(len(orientation), len(magnetometer), len(temperature), len(volts), len(current), len(blade_orientation))

# # Trim data to the same length
# orientation = orientation[:min_length]
# magnetometer = magnetometer[:min_length]
# temperature = temperature[:min_length]
# volts = volts[:min_length]
# current = current[:min_length]
# blade_orientation = blade_orientation[:min_length]
# target_rpm = target_rpm[:min_length]
# target_volts = target_volts[:min_length]

# # Create sequences of n_timesteps for each feature
# X = []
# y = []
# for i in range(len(orientation) - n_timesteps):
#     X.append([
#         orientation[i:i+n_timesteps],
#         magnetometer[i:i+n_timesteps],
#         temperature[i:i+n_timesteps],
#         volts[i:i+n_timesteps],
#         current[i:i+n_timesteps],
#         blade_orientation[i:i+n_timesteps]
#     ])
#     y.append([target_rpm[i+n_timesteps], target_volts[i+n_timesteps]])

# # Convert X and y to numpy arrays with shapes (samples, timesteps, features) and (samples, outputs)
# X = np.array(X).transpose(0, 2, 1)  # Transpose to shape (samples, timesteps, features)
# y = np.array(y)

# print("Shape of X:", X.shape)  # Should be (samples, n_timesteps, n_features)
# print("Shape of y:", y.shape)  # Should be (samples, n_outputs)

# # ---------------
# # Define and Train the LSTM Model
# # ---------------

# # Define the LSTM model
# model = Sequential()
# model.add(LSTM(64, activation='relu', input_shape=(n_timesteps, n_features)))
# model.add(Dense(32, activation='relu'))
# model.add(Dense(n_outputs))  # Two outputs: RPM and Volts

# # Compile the model
# model.compile(optimizer='adam', loss='mse')

# # Train the model
# history = model.fit(X, y, epochs=50, batch_size=32, validation_split=0.2)

# # Save the model
# model.save("lstm_wind_turbine_model.h5")
# print("Model trained and saved as 'lstm_wind_turbine_model.h5'")
