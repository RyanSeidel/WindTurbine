from influxdb_client import InfluxDBClient
import pandas as pd
import numpy as np
from sklearn.preprocessing import MinMaxScaler
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense, Input
from tensorflow.keras.metrics import MeanAbsoluteError

# InfluxDB Configuration
INFLUXDB_URL = "http://localhost:8086"
INFLUXDB_TOKEN = "iNLROvcnYQmb6CNVmUyrNuB6CG2EiKOjUrT-F13uF-x1pSYLZGcGS-rbgj9J1cS-zaUwMB6UPd8_SJgVl3KFdQ=="
INFLUXDB_ORG = "TAMUCC"
INFLUXDB_BUCKET = "WindTurbine"

# Initialize the InfluxDB client
client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
query_api = client.query_api()

# Define the measurements and fields you want to query
measurements = {
    "rpm": ["value"],
    "orientation": ["heading", "roll", "pitch"],
    "temperature": ["value"],
    "magnetometer": ["mx", "my", "mz"],
    "gyroscope": ["gx", "gy", "gz"],
    "accelerometer": ["ax", "ay", "az"],
    "linear_acceleration": ["lx", "ly", "lz"],
    "gravity": ["grx", "gry", "grz"],
    "voltage": ["value"],
    "current": ["value"],
    "power": ["value"]
}

# Retrieve data for each measurement and store it in a DataFrame
data = {}
print("Retrieving data from InfluxDB...")
for measurement, fields in measurements.items():
    for field in fields:
        query = f'''
        from(bucket: "{INFLUXDB_BUCKET}")
        |> range(start: -1h)
        |> filter(fn: (r) => r["_measurement"] == "{measurement}")
        |> filter(fn: (r) => r["_field"] == "{field}")
        '''
        result = query_api.query(org=INFLUXDB_ORG, query=query)
        
        times = []
        values = []
        for table in result:
            for record in table.records:
                times.append(record.get_time())
                values.append(record.get_value())
        
        if values:
            print(f"{measurement} ({field}) sample data: {values[:5]}")
        else:
            print(f"No data found for {measurement} ({field})")
        
        data[f"{measurement}_{field}"] = pd.Series(data=values, index=times)

client.close()

# Combine series into a DataFrame, aligning on the timestamp
df = pd.DataFrame(data)
df = df.ffill().bfill()

# Normalize the data for the LSTM model
scaler = MinMaxScaler()
scaled_data = scaler.fit_transform(df)
df_scaled = pd.DataFrame(scaled_data, columns=df.columns, index=df.index)

# Convert data to LSTM format [samples, timesteps, features]
def create_sequences(data, timesteps):
    sequences = []
    for i in range(len(data) - timesteps):
        sequences.append(data[i:i+timesteps])
    return np.array(sequences)

timesteps = 10
formatted_data = create_sequences(df_scaled.values, timesteps)

# Split data into training and testing sets
train_size = int(len(formatted_data) * 0.8)
train_data = formatted_data[:train_size]
test_data = formatted_data[train_size:]
print("Training data shape:", train_data.shape)
print("Testing data shape:", test_data.shape)

# Build the LSTM model
# Build the LSTM model with an explicit Input layer
model = Sequential([
    Input(shape=(timesteps, train_data.shape[2])),  # Define input shape in Input layer
    LSTM(50, activation='relu', return_sequences=True),
    LSTM(50, activation='relu', return_sequences=True),
    Dense(train_data.shape[2])  # Output layer with the same number of features
])

model.compile(optimizer='adam', loss='mse', metrics=[MeanAbsoluteError()])
model.summary()

# Train the model
history = model.fit(train_data, train_data, epochs=50, batch_size=32, validation_split=0.2)

# Evaluate the model on the test set
test_loss, test_mae = model.evaluate(test_data, test_data)
print("Test Loss (MSE):", test_loss)
print("Test MAE:", test_mae)

# Optional: Save the model and scaler for future use
model.save("wind_turbine_lstm_model.h5")
np.save("scaler_min.npy", scaler.data_min_)
np.save("scaler_max.npy", scaler.data_max_)
