import os
import time
import logging
import pandas as pd
import numpy as np
import paho.mqtt.client as mqtt
import joblib

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# Load blade configuration from environment variables
blade_1 = int(os.environ.get('blade_1', 0))

# Load environment variables or use defaults
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
RASP_BROKER = os.getenv("RASP_BROKER", "192.168.1.208")
PUBLISH_TOPIC = os.getenv("PUBLISH_TOPIC", "wind_turbine/anomaly")

# Topics under wind_turbine namespace
MQTT_TOPICS = {
    'speed': 'wind_turbine/speed',
    'rpm': 'wind_turbine/rpm',
    'magnetometer': 'wind_turbine/magnetometer',
    'gyroscope': 'wind_turbine/gyroscope',
    'accelerometer': 'wind_turbine/accelerometer',
    'linear_acceleration': 'wind_turbine/linear_acceleration',
    'gravity': 'wind_turbine/gravity',
    'voltage': 'wind_turbine/volt',
    'power': 'wind_turbine/power',
    'current': 'wind_turbine/current'
}

# Initialize the MQTT clients
main_client = mqtt.Client()
rasp_client = mqtt.Client()

# Load Pre-trained Models
try:
  # 97 Percent Score
    scaler60 = joblib.load('model/60BladeModel_gyro_components.pkl')
    poly60 = joblib.load('model/60BladeModelVibration.pkl')
    poly_model60 = joblib.load('model/60BladeModel_components_model.pkl')
  # 96.7 Percent Score
    scaler45 = joblib.load('model/45BladeModel_gyro_components.pkl')
    poly45 = joblib.load('model/45BladeModelVibration.pkl')
    poly_model45 = joblib.load('model/45BladeModel_components_model.pkl')
  # 88 Percent Score
    scaler30 = joblib.load('model/30BladeModel_gyro_components.pkl')
    poly30 = joblib.load('model/30BladeModelVibration.pkl')
    poly_model30 = joblib.load('model/30BladeModel_components_model.pkl')
  # 96 Percent Score
    scaler15 = joblib.load('model/15BladeModel_gyro_components.pkl')
    poly15 = joblib.load('model/15BladeModelVibration.pkl')
    poly_model15 = joblib.load('model/15BladeModel_components_model.pkl')
    logging.info("Loaded pre-trained models.")
except FileNotFoundError as e:
    logging.error(f"Error loading models: {e}")
    exit()


# Data storage
columns = [
    'speed_value', 'rpm_value', 
    'linear_acceleration_lx', 'linear_acceleration_ly', 'linear_acceleration_lz',
    'gyroscope_gx', 'gyroscope_gy', 'gyroscope_gz',
    'gravity_grx', 'gravity_gry', 'gravity_grz',
    'magnetometer_mx', 'magnetometer_my', 'magnetometer_mz',
    'alignment_0', 'alignment_45', 
    'accelerometer_ax', 'accelerometer_ay', 'accelerometer_az',
    'power_value', 'current_value', 'voltage_value'
]

df = pd.DataFrame([{col: None for col in columns}])

# MQTT callback functions
def on_connect(client, userdata, flags, rc):
    """Callback for when a client connects to the broker."""
    if rc == 0:
        logging.info(f"Connected to MQTT broker.")
        # Subscribe to all relevant topics
        for topic in MQTT_TOPICS.values():
            client.subscribe(topic)
            logging.info(f"Subscribed to topic: {topic}")
    else:
        logging.error(f"Failed to connect to MQTT broker. Return code: {rc}")

def on_message(client, userdata, msg):
    """Callback for when a message is received."""
    topic = msg.topic
    payload = msg.payload.decode()
    logging.info(f"Received message from topic {topic}: {payload}")
    update_dataframe(topic, payload)
    
    # Make prediction and publish it
    make_and_publish_prediction()

def update_dataframe(topic, payload):
    global df
    try:
        # Update values from MQTT topics
        if topic == MQTT_TOPICS['accelerometer']:
            ax, ay, az = map(float, payload.split(','))
            df.at[0, 'accelerometer_ax'] = ax
            df.at[0, 'accelerometer_ay'] = ay
            df.at[0, 'accelerometer_az'] = az
        elif topic == MQTT_TOPICS['speed']:
            speed_value = float(payload)
            df.at[10, 'speed_value'] = speed_value
        elif topic == MQTT_TOPICS['magnetometer']:
            mx, my, mz = map(float, payload.split(','))
            df.at[0, 'magnetometer_mx'] = mx
            df.at[0, 'magnetometer_my'] = my
            df.at[0, 'magnetometer_mz'] = mz
        elif topic == MQTT_TOPICS['gyroscope']:
            gx, gy, gz = map(float, payload.split(','))
            df.at[0, 'gyroscope_gx'] = gx
            df.at[0, 'gyroscope_gy'] = gy
            df.at[0, 'gyroscope_gz'] = gz
        elif topic == MQTT_TOPICS['gravity']:
            grx, gry, grz = map(float, payload.split(','))
            df.at[0, 'gravity_grx'] = grx
            df.at[0, 'gravity_gry'] = gry
            df.at[0, 'gravity_grz'] = grz
            logging.info(f"Updated Gravity values: grx={grx}, gry={gry}, grz={grz}")
        elif topic == MQTT_TOPICS['linear_acceleration']:
            lx, ly, lz = map(float, payload.split(','))
            df.at[0, 'linear_acceleration_lx'] = lx
            df.at[0, 'linear_acceleration_ly'] = ly
            df.at[0, 'linear_acceleration_lz'] = lz
        elif topic == MQTT_TOPICS['power']:
            power = float(payload)  # Payload is a single value
            df.at[0, 'power_value'] = power
        elif topic == MQTT_TOPICS['voltage']:
            voltage = float(payload)  # Payload is a single value
            df.at[0, 'voltage_value'] = voltage
        elif topic == MQTT_TOPICS['current']:
            current = float(payload)  # Payload is a single value
            df.at[0, 'current_value'] = current
        

        # Add static or default values for missing features
        df.at[0, 'speed_value'] = 10
        df.at[0, 'rpm_value'] = 1
        df.at[0, 'alignment_0'] = 1 
        df.at[0, 'alignment_45'] = 0 
        
        # Ensure the full DataFrame is printed
        # pd.set_option('display.max_columns', None)  # Display all columns
        # pd.set_option('display.max_rows', None)     # Display all rows
        # pd.set_option('display.width', 1000)       # Adjust display width
        # pd.set_option('display.colheader_justify', 'center')  # Align headers

        # Print the entire DataFrame
        # logging.info(f"Entire DataFrame:\n{df}")
        # print(f"Entire DataFrame:\n{df}")  # For quick debugging in terminal
    except Exception as e:
        logging.error(f"Error updating DataFrame: {e}")
    
# Initialize a sliding window for residuals
residual_window = []
WINDOW_SIZE = 50  # Number of recent samples to consider for threshold calculation

def update_threshold(new_residual):
    global residual_window
    
    # Add the new residual to the sliding window
    residual_window.append(new_residual)
    
    # Keep only the last N residuals
    if len(residual_window) > WINDOW_SIZE:
        residual_window.pop(0)
    
    # Calculate mean and standard deviation dynamically
    mean_residual = np.mean(residual_window)
    std_residual = np.std(residual_window)
    
    # Set the threshold
    return mean_residual + 2 * std_residual

def make_and_publish_prediction():
    global df
    try:
        # Drop `accel_magnitude` from the input features if it exists
        input_features = df.drop(columns=['accel_magnitude'], errors='ignore')
    
        # Ensure all feature names match those used during training
        input_data = input_features.astype('float32')
               
        if blade_1 == 60:
          logging.info(f"Blade is 60")
          # Standardize and transform the input features
          input_data_scaled = scaler60.transform(input_data)
          input_data_poly = poly60.transform(input_data_scaled)
          # Predict acceleration magnitude
          predictions = poly_model60.predict(input_data_poly)
        elif blade_1 == 45:
          logging.info(f"Blade is 45")
          # Standardize and transform the input features
          input_data_scaled = scaler45.transform(input_data)
          input_data_poly = poly45.transform(input_data_scaled)
          # Predict acceleration magnitude
          predictions = poly_model45.predict(input_data_poly)
        elif blade_1 == 30:
          logging.info(f"Blade is 30")
          # Standardize and transform the input features
          input_data_scaled = scaler30.transform(input_data)
          input_data_poly = poly30.transform(input_data_scaled)
          # Predict acceleration magnitude
          predictions = poly_model30.predict(input_data_poly)       
        elif blade_1 == 15:
          logging.info(f"Blade is 15")
          # Standardize and transform the input features
          input_data_scaled = scaler15.transform(input_data)
          input_data_poly = poly15.transform(input_data_scaled)
          # Predict acceleration magnitude
          predictions = poly_model15.predict(input_data_poly)
        else:
          logging.info(f"Blade is 0")
          
        # Calculate the actual acceleration magnitude from accelerometer data
        accel_magnitude = np.sqrt(
          df.at[0, 'accelerometer_ax']**2 +
          df.at[0, 'accelerometer_ay']**2 +
          df.at[0, 'accelerometer_az']**2
        )
    
        # Calculate residuals (difference between predicted and actual)
        residuals = np.abs(predictions.flatten() - accel_magnitude)

        # Update the anomaly threshold dynamically
        threshold = update_threshold(residuals[0])

        # Determine if the residual exceeds the threshold
        anomalies = residuals > threshold

        # Log the results
        logging.info(f"Predictions: {predictions.flatten()}")
        logging.info(f"Actual Magnitude: {accel_magnitude}")
        logging.info(f"Residuals: {residuals}")
        logging.info(f"Anomaly Threshold: {threshold}")
        logging.info(f"Anomalies Detected: {anomalies}")

        # Prepare the result for publishing
        result = {
          "Predicted_Magnitude": float(predictions[0]),
          "Actual_Magnitude": float(accel_magnitude),
          "Residual": float(residuals[0]),
          "Threshold": float(threshold),
          "Anomaly": bool(anomalies[0]),
        }
        # Publish the result
        main_client.publish(PUBLISH_TOPIC, str(result))
        logging.info(f"Published: {result}")

    except Exception as e:
        logging.error(f"Error during prediction: {e}")

# Main function
def main():
    try:
        # Attach callbacks to rasp_client
        rasp_client.on_connect = on_connect
        rasp_client.on_message = on_message

        # Connect rasp_client to the Raspberry Pi MQTT broker
        logging.info(f"Connecting to Raspberry Pi MQTT broker at {RASP_BROKER}:{MQTT_PORT}...")
        rasp_client.connect(RASP_BROKER, MQTT_PORT, 60)
        rasp_client.loop_start()  # Start the subscriber loop
        logging.info("Raspberry Pi MQTT client loop started.")

        # Connect and start main_client for publishing predictions
        logging.info(f"Connecting to Main MQTT broker at {MQTT_BROKER}:{MQTT_PORT}...")
        main_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        main_client.loop_start()  # Start the publisher loop
        logging.info("Main MQTT client loop started.")

        # Keep the script running
        while True:
            time.sleep(1), 
    except KeyboardInterrupt:
        logging.info("Stopped by user.")
    finally:
        # Graceful shutdown of both clients
        rasp_client.loop_stop()
        rasp_client.disconnect()
        logging.info("Disconnected from Raspberry Pi MQTT broker.")

        main_client.loop_stop()
        main_client.disconnect()
        logging.info("Disconnected from Main MQTT broker.")

if __name__ == "__main__":
    main()
