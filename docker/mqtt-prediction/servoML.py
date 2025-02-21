import os
import time
import logging
import json
import serial
import numpy as np
import paho.mqtt.client as mqtt
from sklearn.linear_model import SGDRegressor

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# Load environment variables or use defaults
MQTT_BROKER = os.getenv("MQTT_BROKER", "mosquitto")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
RASP_BROKER = os.getenv("RASP_BROKER")
PUBLISH_TOPIC = "orientation_updated"

# Sensor topics under the wind_turbine namespace
MQTT_TOPICS = {
    'wind_direction': 'wind_turbine/wind_direction',
    'rpm': 'wind_turbine/rpm',
    'voltage': 'wind_turbine/voltage',
    'anomaly': 'wind_turbine/anomaly_predictions'
}

# Bluetooth Serial (ESP32) Configuration
SERIAL_PORT = "COM6"  # Change as needed
BAUD_RATE = 115200

# Global dictionary to store the latest sensor readings
latest_features = {
    'wind_direction': None,
    'rpm': None,
    'voltage': None,
    'anomaly': None
}

# Define feature names to be used in the model
feature_names = ['wind_direction', 'rpm', 'voltage', 'anomaly']

# Linear regression model for online updates
model = None
model_initialized = False

def send_angle(angle):
    """Send the predicted orientation angle to the servo over Bluetooth serial."""
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            ser.write(f"{angle}\n".encode())
            logging.info(f"Sent angle {angle} to servo")
            time.sleep(1)  # Allow time for the servo/ESP32 to process the command
    except Exception as e:
        logging.error(f"Error sending angle: {e}")

def compute_target_orientation(wind_direction, rpm, voltage, anomaly):
    """
    Compute the target orientation based on sensor data.
    For demonstration, this function rounds the wind_direction to the nearest 15,
    but if the value reaches 90 or above, it caps it to 75.
    Modify this heuristic as needed.
    """
    target = round(wind_direction / 15) * 15
    if target >= 90:
        target = 75
    return target

def on_connect(client, userdata, flags, rc):
    """Callback for MQTT connection."""
    if rc == 0:
        logging.info("Connected to MQTT broker")
        # Subscribe to sensor topics only
        for topic in MQTT_TOPICS.values():
            client.subscribe(topic)
            logging.info(f"Subscribed to topic: {topic}")
    else:
        logging.error(f"Failed to connect. Return code: {rc}")

def on_message(client, userdata, msg):
    """
    Callback for handling incoming MQTT messages.
    
    For each sensor topic, update the latest sensor data.
    When all sensor data is available, compute the target orientation,
    update the linear regression model with the new sample,
    and then predict the orientation. The result is published on the
    "orientation_updated" topic and sent to the servo via Bluetooth.
    """
    global latest_features, model, model_initialized
    topic = msg.topic
    payload = msg.payload.decode()
    logging.info(f"Received message on {topic}: {payload}")

    try:
        if topic == MQTT_TOPICS['wind_direction']:
            latest_features['wind_direction'] = float(payload)
        elif topic == MQTT_TOPICS['rpm']:
            latest_features['rpm'] = float(payload)
        elif topic == MQTT_TOPICS['voltage']:
            latest_features['voltage'] = float(payload)
        elif topic == MQTT_TOPICS['anomaly']:
            latest_features['anomaly'] = float(payload)
    except Exception as e:
        logging.error(f"Error updating sensor data: {e}")
        return

    # Check if all sensor data are available
    if all(latest_features[key] is not None for key in feature_names):
        # Compute the target orientation from the sensor readings
        target_orientation = compute_target_orientation(
            latest_features['wind_direction'],
            latest_features['rpm'],
            latest_features['voltage'],
            latest_features['anomaly']
        )
        logging.info(f"Computed target orientation: {target_orientation}")

        # Prepare feature array for the model
        features = np.array([latest_features[key] for key in feature_names]).reshape(1, -1)

        # Update the model with the new sample using online learning (partial_fit)
        if not model_initialized:
            model = SGDRegressor(max_iter=1000, tol=1e-3)
            model.partial_fit(features, [target_orientation])
            model_initialized = True
            logging.info("Initialized model with first sample.")
        else:
            model.partial_fit(features, [target_orientation])
            logging.info("Updated model with new sample.")

        # Make a prediction using the current sensor readings
        predicted_orientation = model.predict(features)[0]
        # Constrain the prediction to the servo's valid range and round it
        predicted_orientation = max(1, min(180, round(predicted_orientation)))
        logging.info(f"Predicted orientation: {predicted_orientation}")

        # Publish the updated orientation
        result = {"orientation": predicted_orientation}
        client.publish(PUBLISH_TOPIC, json.dumps(result))
        logging.info(f"Published updated orientation: {result}")

        # Send the predicted orientation to the servo via Bluetooth
        send_angle(predicted_orientation)

def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        # Connect to the Raspberry Pi MQTT broker (or the designated broker)
        client.connect(RASP_BROKER, MQTT_PORT, 60)
    except Exception as e:
        logging.error(f"Failed to connect to MQTT broker: {e}")
        return

    # Run the MQTT loop indefinitely
    client.loop_forever()

if __name__ == "__main__":
    main()
