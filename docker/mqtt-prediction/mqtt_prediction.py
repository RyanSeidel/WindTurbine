import os
import time
import logging
import paho.mqtt.client as mqtt
import pandas as pd
import joblib

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# MQTT Configuration
MQTT_BROKER = os.getenv("MQTT_BROKER", "mosquitto")  # Default: Mosquitto broker
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
RPS_INPUT_TOPIC = "rpsinputform"  # Topic for listening to form data
PREDICTION_TOPIC = "rps_predictions"  # Topic for publishing predictions

# Load Pre-trained Models and Scalers
MODEL_FILE = "model/linear_regression_model.pkl"
SCALER_FILE = "model/scaler.pkl"
VOLTAGE_MODEL_FILE = "model/voltage_wind_turbine_model.pkl"
VOLTAGE_SCALER_FILE = "model/voltage_scaler.pkl"
VOLTAGE_POLY_FILE = "model/voltage_poly_features.pkl"

try:
    linear_model = joblib.load(MODEL_FILE)
    scaler = joblib.load(SCALER_FILE)
    voltage_model = joblib.load(VOLTAGE_MODEL_FILE)
    voltage_scaler = joblib.load(VOLTAGE_SCALER_FILE)
    voltage_poly = joblib.load(VOLTAGE_POLY_FILE)
    logging.info("Loaded pre-trained models and scalers.")
except FileNotFoundError as e:
    logging.error(f"Model or scaler file not found: {e}")
    exit()

# Initialize MQTT client
mqtt_client = mqtt.Client()

# MQTT Callback Functions
def on_connect(client, userdata, flags, rc):
    """Callback for when the client connects to the broker."""
    if rc == 0:
        logging.info("Connected to MQTT broker.")
        mqtt_client.subscribe(RPS_INPUT_TOPIC)
        logging.info(f"Subscribed to topic: {RPS_INPUT_TOPIC}")
    else:
        logging.error(f"Failed to connect to MQTT broker. Return code: {rc}")

def on_message(client, userdata, msg):
    """Callback for handling received messages."""
    topic = msg.topic
    payload = msg.payload.decode()
    if topic == RPS_INPUT_TOPIC:
        logging.info(f"Received message on {topic}: {payload}")
        process_rps_input(payload)

def process_rps_input(payload):
    """Process the received form data."""
    try:
        # Parse the payload (assuming JSON-like data)
        data = eval(payload)  # Use `json.loads(payload)` if JSON format is ensured

        # Extract the relevant values
        wind_speed = data.get("windSpeed", 0)
        wind_direction = data.get("windDirection", 0)
        blade_angle = data.get("bladeAngle", 15)
        orientation = data.get("orientation", 0)

        # Define direction mappings for `orientation_heading` and `servo_value`
        direction_mapping = {
            0: {"orientation_heading": 360, "servo_value": 0},  # North
            45: {"orientation_heading": 315, "servo_value": 45},  # Northeast
            90: {"orientation_heading": 270, "servo_value": 90},  # East
            135: {"orientation_heading": 225, "servo_value": 135},  # Southeast
            180: {"orientation_heading": 180, "servo_value": 180},  # South
            225: {"orientation_heading": 135, "servo_value": 225},  # Southwest
            270: {"orientation_heading": 90, "servo_value": 270},  # West
            315: {"orientation_heading": 45, "servo_value": 315},  # Northwest
        }

        # Use the wind direction to get the corresponding values
        mapped_values = direction_mapping.get(wind_direction, {"orientation_heading": 360, "servo_value": 0})
        orientation_heading = mapped_values["orientation_heading"]
        servo_value = mapped_values["servo_value"]

        # Handle alignment based on orientation
        alignment_0 = 1 if orientation == 0 else 0
        alignment_1 = 1 if orientation == 45 else 0
        alignment_2 = 0  # Default to 0 for simplicity (add more if needed)

        # Prepare data for RPS prediction
        test_data = pd.DataFrame({
            "speed_value": [wind_speed],
            "blade_60": [1 if blade_angle == 60 else 0],
            "blade_45": [1 if blade_angle == 45 else 0],
            "blade_30": [1 if blade_angle == 30 else 0],
            "blade_15": [1 if blade_angle == 15 else 0],
            "alignment_0": [alignment_0],
            "alignment_1": [alignment_1],
            "alignment_2": [alignment_2],
            "orientation_heading": [orientation_heading],
            "orientation_roll": [0],
            "orientation_pitch": [0],
            "servo_value": [servo_value],
        })

        # Standardize the data
        test_data_scaled = scaler.transform(test_data)

        # Predict RPS
        predicted_rps = linear_model.predict(test_data_scaled)[0]
        
        # Convert RPS to RPM
        predicted_rpm = predicted_rps * 60

        # Prepare data for voltage prediction
        voltage_test_data = pd.DataFrame({"rpm_value": [predicted_rpm]})
        voltage_test_data_scaled = voltage_scaler.transform(voltage_test_data)
        voltage_test_data_poly = voltage_poly.transform(voltage_test_data_scaled)

        # Predict Voltage
        predicted_voltage = voltage_model.predict(voltage_test_data_poly)[0]

        # Log and publish the predictions
        prediction_payload = {
            "predictedRPS": predicted_rps,
            "predictedVoltage": predicted_voltage
        }
        mqtt_client.publish(PREDICTION_TOPIC, str(prediction_payload))
        logging.info(f"Published prediction: {prediction_payload}")

    except Exception as e:
        logging.error(f"Error processing payload: {e}")

# Main function
def main():
    try:
        # Attach callbacks
        mqtt_client.on_connect = on_connect
        mqtt_client.on_message = on_message

        # Connect to the MQTT broker
        logging.info(f"Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}...")
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()  # Start the loop to process messages
        logging.info("MQTT client loop started.")

        # Keep the script running
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        logging.info("Stopped by user.")
    finally:
        # Gracefully disconnect the client
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        logging.info("Disconnected from MQTT broker.")

if __name__ == "__main__":
    main()
