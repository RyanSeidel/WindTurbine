import os
import time
import logging
import pandas as pd
import paho.mqtt.client as mqtt

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# Load environment variables or use defaults
MQTT_BROKER = os.getenv("MQTT_BROKER", "host.docker.internal")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
RASP_BROKER = os.getenv("RASP_BROKER")
PUBLISH_TOPIC = os.getenv("PUBLISH_TOPIC", "wind_turbine/predictions")

# Topics under wind_turbine namespace
MQTT_TOPICS = {
    'speed': 'wind_turbine/speed',
    'direction': 'wind_turbine/direction',
    'pressure': 'wind_turbine/pressure',
    'servo': 'wind_turbine/servo',
    'humidity': 'wind_turbine/humidity',
    'temperature': 'wind_turbine/temperature',
    'rpm': 'wind_turbine/rpm',
    'orientation': 'wind_turbine/orientation',
    'magnetometer': 'wind_turbine/magnetometer',
    'gyroscope': 'wind_turbine/gyroscope',
    'accelerometer': 'wind_turbine/accelerometer',
    'linear_acceleration': 'wind_turbine/linear_acceleration',
    'gravity': 'wind_turbine/gravity',
    'altitude': 'wind_turbine/altitude',
    'current': 'wind_turbine/current',
    'power': 'wind_turbine/power',
}

# Initialize the MQTT clients
main_client = mqtt.Client()
rasp_client = mqtt.Client()

# Data storage
columns = [
    'speed_value', 'direction_value', 'pressure_value', 'servo_value',
    'humidity_value', 'orientation_heading', 'orientation_roll', 'orientation_pitch',
    'temperature_value', 'rpm_blade_1', 'rpm_blade_2', 'rpm_blade_3',
    'current_value', 'power_value', 'magnetometer_mx', 'magnetometer_my', 'magnetometer_mz',
    'gyroscope_gx', 'gyroscope_gy', 'gyroscope_gz', 'accelerometer_ax', 'accelerometer_ay',
    'accelerometer_az', 'linear_acceleration_lx', 'linear_acceleration_ly',
    'linear_acceleration_lz', 'gravity_grx', 'gravity_gry', 'gravity_grz', 'altitude_value'
]

data = {col: [] for col in columns}  # Initialize empty lists for each column
df = pd.DataFrame(data)

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

def update_dataframe(topic, payload):
    """Update the DataFrame with new data."""
    global df
    if topic == MQTT_TOPICS['rpm']:
        rpm_value = float(payload)
        
        # Update the 'rpm' field in the DataFrame
        df.at[0, 'rpm_value'] = rpm_value
        logging.info(f"Updated RPM value in DataFrame: {rpm_value}")
    elif topic == MQTT_TOPICS['accelerometer']:
        # Parse accelerometer payload
        ax, ay, az = map(float, payload.split(','))  # Split payload and convert to floats

        # Update the accelerometer columns in the DataFrame
        df.at[0, 'accelerometer_ax'] = ax
        df.at[0, 'accelerometer_ay'] = ay
        df.at[0, 'accelerometer_az'] = az

        logging.info(f"Updated Accelerometer values: {ax}, {ay}, {az}")
    e

    # Log DataFrame content for debugging
    print(df)
    logging.info(f"Full DataFrame row:\n{df.iloc[0].to_dict()}")

# Main function
def main():
    try:
        # Attach callbacks
        rasp_client.on_connect = on_connect
        rasp_client.on_message = on_message

        # Connect to the Raspberry Pi MQTT broker
        logging.info(f"Connecting to Raspberry Pi MQTT broker at {RASP_BROKER}:{MQTT_PORT}...")
        rasp_client.connect(RASP_BROKER, MQTT_PORT, 60)

        # Start the loop
        rasp_client.loop_forever()
    except KeyboardInterrupt:
        logging.info("Stopped by user.")
    finally:
        rasp_client.disconnect()
        logging.info("Disconnected from Raspberry Pi MQTT broker.")

if __name__ == "__main__":
    main()
