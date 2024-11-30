import os
import time
import logging
import pandas as pd
import paho.mqtt.client as mqtt
import joblib

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# Load environment variables or use defaults
MQTT_BROKER = os.getenv("MQTT_BROKER", "mosquitto")
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

# Load Pre-trained Model
MODEL_FILE = 'wind_turbine_model.pkl'
try:
    model = joblib.load(MODEL_FILE)
    logging.info(f"Loaded pre-trained model from {MODEL_FILE}.")
except FileNotFoundError:
    logging.error(f"Model file {MODEL_FILE} not found. Please train the model first.")
    exit()

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

# Initialize data with default values from the environment
data = {col: [None] for col in columns}
data['rpm_blade_1'] = [float(os.getenv('blade_1', 0))]
data['rpm_blade_2'] = [float(os.getenv('blade_2', 0))]
data['rpm_blade_3'] = [float(os.getenv('blade_3', 0))]

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
    
    # Make prediction and publish it
    make_and_publish_prediction()

def update_dataframe(topic, payload):
    """Update the DataFrame with new data."""
    global df
    if topic == MQTT_TOPICS['accelerometer']:
        ax, ay, az = map(float, payload.split(','))
        df.at[0, 'accelerometer_ax'] = ax
        df.at[0, 'accelerometer_ay'] = ay
        df.at[0, 'accelerometer_az'] = az
        logging.info(f"Updated Accelerometer values: {ax}, {ay}, {az}")

    elif topic == MQTT_TOPICS['speed']:
        speed_value = float(payload)
        df.at[0, 'speed_value'] = speed_value
        logging.info(f"Updated Speed value in DataFrame: {speed_value}")

    elif topic == MQTT_TOPICS['direction']:
        direction_value = int(payload)
        df.at[0, 'direction_value'] = direction_value
        logging.info(f"Updated Direction value in DataFrame: {direction_value}")

    elif topic == MQTT_TOPICS['pressure']:
        pressure_value = float(payload)
        df.at[0, 'pressure_value'] = pressure_value
        logging.info(f"Updated Pressure value in DataFrame: {pressure_value}")

    elif topic == MQTT_TOPICS['servo']:
        servo_value = int(payload)
        df.at[0, 'servo_value'] = servo_value
        logging.info(f"Updated Servo value in DataFrame: {servo_value}")

    elif topic == MQTT_TOPICS['humidity']:
        humidity_value = float(payload)
        df.at[0, 'humidity_value'] = humidity_value
        logging.info(f"Updated Humidity value in DataFrame: {humidity_value}")

    elif topic == MQTT_TOPICS['temperature']:
        temperature_value = float(payload)
        df.at[0, 'temperature_value'] = temperature_value
        logging.info(f"Updated Temperature value in DataFrame: {temperature_value}")

    elif topic == MQTT_TOPICS['orientation']:
        heading, roll, pitch = map(float, payload.split(','))
        df.at[0, 'orientation_heading'] = heading
        df.at[0, 'orientation_roll'] = roll
        df.at[0, 'orientation_pitch'] = pitch
        logging.info(f"Updated Orientation values: Heading={heading}, Roll={roll}, Pitch={pitch}")

    elif topic == MQTT_TOPICS['magnetometer']:
        mx, my, mz = map(float, payload.split(','))
        df.at[0, 'magnetometer_mx'] = mx
        df.at[0, 'magnetometer_my'] = my
        df.at[0, 'magnetometer_mz'] = mz
        logging.info(f"Updated Magnetometer values: mx={mx}, my={my}, mz={mz}")

    elif topic == MQTT_TOPICS['gyroscope']:
        gx, gy, gz = map(float, payload.split(','))
        df.at[0, 'gyroscope_gx'] = gx
        df.at[0, 'gyroscope_gy'] = gy
        df.at[0, 'gyroscope_gz'] = gz
        logging.info(f"Updated Gyroscope values: gx={gx}, gy={gy}, gz={gz}")

    elif topic == MQTT_TOPICS['linear_acceleration']:
        lx, ly, lz = map(float, payload.split(','))
        df.at[0, 'linear_acceleration_lx'] = lx
        df.at[0, 'linear_acceleration_ly'] = ly
        df.at[0, 'linear_acceleration_lz'] = lz
        logging.info(f"Updated Linear Acceleration values: lx={lx}, ly={ly}, lz={lz}")

    elif topic == MQTT_TOPICS['gravity']:
        grx, gry, grz = map(float, payload.split(','))
        df.at[0, 'gravity_grx'] = grx
        df.at[0, 'gravity_gry'] = gry
        df.at[0, 'gravity_grz'] = grz
        logging.info(f"Updated Gravity values: grx={grx}, gry={gry}, grz={grz}")

    elif topic == MQTT_TOPICS['altitude']:
        altitude_value = float(payload)
        df.at[0, 'altitude_value'] = altitude_value
        logging.info(f"Updated Altitude value in DataFrame: {altitude_value}")

    elif topic == MQTT_TOPICS['current']:
        current_value = float(payload)
        df.at[0, 'current_value'] = current_value
        logging.info(f"Updated Current value in DataFrame: {current_value}")

    elif topic == MQTT_TOPICS['power']:
        power_value = float(payload)
        df.at[0, 'power_value'] = power_value
        logging.info(f"Updated Power value in DataFrame: {power_value}")

    print(df)
    #logging.info(f"Full DataFrame row:\n{df.iloc[0].to_dict()}")
    
def make_and_publish_prediction():
    """Make predictions using the model and publish them to MQTT."""
    global df
    if df.isnull().values.any():
        logging.warning("Incomplete data, skipping prediction.")
        return

    # Prepare data for prediction
    input_data = df.astype('float32')  # Ensure correct data type
    prediction = model.predict(input_data)

    # Convert prediction to RPM (if needed)
    predicted_rpm = prediction[0][0] * 60  # Example: Convert from RPS to RPM
    predicted_voltage = prediction[0][1]

    # Publish predictions
    main_client.publish(PUBLISH_TOPIC, f"Predicted RPM: {predicted_rpm:.2f}, Voltage: {predicted_voltage:.2f}")
    logging.info(f"Published predictions: RPM={predicted_rpm:.2f}, Voltage={predicted_voltage:.2f}")

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
            time.sleep(1)

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

