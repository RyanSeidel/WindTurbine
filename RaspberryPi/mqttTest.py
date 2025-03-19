import paho.mqtt.client as mqtt
import os
import time

# MQTT Configuration
MQTT_BROKER = "192.168.1.208"  # Replace with your Raspberry Pi's IP address
MQTT_PORT = 1883  # Default MQTT port

# Topics under wind_turbine namespace
MQTT_TOPICS = {
    'rpm': 'wind_turbine/rpm',
    'orientation': 'wind_turbine/orientation',
    'temperature': 'wind_turbine/temperature',
    'magnetometer': 'wind_turbine/magnetometer',
    'gyroscope': 'wind_turbine/gyroscope',
    'accelerometer': 'wind_turbine/accelerometer',
    'linear_acceleration': 'wind_turbine/linear_acceleration',
    'gravity': 'wind_turbine/gravity',
    'calibration': 'wind_turbine/calibration',
    'voltage': 'wind_turbine/volt',
    'power': 'wind_turbine/power',
    'current': 'wind_turbine/current',
    'servo': 'wind_turbine/servo'
}

# Initialize MQTT client
mqtt_client = mqtt.Client()

# MQTT on_connect callback to confirm connection
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!", flush=True)
        # Subscribe to all topics in MQTT_TOPICS
        for topic in MQTT_TOPICS.values():
            client.subscribe(topic)
        print("Subscribed to all wind_turbine topics", flush=True)
    else:
        print(f"Failed to connect, return code {rc}", flush=True)

# MQTT on_message callback to handle incoming messages for each topic
def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode()
    print(f"Received message: {payload} on topic {topic}", flush=True)

# Assign callbacks
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

# Connect to MQTT broker and loop
print(f"Attempting to connect to MQTT Broker at {MQTT_BROKER}:{MQTT_PORT}", flush=True)
mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
mqtt_client.loop_forever()