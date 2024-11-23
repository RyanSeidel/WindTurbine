import paho.mqtt.client as mqtt

# MQTT broker configuration
MQTT_BROKER = "127.0.0.1"  # Localhost
MQTT_PORT = 1883
MQTT_TOPIC = "wind_turbine/test"  # The topic to subscribe to

# Callback when the client successfully connects to the broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
        # Subscribe to the topic
        client.subscribe(MQTT_TOPIC)
        print(f"Subscribed to topic: {MQTT_TOPIC}")
    else:
        print(f"Failed to connect, return code {rc}")

# Callback when a message is received on the subscribed topic
def on_message(client, userdata, msg):
    print(f"Received message on topic {msg.topic}: {msg.payload.decode()}")

# Initialize the MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Connect to the MQTT broker
print("Connecting to broker...")
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Start listening for messages
print("Listening for messages...")
client.loop_forever()
