import paho.mqtt.client as mqtt

# MQTT Broker Configuration
MQTT_BROKER = "192.168.1.208"  # Replace with your broker's IP address
MQTT_PORT = 1883               # MQTT port, usually 1883
MQTT_TOPIC = "wind_turbine/3" # Topic to subscribe to

# Define the callback for when the client connects to the broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(MQTT_TOPIC)  # Subscribe to the specified topic
    else:
        print(f"Failed to connect, return code {rc}")

# Define the callback for when a message is received
def on_message(client, userdata, msg):
    print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

# Initialize the MQTT client
client = mqtt.Client()

# Attach the callbacks to the client
client.on_connect = on_connect
client.on_message = on_message

# Connect to the MQTT broker
print("Attempting to connect to MQTT Broker...")
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Start the MQTT client loop
client.loop_forever()
