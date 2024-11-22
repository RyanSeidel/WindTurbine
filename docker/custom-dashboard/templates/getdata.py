import paho.mqtt.client as mqtt

BROKER = "192.168.0.100"  # MQTT broker IP address
PORT = 1883               # Default MQTT port
TOPIC = "wind_turbine/#"  # Subscribe to all wind_turbine topics

# Callback for when the client connects to the broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(TOPIC)
    else:
        print(f"Failed to connect, return code {rc}")

# Callback for when a message is received from the broker
def on_message(client, userdata, msg):
    print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Connect to the broker
client.connect(BROKER, PORT, 60)

# Keep the client loop running to listen for messages
client.loop_forever()
