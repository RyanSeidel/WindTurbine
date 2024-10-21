import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient, Point

# Configuration for InfluxDB
token = "87tXlLtsEcPfC2DenA3KQtoEr6_YfJAmPHkt1xaAvIOshk8Vb2QuiTLahWceg15uVbGehKIDtxvhbYBuE6K68g=="
org = "TAMUCC"
bucket = "wind_turbine_real_time"
client = InfluxDBClient(url="http://influxdb:8086", token=token, org=org)  # 'influxdb' is the Docker service name

write_api = client.write_api()

# MQTT callback for when a message is received
def on_message(client, userdata, msg):
    sensor_data = msg.payload.decode("utf-8")
    data = f"wind_turbine,sensor=wind_speed value={sensor_data}"
    write_api.write(bucket=bucket, record=data)

# Initialize the MQTT client but don't connect yet
mqtt_client = mqtt.Client()
mqtt_client.on_message = on_message

# Placeholder for broker address and topic
mqtt_broker_address = None
mqtt_topic = None

# Function to connect to MQTT (can be triggered later)
def connect_to_mqtt(broker_address, topic):
    global mqtt_broker_address, mqtt_topic
    mqtt_broker_address = broker_address
    mqtt_topic = topic
    print(f"Connecting to MQTT broker at {mqtt_broker_address} on topic {mqtt_topic}")
    
    # Connect to the MQTT broker
    mqtt_client.connect(mqtt_broker_address, 1883, 60)
    mqtt_client.subscribe(mqtt_topic)
    mqtt_client.loop_forever()

# Commented out for now
# connect_to_mqtt("mqtt_broker_address", "your_mqtt_topic")
