from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
import paho.mqtt.client as mqtt
#from machinelearning.lstm_model import predict_rpm_volts
import os
import time


# InfluxDB Configuration
INFLUXDB_URL = os.getenv("INFLUXDB_URL", "http://localhost:8086")
INFLUXDB_TOKEN = os.getenv("INFLUXDB_TOKEN", "iNLROvcnYQmb6CNVmUyrNuB6CG2EiKOjUrT-F13uF-x1pSYLZGcGS-rbgj9J1cS-zaUwMB6UPd8_SJgVl3KFdQ==")
INFLUXDB_ORG = os.getenv("INFLUXDB_ORG", "TAMUCC")
INFLUXDB_BUCKET = os.getenv("INFLUXDB_BUCKET", "WindTurbine")

# Initialize the InfluxDB client
influx_client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
write_api = influx_client.write_api(write_options=SYNCHRONOUS)  # This line initializes write_api

# MQTT Configuration
MQTT_BROKER = os.getenv("MQTT_BROKER", "192.168.0.100")  # Replace with your MQTT broker address
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
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
    'servo':'wind_turbine/servo'
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
    
    
    # Emit data based on topic
    if topic == MQTT_TOPICS['rpm']:
        payload_value = float(payload)
        point = Point("rpm").field("value", payload_value).time(int(time.time() * 1000), write_precision="ms")
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
    elif topic == MQTT_TOPICS['temperature']:
        payload_value = float(payload)
        point = Point("temperature").field("value", payload_value).time(int(time.time() * 1000), write_precision="ms")
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        
    elif topic == MQTT_TOPICS['orientation']:
        # Split the payload by commas
        parts = payload.split(',')
        heading = float(parts[0].strip())
        roll = float(parts[1].strip())
        pitch = float(parts[2].strip())
        
        # Create a point for InfluxDB with each value as a field
        point = (
            Point("orientation")
            .field("heading", heading)
            .field("roll", roll)
            .field("pitch", pitch)
            .time(int(time.time() * 1000), write_precision="ms")
        )
        
        # Write to InfluxDB
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        print(f"Orientation data written to InfluxDB: Heading={heading}, Roll={roll}, Pitch={pitch}", flush=True)
           
    elif topic == MQTT_TOPICS['magnetometer']:
        
        # Split the payload by commas
        parts = payload.split(',')
        mx = float(parts[0].strip())
        my = float(parts[1].strip())
        mz = float(parts[2].strip())
        
                # Create a point for InfluxDB with each value as a field
        point = (
            Point("magnetometer")
            .field("mx", mx)
            .field("my", my)
            .field("mz", mz)
            .time(int(time.time() * 1000), write_precision="ms")
        )
        
        # Write to InfluxDB
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        print(f"Magnetometer data written to InfluxDB: mx={mx}, my={my}, mz={mz}", flush=True)
        
        
    elif topic == MQTT_TOPICS['gyroscope']:
        
        # Split the payload by commas
        parts = payload.split(',')
        gx = float(parts[0].strip())
        gy = float(parts[1].strip())
        gz = float(parts[2].strip())
        
                # Create a point for InfluxDB with each value as a field
        point = (
            Point("gyroscope")
            .field("gx", gx)
            .field("gy", gy)
            .field("gz", gz)
            .time(int(time.time() * 1000), write_precision="ms")
        )
        
        # Write to InfluxDB
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        print(f"Gyroscope data written to InfluxDB: mx={gx}, my={gy}, mz={gz}", flush=True)
        

    elif topic == MQTT_TOPICS['accelerometer']:
        
        # Split the payload by commas
        parts = payload.split(',')
        ax = float(parts[0].strip())
        ay = float(parts[1].strip())
        az = float(parts[2].strip())
        
                # Create a point for InfluxDB with each value as a field
        point = (
            Point("accelerometer")
            .field("ax", ax)
            .field("ay", ay)
            .field("az", az)
            .time(int(time.time() * 1000), write_precision="ms")
        )
        
        # Write to InfluxDB
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        print(f"Accelerometer data written to InfluxDB: mx={ax}, my={ay}, mz={az}", flush=True)
        
    elif topic == MQTT_TOPICS['linear_acceleration']:
        
                # Split the payload by commas
        parts = payload.split(',')
        lx = float(parts[0].strip())
        ly = float(parts[1].strip())
        lz = float(parts[2].strip())
        
                # Create a point for InfluxDB with each value as a field
        point = (
            Point("linear_acceleration")
            .field("lx", lx)
            .field("ly", ly)
            .field("lz", lz)
            .time(int(time.time() * 1000), write_precision="ms")
        )
        
        # Write to InfluxDB
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        print(f"Linear Acceleration data written to InfluxDB: mx={lx}, my={ly}, mz={lz}", flush=True)
        

    elif topic == MQTT_TOPICS['gravity']:
        
                # Split the payload by commas
        parts = payload.split(',')
        grx = float(parts[0].strip())
        gry = float(parts[1].strip())
        grz = float(parts[2].strip())
        
                # Create a point for InfluxDB with each value as a field
        point = (
            Point("gravity")
            .field("grx", grx)
            .field("gry", gry)
            .field("grz", grz)
            .time(int(time.time() * 1000), write_precision="ms")
        )
        
        # Write to InfluxDB
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        print(f"Gravity data written to InfluxDB: mx={grx}, my={gry}, mz={grz}", flush=True)
        

    elif topic == MQTT_TOPICS['calibration']:
        pass

    elif topic == MQTT_TOPICS['voltage']:
        payload_value = float(payload)
        point = Point("voltage").field("value", payload_value).time(int(time.time() * 1000), write_precision="ms")
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        
    elif topic == MQTT_TOPICS['power']:
        payload_value = float(payload)
        point = Point("power").field("value", payload_value).time(int(time.time() * 1000), write_precision="ms")
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        
    elif topic == MQTT_TOPICS['current']:
        payload_value = float(payload)
        point = Point("current").field("value", payload_value).time(int(time.time() * 1000), write_precision="ms")
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        
    elif topic == MQTT_TOPICS['servo']:
        payload_value = float(payload)
        point = Point("servo").field("value", payload_value).time(int(time.time() * 1000), write_precision="ms")
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        
    print(f"Received message: {payload} on topic {topic}", flush=True)
    
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message  # Attach on_message callback

# Connect to MQTT broker and loop
mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
mqtt_client.loop_forever()



    
    









      



