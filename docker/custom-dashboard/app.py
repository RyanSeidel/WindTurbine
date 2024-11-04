from flask import Flask, render_template, jsonify, request
from influxdb_client import InfluxDBClient
from flask_socketio import SocketIO
import paho.mqtt.client as mqtt
import os
import time

app = Flask(__name__)
socketio = SocketIO(app)

# InfluxDB Configuration
INFLUXDB_URL = os.getenv("INFLUXDB_URL", "http://influxdb:8086")
INFLUXDB_TOKEN = os.getenv("INFLUXDB_TOKEN", "iNLROvcnYQmb6CNVmUyrNuB6CG2EiKOjUrT-F13uF-x1pSYLZGcGS-rbgj9J1cS-zaUwMB6UPd8_SJgVl3KFdQ==")
INFLUXDB_ORG = os.getenv("INFLUXDB_ORG", "TAMUCC")
INFLUXDB_BUCKET = os.getenv("INFLUXDB_BUCKET", "WindTurbine")

# Initialize the InfluxDB client
client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)

# MQTT Configuration
MQTT_BROKER = os.getenv("MQTT_BROKER", "192.168.1.208")  # Replace with your MQTT broker address
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
    'current': 'wind_turbine/current'
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
        socketio.emit('rpm_data', {'latest_rpm': float(payload)})
    elif topic == MQTT_TOPICS['temperature']:
        socketio.emit('temperature_data', {'temperature': float(payload)})
    elif topic == MQTT_TOPICS['orientation']:
        socketio.emit('orientation_data', {'orientation': payload})
    elif topic == MQTT_TOPICS['magnetometer']:
        socketio.emit('magnetometer_data', {'magnetometer': payload})
    elif topic == MQTT_TOPICS['gyroscope']:
        socketio.emit('gyroscope_data', {'gyroscope': payload})
    elif topic == MQTT_TOPICS['accelerometer']:
        socketio.emit('accelerometer_data', {'accelerometer': payload})
    elif topic == MQTT_TOPICS['linear_acceleration']:
        socketio.emit('linear_acceleration_data', {'linear_acceleration': payload})
    elif topic == MQTT_TOPICS['gravity']:
        socketio.emit('gravity_data', {'gravity': payload})
    elif topic == MQTT_TOPICS['calibration']:
        socketio.emit('calibration_data', {'calibration': payload})
    elif topic == MQTT_TOPICS['voltage']:
        socketio.emit('voltage_data', {'voltage': float(payload)})
    elif topic == MQTT_TOPICS['power']:
        socketio.emit('power_data', {'power': float(payload)})
    elif topic == MQTT_TOPICS['current']:
        socketio.emit('current_data', {'current': float(payload)})

    print(f"Received message: {payload} on topic {topic}", flush=True)
    
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message  # Attach on_message callback

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/socket')
def socket_page():
    return render_template('socket.html')

# @app.route('/api/data')
# def get_data():
#     try:
#         query = f'''
#         from(bucket: "{INFLUXDB_BUCKET}") 
#         |> range(start: -5m) 
#         |> filter(fn: (r) => r["_measurement"] == "RPM Measurement") 
#         |> filter(fn: (r) => r["_field"] == "rpm") 
#         |> keep(columns: ["_time", "_value"])
#         '''
#         tables = client.query_api().query(query=query)

#         # Extract the data from the query response
#         data = [{"_time": record["_time"], "_value": record["_value"]} for table in tables for record in table.records]

#         print("Data fetched from InfluxDB:", data)  # Log the fetched data

#         if data:
#             return jsonify(data)  # Return the data as JSON
#         else:
#             return jsonify({"status": "Connected to InfluxDB, but no data found"})

#     except Exception as e:
#         return jsonify({"status": "Failed to connect to InfluxDB", "error": str(e)})

# def fetch_and_emit_data():
#     while True:
#         print("Collecting data...")  # Log the collection process
#         query = f'''
#         from(bucket: "{INFLUXDB_BUCKET}") 
#         |> range(start: -5m) 
#         |> filter(fn: (r) => r["_measurement"] == "RPM Measurement") 
#         |> filter(fn: (r) => r["_field"] == "rpm") 
#         |> keep(columns: ["_time", "_value"])
#         '''
#         tables = client.query_api().query(query=query)
        
#         # Prepare the data for emitting
#         data = [
#             {
#                 "_time": record["_time"].isoformat(),  # Convert datetime to string
#                 "_value": record["_value"]
#             } for table in tables for record in table.records
#         ]
        
#         # Extract the latest RPM value
#         latest_rpm = data[-1]["_value"] if data else None  # Get the latest value if available

#         # Emit the data along with the latest RPM
#         socketio.emit('rpm_data', {'data': data, 'latest_rpm': latest_rpm})
#         print(f"Emitting data: {data}")  # Print emitted data to the console
#         time.sleep(1)  # Adjust the frequency as needed

@app.route('/connect-mqtt', methods=['POST'])
def connect_mqtt():
    data = request.json
    broker_ip = data.get('broker_ip')
    blade1_orientation = data.get('blade1_orientation')
    blade2_orientation = data.get('blade2_orientation')
    blade3_orientation = data.get('blade3_orientation')

    if broker_ip:
        print(f"Attempting to connect to MQTT Broker at {broker_ip}", flush=True)
        mqtt_client.connect(broker_ip, MQTT_PORT, 60)
        mqtt_client.loop_start()

        # Optionally, publish the blade orientations to specific MQTT topics
        mqtt_client.publish("wind_turbine/blade1_orientation", blade1_orientation)
        mqtt_client.publish("wind_turbine/blade2_orientation", blade2_orientation)
        mqtt_client.publish("wind_turbine/blade3_orientation", blade3_orientation)

        print(f"Blade 1 Orientation: {blade1_orientation}°")
        print(f"Blade 2 Orientation: {blade2_orientation}°")
        print(f"Blade 3 Orientation: {blade3_orientation}°")

        return jsonify({
            "status": "Connection started",
            "broker_ip": broker_ip,
            "blade1_orientation": blade1_orientation,
            "blade2_orientation": blade2_orientation,
            "blade3_orientation": blade3_orientation
        })
    else:
        return jsonify({"status": "Failed", "error": "No broker IP provided"}), 400


@socketio.on('connect')
def handle_connect():
    print("Client connected")
    # Start a background task to fetch and emit data
    #socketio.start_background_task(fetch_and_emit_data)





      



