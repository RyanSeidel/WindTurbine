from flask import Flask, render_template, jsonify, request
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from flask_socketio import SocketIO
import paho.mqtt.client as mqtt
#from machinelearning.lstm_model import predict_rpm_volts
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
influx_client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
write_api = influx_client.write_api(write_options=SYNCHRONOUS)  # This line initializes write_api

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
    'current': 'wind_turbine/current',
    'servo': 'wind_turbine/servo',
    'speed': 'wind_turbine/speed',
    'direction': 'wind_turbine/direction',
    'pressure': 'wind_turbine/pressure',
    'humidity': 'wind_turbine/humidity',
    'altitude': 'wind_turbine/altitude'
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
        # payload_value = float(payload)
        # point = Point("rpm").field("value", payload_value).time(int(time.time() * 1000), write_precision="ms")
        # write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        socketio.emit('rpm_data', {'latest_rpm': float(payload)})
    elif topic == MQTT_TOPICS['temperature']:
        # payload_value = float(payload)
        # point = Point("temperature").field("value", payload_value).time(int(time.time() * 1000), write_precision="ms")
        # write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        socketio.emit('temperature_data', {'temperature': float(payload)})
        
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
           
        socketio.emit('orientation_data', {'orientation': payload})
    elif topic == MQTT_TOPICS['magnetometer']:
        
        # # Split the payload by commas
        # parts = payload.split(',')
        # mx = float(parts[0].strip())
        # my = float(parts[1].strip())
        # mz = float(parts[2].strip())
        
        #         # Create a point for InfluxDB with each value as a field
        # point = (
        #     Point("magnetometer")
        #     .field("mx", mx)
        #     .field("my", my)
        #     .field("mz", mz)
        #     .time(int(time.time() * 1000), write_precision="ms")
        # )
        
        # # Write to InfluxDB
        # write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        # print(f"Magnetometer data written to InfluxDB: mx={mx}, my={my}, mz={mz}", flush=True)
        
        socketio.emit('magnetometer_data', {'magnetometer': payload})
        
    elif topic == MQTT_TOPICS['gyroscope']:
        
        # # Split the payload by commas
        # parts = payload.split(',')
        # gx = float(parts[0].strip())
        # gy = float(parts[1].strip())
        # gz = float(parts[2].strip())
        
        #         # Create a point for InfluxDB with each value as a field
        # point = (
        #     Point("gyroscope")
        #     .field("gx", gx)
        #     .field("gy", gy)
        #     .field("gz", gz)
        #     .time(int(time.time() * 1000), write_precision="ms")
        # )
        
        # # Write to InfluxDB
        # write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        # print(f"Gyroscope data written to InfluxDB: mx={gx}, my={gy}, mz={gz}", flush=True)
        
        socketio.emit('gyroscope_data', {'gyroscope': payload})
    elif topic == MQTT_TOPICS['accelerometer']:
        
        # # Split the payload by commas
        # parts = payload.split(',')
        # ax = float(parts[0].strip())
        # ay = float(parts[1].strip())
        # az = float(parts[2].strip())
        
        #         # Create a point for InfluxDB with each value as a field
        # point = (
        #     Point("accelerometer")
        #     .field("ax", ax)
        #     .field("ay", ay)
        #     .field("az", az)
        #     .time(int(time.time() * 1000), write_precision="ms")
        # )
        
        # # Write to InfluxDB
        # # write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        # print(f"Accelerometer data written to InfluxDB: mx={ax}, my={ay}, mz={az}", flush=True)
        
        socketio.emit('accelerometer_data', {'accelerometer': payload})
    elif topic == MQTT_TOPICS['linear_acceleration']:
        
        #         # Split the payload by commas
        # parts = payload.split(',')
        # lx = float(parts[0].strip())
        # ly = float(parts[1].strip())
        # lz = float(parts[2].strip())
        
        #         # Create a point for InfluxDB with each value as a field
        # point = (
        #     Point("linear_acceleration")
        #     .field("lx", lx)
        #     .field("ly", ly)
        #     .field("lz", lz)
        #     .time(int(time.time() * 1000), write_precision="ms")
        # )
        
        # # Write to InfluxDB
        # write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        # print(f"Linear Acceleration data written to InfluxDB: mx={lx}, my={ly}, mz={lz}", flush=True)
        
        socketio.emit('linear_acceleration_data', {'linear_acceleration': payload})
    elif topic == MQTT_TOPICS['gravity']:
        
        #         # Split the payload by commas
        # parts = payload.split(',')
        # grx = float(parts[0].strip())
        # gry = float(parts[1].strip())
        # grz = float(parts[2].strip())
        
        #         # Create a point for InfluxDB with each value as a field
        # point = (
        #     Point("gravity")
        #     .field("grx", grx)
        #     .field("gry", gry)
        #     .field("grz", grz)
        #     .time(int(time.time() * 1000), write_precision="ms")
        # )
        
        # # Write to InfluxDB
        # write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        #print(f"Gravity data written to InfluxDB: mx={grx}, my={gry}, mz={grz}", flush=True)
        
        socketio.emit('gravity_data', {'gravity': payload})
    elif topic == MQTT_TOPICS['calibration']:
        socketio.emit('calibration_data', {'calibration': payload})
    elif topic == MQTT_TOPICS['voltage']:
        # payload_value = float(payload)
        # point = Point("voltage").field("value", payload_value).time(int(time.time() * 1000), write_precision="ms")
        #write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        
        socketio.emit('voltage_data', {'voltage': float(payload)})
    elif topic == MQTT_TOPICS['power']:
        # payload_value = float(payload)
        # point = Point("power").field("value", payload_value).time(int(time.time() * 1000), write_precision="ms")
        #write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        
        socketio.emit('power_data', {'power': float(payload)})
    elif topic == MQTT_TOPICS['current']:
        # payload_value = float(payload)
        # point = Point("current").field("value", payload_value).time(int(time.time() * 1000), write_precision="ms")
        # write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        
        socketio.emit('current_data', {'current': float(payload)})
        
    elif topic == MQTT_TOPICS['servo']:
        # payload_value = float(payload)
        # point = Point("current").field("value", payload_value).time(int(time.time() * 1000), write_precision="ms")
        # write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
        
        socketio.emit('servo_data', {'servo': float(payload)})

    elif topic == MQTT_TOPICS['speed']:
        print(f"Speed Payload: {payload}", flush=True)  # Debug: Print the payload
        socketio.emit('speed_data', {'speed': float(payload)})

    elif topic == MQTT_TOPICS['direction']:
        print(f"Direction Payload: {payload}", flush=True)  # Debug: Print the payload
        socketio.emit('direction_data', {'direction': int(payload)})

    elif topic == MQTT_TOPICS['pressure']:
        print(f"Direction Payload: {payload}", flush=True)  # Debug: Print the payload
        socketio.emit('pressure_data', {'pressure': float(payload)})

    elif topic == MQTT_TOPICS['humidity']:
       
        socketio.emit('humidity_data', {'humidity': float(payload)})

    elif topic == MQTT_TOPICS['altitude']:
        
        socketio.emit('altitude_data', {'altitude': float(payload)})

    print(f"Received message: {payload} on topic {topic}", flush=True)
    
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message  # Attach on_message callback



@app.route('/')
def dashboard():
    return render_template('dashboard.html')

@app.route('/analysis')
def analysis():
    return render_template('analysis.html')

@app.route('/settings')
def settings():
    return render_template('settings.html')




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
    
    
@app.route('/predict', methods=['GET', 'POST'])
def predict():
    if request.method == 'POST':
        # Get user inputs from form
        wind_speed = float(request.form.get('wind_speed'))
        wind_direction = float(request.form.get('wind_direction'))
        temperature = float(request.form.get('temperature'))
        orientation = float(request.form.get('orientation'))

        # Call the LSTM prediction function
        predicted_rpm, predicted_volts = predict_rpm_volts(wind_speed, wind_direction, temperature, orientation)

        # Return the results as JSON or render them in a template
        return jsonify({
            "predicted_rpm": predicted_rpm,
            "predicted_volts": predicted_volts
        })

    # Display the prediction form if GET request
    return render_template('predict_form.html')


@socketio.on('connect')
def handle_connect():
    print("Client connected")
    # Start a background task to fetch and emit data
    #socketio.start_background_task(fetch_and_emit_data)





      



