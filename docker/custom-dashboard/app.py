#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
#                            Wind Turbine Digital Twins                     #
#                               By Ryan Seidel and Zac Castaneda            #
#                          Client: Dr. Jose Baca                            #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# Description:                                                              #
# This code implements a digital twin for a wind turbine system using      #
# Flask as the web framework, Socket.IO for real-time data communication,   #
# and InfluxDB for storing and analyzing sensor data. The MQTT protocol     #
# is used for transmitting data from Raspberry Pi sensors, including RPM,  #
# orientation, temperature, voltage, and more. A web-based dashboard        #
# provides visualization and control of the wind turbine's performance,     #
# enabling predictions and analysis of wind turbine behavior.               #
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

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

# Connect to the InfluxDB container
INFLUXDB_URL = os.getenv("INFLUXDB_URL", "http://influxdb:8086")
INFLUXDB_TOKEN = os.getenv("INFLUXDB_TOKEN", "iNLROvcnYQmb6CNVmUyrNuB6CG2EiKOjUrT-F13uF-x1pSYLZGcGS-rbgj9J1cS-zaUwMB6UPd8_SJgVl3KFdQ==")
INFLUXDB_ORG = os.getenv("INFLUXDB_ORG", "TAMUCC")
INFLUXDB_BUCKET = os.getenv("INFLUXDB_BUCKET", "WindTurbine")

# InfluxDB write API for data storage
influx_client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
write_api = influx_client.write_api(write_options=SYNCHRONOUS)  

<<<<<<< Updated upstream
# MQTT Configuration
MQTT_BROKER = os.getenv("MQTT_BROKER", "192.168.1.208")  # Replace with your MQTT broker address
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
# Topics under wind_turbine namespace
MQTT_TOPICS = {
=======
# MQTT Configuration for Raspberry Pi
RASP_BROKER = os.getenv("RASP_BROKER")  # Ensure this is always set in the environment
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))  # Defaults to 1883 if not set

# Model Prediction
MQTT_BROKER = os.getenv("MQTT_BROKER")
PREDICTION_TOPIC = "wind_turbine/predictions"


# These are all the topics of all the data that we are collecting from the Raspberry Pi 5 and Weather Station (Lark)
DATA_TOPICS = {
>>>>>>> Stashed changes
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
        for topic in DATA_TOPICS.values():
            client.subscribe(topic)
        print("Subscribed to all wind_turbine topics", flush=True)
    else:
        print(f"Failed to connect, return code {rc}", flush=True)
    
# MQTT on_message callback to handle incoming messages for each topic
def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode()
    
    
    # Emit data based on topic
<<<<<<< Updated upstream
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
        
=======
    if topic == DATA_TOPICS['rpm']:
        socketio.emit('rpm_data', {'latest_rpm': float(payload)})
        #print(f"Socket.IO: Emitted 'rpm_data' with value {float(payload)}", flush=True)
    elif topic == DATA_TOPICS['temperature']:
        socketio.emit('temperature_data', {'temperature': float(payload)})      
    elif topic == DATA_TOPICS['orientation']:          
        socketio.emit('orientation_data', {'orientation': payload})
    elif topic == DATA_TOPICS['magnetometer']:     
        socketio.emit('magnetometer_data', {'magnetometer': payload})      
    elif topic == DATA_TOPICS['gyroscope']:       
        socketio.emit('gyroscope_data', {'gyroscope': payload})
    elif topic == DATA_TOPICS['accelerometer']:              
        socketio.emit('accelerometer_data', {'accelerometer': payload})
    elif topic == DATA_TOPICS['linear_acceleration']:        
        socketio.emit('linear_acceleration_data', {'linear_acceleration': payload})
    elif topic == DATA_TOPICS['gravity']:       
>>>>>>> Stashed changes
        socketio.emit('gravity_data', {'gravity': payload})
    elif topic == DATA_TOPICS['calibration']:
        socketio.emit('calibration_data', {'calibration': payload})
<<<<<<< Updated upstream
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

    print(f"Received message: {payload} on topic {topic}", flush=True)
    
=======
    elif topic == DATA_TOPICS['voltage']:       
        socketio.emit('voltage_data', {'voltage': float(payload)})
    elif topic == DATA_TOPICS['power']:
        socketio.emit('power_data', {'power': float(payload)})
    elif topic == DATA_TOPICS['current']:    
        socketio.emit('current_data', {'current': float(payload)})        
    elif topic == DATA_TOPICS['servo']:
        socketio.emit('servo_data', {'servo': float(payload)})
    elif topic == DATA_TOPICS['speed']:
        #print(f"Speed Payload: {payload}", flush=True)  # Debug: Print the payload
        socketio.emit('speed_data', {'speed': float(payload)})
    elif topic == DATA_TOPICS['direction']:
        #print(f"Direction Payload: {payload}", flush=True)  # Debug: Print the payload
        socketio.emit('direction_data', {'direction': int(payload)})
    elif topic == DATA_TOPICS['pressure']:
        #print(f"Direction Payload: {payload}", flush=True)  # Debug: Print the payload
        socketio.emit('pressure_data', {'pressure': float(payload)})
    elif topic == DATA_TOPICS['humidity']:     
        socketio.emit('humidity_data', {'humidity': float(payload)})
    elif topic == DATA_TOPICS['altitude']:      
        socketio.emit('altitude_data', {'altitude': float(payload)})
    elif topic == DATA_TOPICS['altitude']:      
        socketio.emit('altitude_data', {'altitude': float(payload)})
    elif topic == DATA_TOPICS['predictedRPS']:      
        # Example payload: "{'predictedRPS': np.float64(2.492183260094821)}"
        try:
            # Parse the payload
            data = eval(payload, {"np": np})  # Allow eval to recognize 'np'
            predicted_rps = data.get('predictedRPS', 0.0)
        
            # Convert to RPM
            predicted_rpm = predicted_rps * 60
        
            # Log and display the formatted output
            print(f"Predicted RPS: {predicted_rps:.2f}, Converted RPM: {predicted_rpm:.2f}", flush=True)
        except Exception as e:
            print(f"Error processing predictedRPS payload: {e}", flush=True)
 
>>>>>>> Stashed changes
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message  # Attach on_message callback

<<<<<<< Updated upstream
=======
# Attach callbacks to the prediction client
prediction_client.on_connect = on_connect
prediction_client.on_message = on_message

# # ----- CONNECT BOTH CLIENTS -----
try:
    #Connect to Raspberry Pi MQTT broker
    mqtt_client.connect(RASP_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_start()
    logging.info("Raspberry Pi MQTT client started.")

    # Connect to Mosquitto MQTT broker
    prediction_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    prediction_client.loop_start()
    logging.info("Mosquitto MQTT client started.")
except Exception as e:
    logging.error(f"Error connecting to MQTT brokers: {e}")
 
 
# This come from the RPS Prediction Input and publishes the data to topic: rpsinputform
@app.route('/submit_prediction', methods=['POST'])
def submit_prediction():
    try:
   
        data = request.get_json()
        logging.info(f"Received data: {data}")  # Log the received data

        wind_speed = float(data.get('windSpeed', 0))
        wind_direction = data.get('windDirection', 'N')
        orientation = int(data.get('orientation', 0))
        blade_angle = int(data.get('blade1', 15))

        # Mapping for wind directions to numeric values
        wind_direction_mapping = {
            'N': 0, 'NE': 45, 'E': 90, 'SE': 135,
            'S': 180, 'SW': 225, 'W': 270, 'NW': 315
        }
        wind_direction_numeric = wind_direction_mapping.get(wind_direction, 0)

        logging.info(f"Parsed windSpeed: {wind_speed}, windDirection: {wind_direction_numeric}, orientation: {orientation}, bladeAngle: {blade_angle}")

        # Prepare payload and publish to MQTT
        payload = {
            'windSpeed': wind_speed,
            'windDirection': wind_direction_numeric,
            'orientation': orientation,
            'bladeAngle': blade_angle
        }
        prediction_client.publish('rpsinputform', str(payload))
        logging.info(f"Published prediction data: {payload}")

        return jsonify({'status': 'success', 'message': 'Prediction data sent via MQTT!'})
    except Exception as e:
        logging.error(f"Error in /submit_prediction: {e}")
        return jsonify({'status': 'error', 'message': 'Failed to send prediction data'}), 500

    
>>>>>>> Stashed changes
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





      



