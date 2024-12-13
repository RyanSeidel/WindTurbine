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
import numpy as np
import json
import logging
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

# MQTT Configuration for Raspberry Pi
RASP_BROKER = os.getenv("RASP_BROKER")  # Ensure this is always set in the environment
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))  # Defaults to 1883 if not set

# Model Prediction
MQTT_BROKER = os.getenv("MQTT_BROKER", "mosquitto")
PREDICTION_TOPIC = "wind_turbine/predictions"

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
    'altitude': 'wind_turbine/altitude',
    'predictedRPS': 'rps_predictions',
    'anomaly': 'anomaly_predictions'
}

# Initialize MQTT client
mqtt_client = mqtt.Client()

prediction_client = mqtt.Client()

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
        #print(f"Socket.IO: Emitted 'rpm_data' with value {float(payload)}", flush=True)
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
    elif topic == MQTT_TOPICS['servo']:
        socketio.emit('servo_data', {'servo': float(payload)})
    elif topic == MQTT_TOPICS['speed']:
        socketio.emit('speed_data', {'speed': float(payload)})
    elif topic == MQTT_TOPICS['direction']:
        #print(f"Direction Payload: {payload}", flush=True)  # Debug: Print the payload
        socketio.emit('direction_data', {'direction': int(payload)})
    elif topic == MQTT_TOPICS['pressure']:
        print(f"Direction Payload: {payload}", flush=True)  # Debug: Print the payload
        socketio.emit('pressure_data', {'pressure': float(payload)})
    elif topic == MQTT_TOPICS['humidity']:     
        socketio.emit('humidity_data', {'humidity': float(payload)})
    elif topic == MQTT_TOPICS['altitude']:      
        socketio.emit('altitude_data', {'altitude': float(payload)})
    elif topic == MQTT_TOPICS['predictedRPS']:      
        # Example payload: "{'predictedRPS': np.float64(2.492183260094821)}"
        try:
            # Parse the payload
            data = eval(payload, {"np": np})  # Allow eval to recognize 'np'

            # Extract predicted RPS and Voltage
            predicted_rps = data.get('predictedRPS', 0.0)
            predicted_voltage = data.get('predictedVoltage', 0.0)
        
            # Convert RPS to RPM
            predicted_rpm = predicted_rps * 60

            # Log and display the formatted output
            print(f"Predicted RPS: {predicted_rps:.2f}, Converted RPM: {predicted_rpm:.2f}, Predicted Voltage: {predicted_voltage:.2f}", flush=True)
        
            # Emit data to the front end
            socketio.emit('predicted_rps', {
            'predictedRPS': round(predicted_rps, 2),  # RPS with 2 decimals
            'predictedRPM': round(predicted_rpm, 2),  # RPM with 2 decimals
            'predictedVoltage': round(predicted_voltage, 2)  # Voltage with 2 decimals
        })
        except Exception as e:
            print(f"Error processing predictedRPS payload: {e}", flush=True)
            
    elif topic == MQTT_TOPICS['anomaly']:  
        
        # Parse the incoming JSON payload
        data = json.loads(payload)

        # Extract magnitude-related fields
        predicted_magnitude = data.get('Predicted_Magnitude', 0.0)
        actual_magnitude = data.get('Actual_Magnitude', 0.0)
        magnitude_residual = data.get('Magnitude_Residual', 0.0)
        magnitude_threshold = data.get('Magnitude_Threshold', 0.0)
        magnitude_anomaly = data.get('Magnitude_Anomaly', False)

        # Extract RPM-related fields
        predicted_rpm = data.get('Predicted_RPM', 0.0)
        actual_rpm = data.get('Actual_RPM', 0.0)
        rpm_residual = data.get('RPM_Residual', 0.0)
        rpm_threshold = data.get('RPM_Threshold', 0.0)
        rpm_anomaly = data.get('RPM_Anomaly', False)

        # Log the extracted data for debugging
        logging.info(f"Received message: {json.dumps(data, indent=4)} from topic: {topic}")
        
        # Print the message to ensure visibility in the console
        print(f"Received message from topic {topic}: {json.dumps(data, indent=4)}")

        # Emit the parsed data for magnitude and RPM to the front-end via SocketIO
        socketio.emit('anomaly_data', {
            'Predicted_Magnitude': predicted_magnitude,
            'Actual_Magnitude': actual_magnitude,
            'Magnitude_Residual': magnitude_residual,
            'Magnitude_Threshold': magnitude_threshold,
            'Magnitude_Anomaly': magnitude_anomaly,
            'Predicted_RPM': predicted_rpm,
            'Actual_RPM': actual_rpm,
            'RPM_Residual': rpm_residual,
            'RPM_Threshold': rpm_threshold,
            'RPM_Anomaly': rpm_anomaly
        })
        
        
 
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message  

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
    
@app.route('/submit_prediction', methods=['POST'])
def submit_prediction():
    try:
        # Parse JSON payload
        data = request.get_json()
        logging.info(f"Received data: {data}")  # Log the received data

        # Extract form values
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

    
@app.route('/')
def dashboard():
    return render_template('dashboard.html')

@app.route('/analysis')
def analysis():
    return render_template('analysis.html')
 






      



