from flask import Flask, render_template, jsonify
from influxdb_client import InfluxDBClient
from flask_socketio import SocketIO
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

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/socket')
def socket_page():
    return render_template('socket.html')

@app.route('/api/data')
def get_data():
    try:
        query = f'''
        from(bucket: "{INFLUXDB_BUCKET}") 
        |> range(start: -5m) 
        |> filter(fn: (r) => r["_measurement"] == "RPM Measurement") 
        |> filter(fn: (r) => r["_field"] == "rpm") 
        |> keep(columns: ["_time", "_value"])
        '''
        tables = client.query_api().query(query=query)

        # Extract the data from the query response
        data = [{"_time": record["_time"], "_value": record["_value"]} for table in tables for record in table.records]

        print("Data fetched from InfluxDB:", data)  # Log the fetched data

        if data:
            return jsonify(data)  # Return the data as JSON
        else:
            return jsonify({"status": "Connected to InfluxDB, but no data found"})

    except Exception as e:
        return jsonify({"status": "Failed to connect to InfluxDB", "error": str(e)})

def fetch_and_emit_data():
    while True:
        print("Collecting data...")  # Log the collection process
        query = f'''
        from(bucket: "{INFLUXDB_BUCKET}") 
        |> range(start: -5m) 
        |> filter(fn: (r) => r["_measurement"] == "RPM Measurement") 
        |> filter(fn: (r) => r["_field"] == "rpm") 
        |> keep(columns: ["_time", "_value"])
        '''
        tables = client.query_api().query(query=query)
        
        # Prepare the data for emitting
        data = [
            {
                "_time": record["_time"].isoformat(),  # Convert datetime to string
                "_value": record["_value"]
            } for table in tables for record in table.records
        ]
        
        # Extract the latest RPM value
        latest_rpm = data[-1]["_value"] if data else None  # Get the latest value if available

        # Emit the data along with the latest RPM
        socketio.emit('rpm_data', {'data': data, 'latest_rpm': latest_rpm})
        print(f"Emitting data: {data}")  # Print emitted data to the console
        time.sleep(1)  # Adjust the frequency as needed



@socketio.on('connect')
def handle_connect():
    print("Client connected")
    # Start a background task to fetch and emit data
    socketio.start_background_task(fetch_and_emit_data)
    





      



