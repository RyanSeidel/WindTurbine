import os
import time
from gpiozero import Button
import paho.mqtt.client as mqtt
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# Setup for Hall effect sensor
hall_sensor = Button(4)  # Assuming your sensor is connected to GPIO4

# MQTT setup
mqtt_broker = "localhost"  # Replace with your MQTT broker IP
mqtt_topic = "wind_turbine/rpm"
client = mqtt.Client()

# Connect to the MQTT broker
client.connect(mqtt_broker, 1883, 60)
client.loop_start()

# InfluxDB Configuration
INFLUXDB_URL = os.getenv("INFLUXDB_URL", "http://localhost:8086")
INFLUXDB_TOKEN = os.getenv("INFLUXDB_TOKEN", "YK5PVqE0aI9pwLBdZMDE5qt_jDvfYv4m2psX6tTQ13unsruDRf8JSzRq2y1cKgVastehinPYlgpDTNu0x0zQ2g==")
INFLUXDB_ORG = os.getenv("INFLUXDB_ORG", "TAMUCC")
INFLUXDB_BUCKET = os.getenv("INFLUXDB_BUCKET", "WindTurbine")

# Initialize the InfluxDB client
influx_client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
write_api = influx_client.write_api(write_options=SYNCHRONOUS)

# Variables for RPM calculation
last_detection_time = None
timeout_threshold = 10  # Seconds of no detection before reporting 0 RPM
max_rpm = 150  # Maximum realistic RPM value
debounce_time = 0.01  # Minimum time between detections (seconds)

def calculate_rpm(time_interval):
    """Calculate RPM from time between pulses, with sanity checks."""
    if time_interval <= 0:
        return 0
    
    rpm = (1 / time_interval) * 60
    
    # Apply reasonable limits
    if rpm > max_rpm:
        return max_rpm
    return rpm

def write_to_influxdb(rpm_value):
    """Write RPM data to InfluxDB with timestamp."""
    point = Point("rpm_measurement") \
        .tag("source", "hall_sensor") \
        .field("rpm", rpm_value) \
        .time(time.time_ns(), write_precision="ns")
    
    try:
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
    except Exception as e:
        print(f"Error writing to InfluxDB: {e}")

def publish_rpm(rpm):
    """Publish the RPM to both MQTT and InfluxDB."""
    print(f"Publishing RPM: {rpm:.2f}")
    client.publish(mqtt_topic, rpm)
    write_to_influxdb(rpm)

def get_readable_time():
    """Return current time in human-readable format."""
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

def check_sensor_status():
    global last_detection_time
    
    print("Starting RPM monitoring...")
    print(f"Current time: {get_readable_time()}")
    
    while True:
        try:
            # Wait for magnet detection with timeout
            detected = hall_sensor.wait_for_press(timeout=timeout_threshold)
            current_time = time.time()
            
            if detected:
                if last_detection_time is not None:
                    time_interval = current_time - last_detection_time
                    
                    # Only process if we've passed the debounce time
                    if time_interval > debounce_time:
                        rpm = calculate_rpm(time_interval)
                        publish_rpm(rpm)
                        print(f"Detection at {get_readable_time()} - RPM: {rpm:.2f}")
                
                last_detection_time = current_time
                hall_sensor.wait_for_release()  # Wait for magnet to pass
            else:
                # Timeout occurred - no detections
                if last_detection_time and (current_time - last_detection_time > timeout_threshold):
                    print(f"No detection - setting RPM to 0 at {get_readable_time()}")
                    publish_rpm(0)
                    last_detection_time = None
                    
        except Exception as e:
            print(f"Sensor error: {e}")
            time.sleep(1)  # Brief pause if something goes wrong

try:
    check_sensor_status()
except KeyboardInterrupt:
    print("\nExiting...")
    client.disconnect()
    influx_client.close()
    print("Cleanup complete")