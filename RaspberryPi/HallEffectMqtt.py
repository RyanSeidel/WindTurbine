import os
import time
from gpiozero import Button
import paho.mqtt.client as mqtt
from collections import deque

# Setup for Hall effect sensor
hall_sensor = Button(4)

# MQTT setup
mqtt_broker = "localhost"  # Replace with the IP address of your MQTT broker
mqtt_topic = "wind_turbine/rpm"
client = mqtt.Client()

# Connect to the MQTT broker
client.connect(mqtt_broker, 1883, 60)
client.loop_start()

# Variables for RPM calculation
last_detection_time = None
rpm_readings = deque(maxlen=20)  # Stores last 5 RPM readings for smoothing

def calculate_rpm(time_interval):
    # RPM = (1 pulse / time in seconds) * 60
    return (1 / time_interval) * 60 if time_interval > 0 else 0

def get_smoothed_rpm():
    if len(rpm_readings) == 0:
        return 0
    return sum(rpm_readings) / len(rpm_readings)

def check_sensor_status():
    global last_detection_time

    while True:
        if hall_sensor.is_pressed:
            current_time = time.time()

            if last_detection_time is None:  # First detection, set the last detection time
                last_detection_time = current_time
            else:
                # Calculate time difference between the current and last detection
                time_interval = current_time - last_detection_time
                last_detection_time = current_time

                # Calculate and store RPM
                rpm = calculate_rpm(time_interval)
                rpm_readings.append(rpm)  # Add to deque for smoothing
                smoothed_rpm = get_smoothed_rpm()
                print(f"RPM (smoothed): {smoothed_rpm:.2f}")

                # Publish the smoothed RPM value to MQTT topic
                client.publish(mqtt_topic, smoothed_rpm)

        time.sleep(0.1)  # Adjust the delay as needed for responsiveness

try:
    check_sensor_status()
except KeyboardInterrupt:
    print("Exiting...")
    client.disconnect()
