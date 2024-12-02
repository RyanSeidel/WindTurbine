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
rpm_readings = deque(maxlen=20)  # Stores last 20 RPM readings for smoothing
max_rpm = 150  # Cap RPM at 100
stored_rpm = 0  # Variable to store the last valid RPM value
timeout_threshold = 10  # Time in seconds to consider as idle (no movement)

def calculate_rpm(time_interval):
    # RPM = (1 pulse / time in seconds) * 60
    return (1 / time_interval) * 60 if time_interval > 0 else 0

def get_smoothed_rpm():
    if len(rpm_readings) == 0:
        return 0
    return sum(rpm_readings) / len(rpm_readings)

def publish_rpm(rpm):
    """Publish the RPM to the MQTT topic."""
    print(f"Publishing RPM: {rpm:.2f}")
    client.publish(mqtt_topic, rpm)

def check_sensor_status():
    global last_detection_time, stored_rpm

    while True:
        current_time = time.time()

        if hall_sensor.is_pressed:
            # Update the last detection time
            if last_detection_time is None:
                last_detection_time = current_time
            else:
                # Calculate time difference between detections
                time_interval = current_time - last_detection_time
                last_detection_time = current_time

                # Calculate RPM
                rpm = calculate_rpm(time_interval)

                # Cap the RPM to avoid unrealistic values
                if rpm > max_rpm:
                    rpm = stored_rpm  # Use the last valid RPM if the current one is too high
                else:
                    stored_rpm = rpm  # Update the last valid RPM

                # Add RPM to deque for smoothing
                rpm_readings.append(stored_rpm)
                smoothed_rpm = get_smoothed_rpm()

                # Publish the smoothed RPM value
                publish_rpm(smoothed_rpm)
        else:
            # If no detection within the timeout threshold, set RPM to 0
            if last_detection_time and (current_time - last_detection_time > timeout_threshold):
                last_detection_time = None
                rpm_readings.clear()  # Clear RPM readings to reset smoothing
                publish_rpm(0)  # Publish RPM as 0

        time.sleep(0.1)  # Adjust the delay as needed for responsiveness

try:
    check_sensor_status()
except KeyboardInterrupt:
    print("Exiting...")
    client.disconnect()
