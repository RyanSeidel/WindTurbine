#!/usr/bin/env python

import time
import paho.mqtt.client as mqtt

# MQTT broker settings
mqtt_broker = "localhost"  # Replace with the IP address of your MQTT broker if it's not on the same machine
mqtt_rpm_topic = "wind_turbine/rpm"
mqtt_temp_topic = "wind_turbine/temperature"

# Set up the MQTT client
client = mqtt.Client()

# Connect to the MQTT broker
client.connect(mqtt_broker, 1883, 60)
client.loop_start()

def sensor_data_publisher():
    while True:
        try:
            # Replace these static values with actual sensor readings
            rpm = 1500.0  # Simulated RPM value (replace with real sensor data)
            temperature = 85.5  # Simulated temperature value (replace with real sensor data)

            # Publish the sensor data to the MQTT topics
            client.publish(mqtt_rpm_topic, rpm)
            client.publish(mqtt_temp_topic, temperature)

            print(f"Published RPM: {rpm}, Temperature: {temperature}")

            # Sleep for a while before publishing the next set of values (adjust the rate as needed)
            time.sleep(1)  # Adjust the delay as per your requirements (e.g., 1 second interval)

        except Exception as e:
            print(f"An error occurred: {e}")
            break

if __name__ == '__main__':
    try:
        sensor_data_publisher()
    except KeyboardInterrupt:
        print("Exiting...")
        client.disconnect()
