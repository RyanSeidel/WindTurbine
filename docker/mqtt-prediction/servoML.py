import serial
import time
import os
import logging
import pandas as pd
import numpy as np
import json  
import paho.mqtt.client as mqtt
import joblib


# So this program would have to be contain in a MQTT to subscribe to the events 
# MQTT Configuration
MQTT_BROKER = os.getenv("MQTT_BROKER", "mosquitto")  # Default: Mosquitto broker
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
RPS_INPUT_TOPIC = "rpsinputform"  # Topic for listening to form data
PREDICTION_TOPIC = "rps_predictions"  # Topic for publishing predictions

# Topics under wind_turbine namespace
MQTT_TOPICS = {
    'wind_direction': 'wind_turbine/wind_direction',
    'rpm': 'wind_turbine/rpm',
    'voltage': 'wind_turbine/voltage',
    'anamoly': 'wind_turbine/anomaly_predictions'
}

#Collect the Data moving the servo orientation from 15 30 45 60 75 90
# We know 90 by default is not what we want the orientation away we do simple math equation with wind direction and orientation to check for 90



# Once subscribed then it can use ML to take inputs like Anamoly, Volts, RPM
# Based on this data, we would be able to use a simple linear relationships to change decisions

# Replace 'COMx' with your ESP32 Bluetooth COM port (e.g., COM5 on Windows, /dev/rfcomm0 on Linux)
SERIAL_PORT = "COM6"  
BAUD_RATE = 115200

def send_angle(angle):
    """Send an angle value to ESP32 over Bluetooth serial."""
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            ser.write(f"{angle}\n".encode())  # Send angle with newline
            print(f"Sent: {angle}")
            time.sleep(1)  # Small delay to allow ESP32 to process
    except Exception as e:
        print(f"Error: {e}")

def main():
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    
    while True:
        angle = input("Enter angle (1-180) or 'exit' to quit: ").strip()
        
        if angle.lower() == "exit":
            print("Exiting...")
            break

        if angle.isdigit():
            angle = int(angle)
            if 1 <= angle <= 180:
                send_angle(angle)
            else:
                print("Invalid range! Enter a number between 1 and 180.")
        else:
            print("Invalid input! Enter a valid number.")

if __name__ == "__main__":
    main()
