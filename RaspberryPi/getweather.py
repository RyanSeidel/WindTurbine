import sys
import os
import time
import paho.mqtt.client as mqtt

# Add the path to the DFRobot_LarkWeatherStation module
sys.path.append(os.path.expanduser("~/RaspberryPi/DFRobot_LarkWeatherStation/python/raspberry"))

from DFRobot_LarkWeatherStation import DFRobot_LarkWeatherStation_I2C

ADDRESS = 0x42
BROKER = "localhost"  # Replace with your MQTT broker address
PORT = 1883  # Default MQTT port
TOPIC_PREFIX = "wind_turbine"

# I2C mode
EDU0157 = DFRobot_LarkWeatherStation_I2C(ADDRESS)

# Map for cardinal and intermediate directions
direction_map = {
    "N": 1, "NE": 2, "E": 3, "SE": 4,
    "S": 5, "SW": 6, "W": 7, "NW": 8
}

# Initialize MQTT client
client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print(f"Failed to connect, return code {rc}")

def setup():
    # Initialize the sensor
    while EDU0157.begin() != 0:
        print("Sensor initialize failed!!")
        time.sleep(1)
    print("Sensor initialize success!!")
    EDU0157.set_time(2023, 1, 11, 23, 59, 0)
    time.sleep(1)

    # Connect to MQTT broker
    client.on_connect = on_connect
    client.connect(BROKER, PORT, 60)
    client.loop_start()

def loop():
    print("------------------")
    
    # Retrieve sensor data
    speed = EDU0157.get_value("Speed")
    direction = EDU0157.get_value("Dir")
    temperature = EDU0157.get_value("Temp")
    humidity = EDU0157.get_value("Humi")
    pressure = EDU0157.get_value("Pressure")
    altitude = EDU0157.get_value("Altitude")
    
    # Map direction to numeric value
    direction_map = {
        "N": 1, "NE": 2, "E": 3, "SE": 4,
        "S": 5, "SW": 6, "W": 7, "NW": 8
    }
    direction_numeric = direction_map.get(direction, -1)  # Default to -1 for invalid directions
    
    # Print data
    print(f"Speed: {speed} {EDU0157.get_unit('Speed')}")
    print(f"Direction: {direction} (Numeric: {direction_numeric})")
    print(f"Temperature: {temperature} {EDU0157.get_unit('Temp')}")
    print(f"Humidity: {humidity} {EDU0157.get_unit('Humi')}")
    print(f"Pressure: {pressure} {EDU0157.get_unit('Pressure')}")
    print(f"Altitude: {altitude} {EDU0157.get_unit('Altitude')}")
    
    # Publish data to MQTT
    client.publish(f"{TOPIC_PREFIX}/speed", speed)
    client.publish(f"{TOPIC_PREFIX}/direction", direction_numeric)
    client.publish(f"{TOPIC_PREFIX}/temperature", temperature)
    client.publish(f"{TOPIC_PREFIX}/humidity", humidity)
    client.publish(f"{TOPIC_PREFIX}/pressure", pressure)
    client.publish(f"{TOPIC_PREFIX}/altitude", altitude)


   

if __name__ == "__main__":
    try:
        setup()
        while True:
            loop()
    except KeyboardInterrupt:
        print("Exiting program...")
        client.loop_stop()
        client.disconnect()
