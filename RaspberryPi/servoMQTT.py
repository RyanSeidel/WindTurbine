import paho.mqtt.client as mqtt
from gpiozero import AngularServo
from time import sleep
import time

# Set up MQTT client
mqtt_client = mqtt.Client()
mqtt_broker = "localhost"  # Use 'localhost' if the broker is on the same device; replace with IP if remote
mqtt_client.connect(mqtt_broker, 1883, 60)  # Connect to the MQTT broker


time.sleep(2)  # Wait for the connection to initialize

servo = AngularServo(17, min_angle=0, max_angle=360, min_pulse_width=0.0005, max_pulse_width=0.0025)
try:
    while True:
        # Set servo to each angle and hold for 10 seconds
        for angle in [0]:  # Use only 1 angle at a time
            servo.angle = angle
            print(f"Setting angle to {angle}")
            sleep(5)  # Hold each angle for 10 seconds

            # Publish the angle to the MQTT topic
            mqtt_client.publish("wind_turbine/servo", angle)  


except KeyboardInterrupt:
    print("Program interrupted")
finally:
    mqtt_client.disconnect()  # Disconnect from the MQTT broker
