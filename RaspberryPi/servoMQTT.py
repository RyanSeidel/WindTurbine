from gpiozero import AngularServo
from time import sleep
import paho.mqtt.client as mqtt

# Initialize the servo on GPIO 14
servo = AngularServo(17, min_angle=0, max_angle=360, min_pulse_width=0.0004, max_pulse_width=0.0025)

# Set initial servo angle
servo.angle = 0
current_angle = 0  # Track the current angle of the servo

# MQTT setup
mqtt_broker = "localhost"  # Replace with the Raspberry Pi IP if needed
mqtt_topic = "wind_turbine/servo"
client = mqtt.Client()

def on_message(client, userdata, msg):
    global current_angle
    try:
        # Decode the received message (it should be a number representing degrees)
        command = float(msg.payload.decode().strip())
        print(f"Received command: {command}")

        # Incremental adjustment (relative)
        if -360 <= command <= 360:
            new_angle = current_angle + command  # Move by the amount in `command`

            # Clamp the new angle within 0 to 360 degrees
            new_angle = max(0, min(360, new_angle))
            print(f"Moving servo to new angle: {new_angle} degrees")

            # Set the servo to the new angle and update the current angle
            servo.angle = new_angle
            current_angle = new_angle  # Update the current angle

        # Sleep for 1 second to give the servo time to reach the position
        sleep(1)

    except ValueError:
        print("Invalid command or angle value received")

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker")
    client.subscribe(mqtt_topic)

# MQTT configuration
client.on_connect = on_connect
client.on_message = on_message

# Connect to the MQTT broker
client.connect(mqtt_broker, 1883, 60)
client.loop_forever()
