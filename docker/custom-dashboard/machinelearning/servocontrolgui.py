import tkinter as tk
import paho.mqtt.client as mqtt

# MQTT setup
mqtt_broker = "192.168.1.208"
mqtt_topic = "wind_turbine/servo"
client = mqtt.Client()

def send_command(command):
    client.publish(mqtt_topic, command)

# GUI setup with tkinter
root = tk.Tk()
root.title("Servo Control")

# Left button to decrement angle
left_button = tk.Button(root, text="←", command=lambda: send_command(-15))
left_button.pack(side="left", padx=20, pady=20)

# Right button to increment angle
right_button = tk.Button(root, text="→", command=lambda: send_command(15))
right_button.pack(side="right", padx=20, pady=20)

# Connect to MQTT broker and start GUI
client.connect(mqtt_broker, 1883, 60)
client.loop_start()  # Start the MQTT client loop
root.mainloop()
