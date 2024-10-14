import sys
import paho.mqtt.client as mqtt
from PySide2.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QPushButton, QWidget

class DigitalTwinGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Digital Twin Control with MQTT')

        # Main widget and layout
        main_widget = QWidget()
        layout = QVBoxLayout()

        # Label for displaying sensor data
        self.rpm_label = QLabel("RPM: N/A")
        self.temp_label = QLabel("Temperature: N/A")
        layout.addWidget(self.rpm_label)
        layout.addWidget(self.temp_label)

        # Button to start Gazebo (optional)
        start_btn = QPushButton("Start Gazebo")
        start_btn.clicked.connect(self.start_gazebo)
        layout.addWidget(start_btn)

        # Button to reset Gazebo (optional)
        reset_btn = QPushButton("Reset Gazebo")
        reset_btn.clicked.connect(self.reset_gazebo)
        layout.addWidget(reset_btn)

        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)

        # Setup MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # Connect to the MQTT broker (Replace with Raspberry Pi's IP)
        self.client.connect("192.168.1.204", 1883, 60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker with result code " + str(rc))
        # Subscribe to the MQTT topics for sensor data
        client.subscribe("wind_turbine/rpm")
        client.subscribe("wind_turbine/temperature")

    def on_message(self, client, userdata, msg):
        # Decode and display RPM data
        if msg.topic == "wind_turbine/rpm":
            rpm = float(msg.payload.decode())
            self.rpm_label.setText(f"RPM: {rpm:.2f}")
        # Decode and display Temperature data
        elif msg.topic == "wind_turbine/temperature":
            temp = float(msg.payload.decode())
            self.temp_label.setText(f"Temperature: {temp:.2f}")

    def start_gazebo(self):
        # Optional: Implement Gazebo control here
        print("Gazebo start pressed")

    def reset_gazebo(self):
        # Optional: Implement Gazebo reset here
        print("Gazebo reset pressed")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = DigitalTwinGUI()
    gui.show()
    sys.exit(app.exec_())
