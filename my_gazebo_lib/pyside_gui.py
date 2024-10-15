import sys
import paho.mqtt.client as mqtt
from PySide2.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QPushButton, QWidget, QTextEdit
from PySide2.QtCore import QTimer
import cv2
from PySide2.QtGui import QImage, QPixmap
from PySide2.QtWidgets import QLabel

class DigitalTwinGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Wind Turbine Digital Twin')

        # Set window size (wider rectangular window for your setup)
        self.resize(2560, 1080)

        # Main widget and layout
        main_widget = QWidget()
        layout = QVBoxLayout()

        # Labels for displaying sensor data
        self.rpm_label = QLabel("RPM: N/A")
        self.temp_label = QLabel("Temperature: N/A")
        self.wind_dir_label = QLabel("Wind Direction: N/A")
        self.wind_speed_label = QLabel("Wind Speed: N/A")
        self.voltage_label = QLabel("Voltage: N/A")
        self.current_label = QLabel("Current: N/A")
        self.vibration_label = QLabel("Vibration: N/A")
        self.warning_label = QTextEdit("Warnings/Predictions: None")
        self.warning_label.setReadOnly(True)

        # Camera view (for capturing real-time feed)
        self.camera_label = QLabel("Camera Feed")
        self.camera_label.setFixedSize(640, 480)  # Adjust as needed

        # Add labels to the layout
        layout.addWidget(self.rpm_label)
        layout.addWidget(self.temp_label)
        layout.addWidget(self.wind_dir_label)
        layout.addWidget(self.wind_speed_label)
        layout.addWidget(self.voltage_label)
        layout.addWidget(self.current_label)
        layout.addWidget(self.vibration_label)
        layout.addWidget(self.warning_label)
        layout.addWidget(self.camera_label)

        # Add buttons for controlling the servo at specific angles
        btn_0 = QPushButton("Set Wind Direction: 0°")
        btn_90 = QPushButton("Set Wind Direction: 90°")
        btn_135 = QPushButton("Set Wind Direction: 270°")

        btn_0.clicked.connect(lambda: self.set_servo_angle(0))
        btn_90.clicked.connect(lambda: self.set_servo_angle(-45))
        btn_135.clicked.connect(lambda: self.set_servo_angle(270))

        layout.addWidget(btn_0)
        layout.addWidget(btn_90)
        layout.addWidget(btn_135)

        # Buttons for optional Gazebo control
        start_btn = QPushButton("Start Gazebo")
        start_btn.clicked.connect(self.start_gazebo)
        layout.addWidget(start_btn)

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

        # Setup camera feed using OpenCV
        self.camera = cv2.VideoCapture(0)  # Use 0 for the first webcam
        self.camera.set(cv2.CAP_PROP_FPS, 30)  # Set FPS to 30
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Reduce resolution
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_camera)
        self.timer.start(33)  # Update every 33 ms (approx 30 FPS)

        self.frame_count = 0  # Initialize frame counter

    def on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker with result code " + str(rc))
        # Subscribe to the MQTT topics for sensor data
        client.subscribe("wind_turbine/rpm")
        client.subscribe("wind_turbine/temperature")
        client.subscribe("wind_turbine/wind_direction")
        client.subscribe("wind_turbine/wind_speed")
        client.subscribe("wind_turbine/voltage")
        client.subscribe("wind_turbine/current")
        client.subscribe("wind_turbine/vibration")
        client.subscribe("wind_turbine/warnings")

    def on_message(self, client, userdata, msg):
        # Update GUI based on the received MQTT messages
        if msg.topic == "wind_turbine/rpm":
            rpm = float(msg.payload.decode())
            self.rpm_label.setText(f"RPM: {rpm:.2f}")

        elif msg.topic == "wind_turbine/temperature":
            temp = float(msg.payload.decode())
            self.temp_label.setText(f"Temperature: {temp:.2f}")

        elif msg.topic == "wind_turbine/wind_direction":
            wind_dir = msg.payload.decode()
            self.wind_dir_label.setText(f"Wind Direction: {wind_dir}")

        elif msg.topic == "wind_turbine/wind_speed":
            wind_speed = float(msg.payload.decode())
            self.wind_speed_label.setText(f"Wind Speed: {wind_speed:.2f} m/s")

        elif msg.topic == "wind_turbine/voltage":
            voltage = float(msg.payload.decode())
            self.voltage_label.setText(f"Voltage: {voltage:.2f} V")

        elif msg.topic == "wind_turbine/current":
            current = float(msg.payload.decode())
            self.current_label.setText(f"Current: {current:.2f} A")

        elif msg.topic == "wind_turbine/vibration":
            vibration = float(msg.payload.decode())
            self.vibration_label.setText(f"Vibration: {vibration:.2f} m/s^2")

        elif msg.topic == "wind_turbine/warnings":
            warnings = msg.payload.decode()
            self.warning_label.setPlainText(f"Warnings/Predictions: {warnings}")

    def set_servo_angle(self, angle):
        print(f"Setting wind direction angle: {angle}°")
        # Publish the angle to the MQTT topic
        self.client.publish("wind_turbine/servo", angle)

    def update_camera(self):
        self.frame_count += 1
        # Capture frame-by-frame from the webcam
        ret, frame = self.camera.read()
        if ret and self.frame_count % 2 == 0:  # Skip every other frame
            # Convert the frame to a QImage to display in the QLabel
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame_rgb.shape
            bytes_per_line = ch * w
            q_img = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.camera_label.setPixmap(QPixmap.fromImage(q_img))

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
