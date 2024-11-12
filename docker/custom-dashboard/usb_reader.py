import serial
import time

# Set up the serial connection (replace 'COM3' with your ESP32 port, e.g., '/dev/ttyUSB0' for Linux/Mac)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust 'COM3' to your port name
time.sleep(2)  # Wait for the connection to initialize

try:
    while True:
        if ser.in_waiting > 0:  # Check if there's incoming data
            angle_data = ser.readline().decode('utf-8').strip()  # Read and decode the angle data
            try:
                angle = int(angle_data)  # Convert to integer
                print(f"Current Angle: {angle}°")
            except ValueError:
                print("Invalid data received")  # Handle any invalid data
except KeyboardInterrupt:
    print("Program interrupted")
finally:
    ser.close()  # Close the serial connection
