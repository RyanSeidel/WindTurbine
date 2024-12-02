import serial

bluetooth_port = 'COM6'  # Replace with your Bluetooth COM port
baud_rate = 115200

try:
    with serial.Serial(bluetooth_port, baud_rate, timeout=1) as ser:
        print("Connected to ESP32_BT")

        while True:
            # Prompt the user for servo angle input
            angle = input("Enter servo angle (0-360 or 'exit' to quit): ")
            if angle.lower() == 'exit':
                print("Exiting...")
                break

            # Validate angle input
            if angle.isdigit() and 0 <= int(angle) <= 360:
                command = f"angle:{angle}\n"
                ser.write(command.encode())  # Send command to ESP32
                print(f"Sent: {command.strip()}")
                
                # Read ESP32 response
                response = ser.readline().decode('utf-8').strip()
                if response:
                    print("ESP32 Response:", response)
            else:
                print("Invalid input. Please enter a value between 0 and 360.")
except serial.SerialException as e:
    print(f"Error: {e}")
