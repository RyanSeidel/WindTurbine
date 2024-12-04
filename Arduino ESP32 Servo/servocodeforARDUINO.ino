#include "BluetoothSerial.h"
#include <ESP32Servo.h>

// Create a BluetoothSerial object
BluetoothSerial SerialBT;

// Create a Servo object
Servo myServo;

// Pulse width range for 360° control (in microseconds)
const int minPulseWidth = 500;  // Corresponds to 0°
const int maxPulseWidth = 2500; // Corresponds to 360°

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT"); // Bluetooth device name
  Serial.println("Bluetooth is ready. Pair with ESP32_BT");

  // Attach the servo to GPIO 18 with custom pulse width range
  myServo.attach(18, minPulseWidth, maxPulseWidth);
  myServo.writeMicroseconds(minPulseWidth); // Initialize servo to 0°
}

void loop() {
  if (SerialBT.available()) {
    String message = SerialBT.readStringUntil('\n');
    Serial.println("Received: " + message);

    if (message.startsWith("angle:")) {
      // Extract the angle from the message
      int angle = message.substring(6).toInt();
      if (angle >= 0 && angle <= 360) {
        // Map the angle to the pulse width range
        int pulseWidth = map(angle, 0, 360, minPulseWidth, maxPulseWidth);
        myServo.writeMicroseconds(pulseWidth); // Set the servo to the specified angle
        SerialBT.println("Servo moved to " + String(angle) + " degrees");
      } else {
        SerialBT.println("Invalid angle! Use 0-360.");
      }
    } else {
      SerialBT.println("Unknown command");
    }
  }
}
