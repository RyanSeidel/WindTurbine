import time
import board
import adafruit_ina260
import paho.mqtt.client as mqtt

# MQTT Configuration
MQTT_BROKER = "localhost"  # Replace with your broker's IP address if needed
MQTT_PORT = 1883
MQTT_TOPICS = {
    'voltage': 'wind_turbine/volt',
    'power': 'wind_turbine/power',
    'current': 'wind_turbine/current'
}

# Initialize the MQTT Client
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
mqtt_client.loop_start()

# Initialize the INA260 sensor
i2c = board.I2C()  # Uses board.SCL and board.SDA
ina260 = adafruit_ina260.INA260(i2c)

try:
    while True:
        # Read values from INA260
        current = ina260.current  # in mA
        voltage = ina260.voltage  # in V
        power = ina260.power      # in mW

        # Print readings for debugging
        print(f"Current: {current:.2f} mA Voltage: {voltage:.2f} V Power: {power:.2f} mW")

        # Publish readings to respective MQTT topics
        mqtt_client.publish(MQTT_TOPICS['current'], current)
        mqtt_client.publish(MQTT_TOPICS['voltage'], voltage)
        mqtt_client.publish(MQTT_TOPICS['power'], power)

        # Wait before taking the next reading
        time.sleep(1)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    mqtt_client.disconnect()
    mqtt_client.loop_stop()
