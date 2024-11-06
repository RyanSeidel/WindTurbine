import logging
import time
import paho.mqtt.client as mqtt
from Adafruit_BNO055 import BNO055

# Configure logging
#logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Initialize the BNO055 sensor
bno = BNO055.BNO055(busnum=1)

# MQTT Configuration
MQTT_BROKER = "localhost"  # Change this to your broker's IP address if needed
MQTT_PORT = 1883
MQTT_TOPICS = {
    'orientation': 'wind_turbine/orientation',
    'temperature': 'wind_turbine/temperature',
    'magnetometer': 'wind_turbine/magnetometer',
    'gyroscope': 'wind_turbine/gyroscope',
    'accelerometer': 'wind_turbine/accelerometer',
    'linear_acceleration': 'wind_turbine/linear_acceleration',
    'gravity': 'wind_turbine/gravity',
    'calibration': 'wind_turbine/calibration'
}

# Initialize MQTT Client
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
mqtt_client.loop_start()

# Initialize the sensor and check if it's connected
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

#logging.info('Reading BNO055 data and publishing to MQTT, press Ctrl-C to quit...')
while True:
    # Read and publish Euler angles (heading, roll, pitch) with rounding
    heading, roll, pitch = bno.read_euler()
    mqtt_client.publish(MQTT_TOPICS['orientation'], f"{heading:.3f},{roll:.3f},{pitch:.3f}")
    #logging.debug(f"Orientation Data: {heading:.3f}, {roll:.3f}, {pitch:.3f}")

    # Read and publish temperature
    temp_c = bno.read_temp()
    mqtt_client.publish(MQTT_TOPICS['temperature'], f"{temp_c:.2f}")
    #logging.debug(f"Temperature: {temp_c:.2f}°C")
    


    # Read and publish magnetometer data with rounding
    mx, my, mz = bno.read_magnetometer()
    mqtt_client.publish(MQTT_TOPICS['magnetometer'], f"{mx:.3f},{my:.3f},{mz:.3f}")
    #logging.debug(f"Magnetometer Data: {mx:.3f}, {my:.3f}, {mz:.3f}")

    # Read and publish gyroscope data with rounding
    gx, gy, gz = bno.read_gyroscope()
    mqtt_client.publish(MQTT_TOPICS['gyroscope'], f"{gx:.3f},{gy:.3f},{gz:.3f}")
    #logging.debug(f"Gyroscope Data: {gx:.3f}, {gy:.3f}, {gz:.3f}")

    # Read and publish accelerometer data with rounding
    ax, ay, az = bno.read_accelerometer()
    mqtt_client.publish(MQTT_TOPICS['accelerometer'], f"{ax:.3f},{ay:.3f},{az:.3f}")
    #logging.debug(f"Accelerometer Data: {ax:.3f}, {ay:.3f}, {az:.3f}")

    # Read and publish linear acceleration data with rounding
    lax, lay, laz = bno.read_linear_acceleration()
    mqtt_client.publish(MQTT_TOPICS['linear_acceleration'], f"{lax:.3f},{lay:.3f},{laz:.3f}")
    #logging.debug(f"Linear Acceleration Data: {lax:.3f}, {lay:.3f}, {laz:.3f}")

    # Read and publish gravity data with rounding
    gx, gy, gz = bno.read_gravity()
    mqtt_client.publish(MQTT_TOPICS['gravity'], f"{gx:.3f},{gy:.3f},{gz:.3f}")
    #logging.debug(f"Gravity Data: {gx:.3f}, {gy:.3f}, {gz:.3f}")

    # Read and publish calibration status
    sys_cal, gyro_cal, accel_cal, mag_cal = bno.get_calibration_status()
    mqtt_client.publish(MQTT_TOPICS['calibration'], f"{sys_cal},{gyro_cal},{accel_cal},{mag_cal}")
    #logging.debug(f"Calibration Status: {sys_cal}, {gyro_cal}, {accel_cal}, {mag_cal}")

    # Delay between readings
    time.sleep(1)
