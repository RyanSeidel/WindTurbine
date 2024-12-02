# -*- coding: utf-8 -*-
import time
import random
import paho.mqtt.client as mqtt
from collections import deque

# MQTT setup
mqtt_broker = "localhost"  # Replace with the IP address of your MQTT broker
mqtt_topics = {
    'rpm': 'wind_turbine/rpm',
    'temperature': 'wind_turbine/temperature',
    'voltage': 'wind_turbine/volt',
    'accelerometer': 'wind_turbine/accelerometer',
    'magnetometer': 'wind_turbine/magnetometer',
    'gyroscope': 'wind_turbine/gyroscope',
    'gravity': 'wind_turbine/gravity',
    'linear_acceleration': 'wind_turbine/linear_acceleration',
    'pressure': 'wind_turbine/pressure',
    'humidity': 'wind_turbine/humidity',
    'current': 'wind_turbine/current',
    'power': 'wind_turbine/power',
    'wind_speed': 'wind_turbine/speed',
    'wind_direction': 'wind_turbine/direction',
    'altitude': 'wind_turbine/altitude',
    'servo': 'wind_turbine/servo',
    'orientation': 'wind_turbine/orientation'
}
client = mqtt.Client()

# Connect to the MQTT broker
client.connect(mqtt_broker, 1883, 60)
client.loop_start()

# Variables for simulation
rpm_readings = deque(maxlen=20)  # Stores last 20 RPM readings for smoothing
max_rpm = 100  # Maximum simulated RPM
max_temperature = 50  # Maximum temperature (°C)
min_temperature = 20  # Minimum temperature (°C)
max_voltage = 12  # Maximum voltage (V)
min_voltage = 0  # Minimum voltage (V)

# Simulated ranges
ax_range = (-1.0, 1.0)
ay_range = (-1.0, 1.0)
az_range = (9.3, 9.5)

gravity_range = (-9.8, 9.8)
linear_acceleration_range = (-1.0, 1.0)

pressure_range = (950, 1050)  # hPa
humidity_range = (30.0, 90.0)  # %RH
current_range = (0, 20)  # Amperes
power_range = (0, 500)  # Watts

altitude_range = (0, 5000)  # Altitude in meters
wind_speed_range = (1, 10)  # Wind speed in m/s
wind_direction_fixed = 5  # Fixed wind direction value
servo_fixed = 60  # Fixed servo position

mx_range = (-100.0, 100.0)
my_range = (-5.0, 5.0)
mz_range = (-15.0, 15.0)

gx_range = (-0.1, 0.1)
gy_range = (-0.1, 0.1)
gz_range = (-0.1, 0.1)

heading_range = (0, 360)  # Heading in degrees (0 to 360)
pitch_range = (-90, 90)   # Pitch in degrees (-90 to 90)
roll_range = (-180, 180)  # Roll in degrees (-180 to 180)


def generate_fake_rpm():
    """Simulates the RPM with random fluctuations."""
    return random.uniform(0, max_rpm)


def generate_fake_temperature():
    """Simulates the temperature with random fluctuations."""
    return random.uniform(min_temperature, max_temperature)


def generate_fake_voltage():
    """Simulates the voltage with random fluctuations."""
    return random.uniform(min_voltage, max_voltage)


def generate_fake_accelerometer():
    """Simulates accelerometer data with random fluctuations."""
    ax = random.uniform(ax_range[0], ax_range[1])
    ay = random.uniform(ay_range[0], ay_range[1])
    az = random.uniform(az_range[0], az_range[1])
    return ax, ay, az


def generate_fake_magnetometer():
    """Simulates magnetometer data with random fluctuations."""
    mx = random.uniform(mx_range[0], mx_range[1])
    my = random.uniform(my_range[0], my_range[1])
    mz = random.uniform(mz_range[0], mz_range[1])
    return mx, my, mz


def generate_fake_gyroscope():
    """Simulates gyroscope data with random fluctuations."""
    gx = random.uniform(gx_range[0], gx_range[1])
    gy = random.uniform(gy_range[0], gy_range[1])
    gz = random.uniform(gz_range[0], gz_range[1])
    return gx, gy, gz


def generate_fake_gravity():
    """Simulates gravity vector."""
    grx = random.uniform(gravity_range[0], gravity_range[1])
    gry = random.uniform(gravity_range[0], gravity_range[1])
    grz = random.uniform(gravity_range[0], gravity_range[1])
    return grx, gry, grz


def generate_fake_linear_acceleration():
    """Simulates linear acceleration."""
    lx = random.uniform(linear_acceleration_range[0], linear_acceleration_range[1])
    ly = random.uniform(linear_acceleration_range[0], linear_acceleration_range[1])
    lz = random.uniform(linear_acceleration_range[0], linear_acceleration_range[1])
    return lx, ly, lz


def generate_fake_pressure():
    """Simulates atmospheric pressure."""
    return random.uniform(pressure_range[0], pressure_range[1])


def generate_fake_humidity():
    """Simulates humidity."""
    return random.uniform(humidity_range[0], humidity_range[1])


def generate_fake_current():
    """Simulates electrical current."""
    return random.uniform(current_range[0], current_range[1])


def generate_fake_power():
    """Simulates electrical power."""
    return random.uniform(power_range[0], power_range[1])


def generate_fake_altitude():
    """Simulates altitude."""
    return random.uniform(altitude_range[0], altitude_range[1])


def generate_fake_wind_speed():
    """Simulates wind speed."""
    return random.uniform(wind_speed_range[0], wind_speed_range[1])


def get_smoothed_rpm():
    """Calculates the smoothed RPM using the deque."""
    if len(rpm_readings) == 0:
        return 0
    return sum(rpm_readings) / len(rpm_readings)


def publish_data(topic, value):
    """Publish data to the specified MQTT topic."""
    print(f"Publishing to {topic}: {value}")
    client.publish(topic, value)

def generate_fake_orientation():
    """Simulates orientation data for heading, pitch, and roll."""
    heading = random.uniform(heading_range[0], heading_range[1])
    pitch = random.uniform(pitch_range[0], pitch_range[1])
    roll = random.uniform(roll_range[0], roll_range[1])
    return heading, pitch, roll

def simulate_sensor_data():
    """Continuously simulates all sensor readings."""
    while True:
        # Generate fake data
        rpm = generate_fake_rpm()
        temperature = generate_fake_temperature()
        voltage = generate_fake_voltage()
        ax, ay, az = generate_fake_accelerometer()
        mx, my, mz = generate_fake_magnetometer()
        gx, gy, gz = generate_fake_gyroscope()
        grx, gry, grz = generate_fake_gravity()
        lx, ly, lz = generate_fake_linear_acceleration()
        pressure = generate_fake_pressure()
        humidity = generate_fake_humidity()
        current = generate_fake_current()
        power = generate_fake_power()
        altitude = generate_fake_altitude()
        wind_speed = generate_fake_wind_speed()
        heading, pitch, roll = generate_fake_orientation()

        # Add RPM to deque for smoothing
        rpm_readings.append(rpm)
        smoothed_rpm = get_smoothed_rpm()

        # Prepare formatted data
        accelerometer_data = f"{ax:.2f}, {ay:.2f}, {az:.2f}"
        magnetometer_data = f"{mx:.2f}, {my:.2f}, {mz:.2f}"
        gyroscope_data = f"{gx:.2f}, {gy:.2f}, {gz:.2f}"
        gravity_data = f"{grx:.2f}, {gry:.2f}, {grz:.2f}"
        linear_acceleration_data = f"{lx:.2f}, {ly:.2f}, {lz:.2f}"
        orientation_data = f"{heading:.2f}, {pitch:.2f}, {roll:.2f}"

        # Publish the simulated data
        publish_data(mqtt_topics['rpm'], f"{smoothed_rpm:.2f}")
        publish_data(mqtt_topics['temperature'], f"{temperature:.2f}")
        publish_data(mqtt_topics['voltage'], f"{voltage:.2f}")
        publish_data(mqtt_topics['accelerometer'], accelerometer_data)
        publish_data(mqtt_topics['magnetometer'], magnetometer_data)
        publish_data(mqtt_topics['gyroscope'], gyroscope_data)
        publish_data(mqtt_topics['gravity'], gravity_data)
        publish_data(mqtt_topics['linear_acceleration'], linear_acceleration_data)
        publish_data(mqtt_topics['pressure'], f"{pressure:.2f}")
        publish_data(mqtt_topics['humidity'], f"{humidity:.2f}")
        publish_data(mqtt_topics['current'], f"{current:.2f}")
        publish_data(mqtt_topics['power'], f"{power:.2f}")
        publish_data(mqtt_topics['wind_speed'], f"{wind_speed:.2f}")
        publish_data(mqtt_topics['wind_direction'], wind_direction_fixed)
        publish_data(mqtt_topics['altitude'], f"{altitude:.2f}")
        publish_data(mqtt_topics['servo'], servo_fixed)
        publish_data(mqtt_topics['orientation'], orientation_data)

        # Print for debugging
        print(f"Accelerometer: {accelerometer_data}")
        print(f"Magnetometer: {magnetometer_data}")
        print(f"Gyroscope: {gyroscope_data}")
        print(f"Gravity: {gravity_data}")
        print(f"Linear Acceleration: {linear_acceleration_data}")
        print(f"Wind Speed: {wind_speed:.2f} m/s")
        print(f"Wind Direction: {wind_direction_fixed}")
        print(f"Altitude: {altitude:.2f} m")
        print(f"Servo Position: {servo_fixed}")

        time.sleep(1)  # Simulate data generation every second


try:
    simulate_sensor_data()
except KeyboardInterrupt:
    print("Exiting...")
    client.disconnect()
