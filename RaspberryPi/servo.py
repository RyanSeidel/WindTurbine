from gpiozero import AngularServo
from time import sleep

# Initialize the servo on GPIO pin 17
# Experiment with pulse width to match servo's requirements
servo = AngularServo(17, min_angle=0, max_angle=360, min_pulse_width=0.0005, max_pulse_width=0.0025)

while True:
    # Set servo to different angles and observe its response
    for angle in [0, 45, 90, 135, 180, 270, 360]:
        servo.angle = angle
        print(f"Setting angle to {angle}")
        sleep(2)  # Hold each angle for 2 seconds
