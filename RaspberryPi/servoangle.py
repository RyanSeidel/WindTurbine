
from gpiozero import AngularServo
from time import sleep

# Initialize the servo on GPIO pin 17
# Adjust pulse width based on your servo's specifications
servo = AngularServo(17, min_angle=0, max_angle=360, min_pulse_width=0.0005, max_pulse_width=0.0025) 

while True:
    # Set servo to each angle and hold for 10 seconds
    for angle in [0]:  # Added 225 and 315 for all 45-degree intervals 
        servo.angle = angle
        print(f"Setting angle to {angle}")
        sleep(10)  # Hold each angle for 10 seconds

