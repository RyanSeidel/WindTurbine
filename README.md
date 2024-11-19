# Wind Turbine Digital Twins

## Table of Contents
- [Abstract](#abstract)
- [Usage](#usage)
- [How_We_Created_the_Digital_Twin](#How-We-Created-The-Digital-Twin)
- [Raspberry Pi Setup](#RaspberryPi)
- [Dashboard Setup](#Dashboard)
- [ROS/RViz Setup](#RViz)
- [Conclusion and Future Works](#Conclusion and Future Works)
- [Contributing](#contributing)
- [License](#license)


## Abstract 
Digital twins refer to virtual recreations of real-world objects that can mirror the operation and condition of physical systems and can also be used for real-time monitoring and analysis. Wind turbines are often located in caustic environments that increase the risk of wear and tear to mechanical systems that can lead to expensive downtime and safety risks. Digital twin technology offers advantages over traditional monitoring techniques that lack the ability to identify specific problems in real time, needed to address concerns quickly. The present project attempted to build a digital twin of a wind turbine making use of IoT sensors that allowed for live data collection, detection of damage, and assessment of risk. Using Raspberry Pi, an inexpensive mini-computer with extensive support for software and hardware, the system used in the project was able to integrate a variety of tools including Hall Effect Sensors, BNO055 sensors, and INA260 sensors for real-time monitoring and control. This approach included the use of MQTT for communication, InfluxDB for storing data, and visualization using a Flask-based dashboard. For the 3D model, RViz and ROS were used to generate 3D point cloud models using an Xbox Kinect 360.


## Usage
images of the projeccts!!


## How We Created the Digital Twin
We developed the wind turbine's digital twin by combining a Raspberry Pi 5 with a variety of sensors and tools to enable real-time monitoring, control, and visualization. A Hall Effect Sensor tracked the turbine’s RPM, while a BNO055 Sensor captured critical alignment data, including yaw, roll, and pitch. To monitor the turbine’s electrical performance, we used an INA260 Sensor to measure voltage and current. An Angular Servo was integrated to adjust the turbine’s orientation, ensuring it stayed aligned with the wind direction for optimal operation. To handle data flow, we used an MQTT Broker for real-time communication, and sensor data was stored in InfluxDB for further analysis. The entire system was containerized using Docker, ensuring consistency and portability. A custom-built Flask dashboard brought everything together, providing an intuitive interface where live sensor data could be visualized, making it easy to monitor the turbine’s performance in real time. A 3D model of the turbine was created using LiDAR data from an Xbox Kinect 360, processed through RViz and ROS to generate a point cloud. This model was synchronized with real-time sensor data, enabling the digital twin to reflect the turbine’s behavior dynamically. The integration of IoT sensors, real-time communication, and 3D visualization tools created a functional digital twin, providing actionable insights for predictive maintenance and enhanced operational control. This setup serves as a robust foundation for future optimization through AI and machine learning applications.

# RaspberryPi

## Raspberry Libraries Required

To set up the necessary libraries and configurations for this project on a Raspberry Pi, follow these steps:

### Install Required Python Libraries

Run the following commands to install the required Python libraries:

- `pip install python3-rpi.gpio`
- `pip install Adafruit-GPIO`
- `pip3 install influxdb-client`
- `pip3 install adafruit-circuitpython-ina260`
- `pip3 install gpiozero`

### Enable I2C Connection

Run the Raspberry Pi configuration tool to enable I2C:

- Use the command `sudo raspi-config`.
- Navigate to **"Interfacing Options"** and enable **I2C**.

### Enable Serial Connection

Using the same Raspberry Pi configuration tool (`raspi-config`):

- Navigate to **"Interfacing Options"**.
- Enable **Serial**.

---

## Using the Raspberry Pi Folder

1. Copy or extract the **Raspberry Pi Folder** onto your Raspberry Pi OS.
2. Inside the folder, you'll find a `Makefile` that can run all your sensor programs.
   
**⚠️ Warning**: Before you start the programs, make sure to first step up all the [Sensors Setup](#Sensors) and [MQTT Protocol](#MQTT) correctly!

### Commands for Running Programs

- To run all programs:
  - `make run_all`
- To stop all programs gracefully:
  - `make stop_all`

   
## MQTT

### Setting up the MQTT Broker

#### Raspberry Pi Configuration

1. Install the required libraries:
   - `pip3 install paho-mqtt`
   - `sudo apt install mosquitto mosquitto-clients`

2. Configure Mosquitto for External Access:
   - Open the Mosquitto configuration file:
     - `sudo nano /etc/mosquitto/mosquitto.conf`
   - Add the following lines to enable anonymous access (only do this if your network is secure) or set up user authentication if needed:
     ```plaintext
     allow_anonymous true
     listener 1883
     ```
   - Save and exit the file, then restart Mosquitto to apply the changes:
     - `sudo systemctl restart mosquitto`

3. Test the setup:
   - Subscribe to the `wind_turbine/hall_effect` topic:
     - `mosquitto_sub -h 192.168.0.100 -t wind_turbine/hall_effect`

### Additional Resources

For more details and troubleshooting, refer to [Steve's Internet Guide on MQTT](http://www.steves-internet-guide.com/mqtt/).


## Dashboard 

## Setting Up the Dashboard with Docker

### Prerequisites
- Download and install **Docker** on your machine.
- Ensure Docker is running before proceeding.

### Steps to Navigate and Run the Flask Application
1. Navigate to the `docker` directory inside the `WindTurbine` project folder:
   - `cd /docker`
2. Use **Visual Studio Code** (VS Code) for navigating and running the Flask application.

### Starting and Stopping Docker Containers
- To start all containers:
  - `docker-compose up -d --build`
- To stop all containers:
  - `docker-compose down`

### Accessing the Dashboard
1. Once all containers are activated, open your browser and navigate to:
   - `http://localhost:5000`
2. If everything is set up correctly, this will redirect you to:
   - `http://localhost:5000/socket`
   - Ensure that your **MQTT Broker IP** is correct.
   - Verify that the Raspberry Pi is running all required programs as outlined in the [Raspberry Pi Setup](#RaspberryPi) section.


## RViz

### ROS/RViz Dependencies
**⚠️ Warning**: Only work for Neotic Ubuntu Version!
To use this project, you must install the following dependencies:

- **ROS Noetic**: The ROS distribution used for managing the various packages and nodes required for the digital twin.
- **freenect_stack**: This library enables the Xbox Kinect 360 camera to function with ROS, providing the necessary data to generate a 3D point cloud of the wind turbine.

### Build Instructions

1. Extract the `digital_ws` zip file.
2. Erase any previous build files and run:
   - `catkin_make`
3. Check and fix any incorrect pathing if necessary.
4. Source the workspace setup file:
   - `source ~/path/digital_ws/devel/setup.bash`
5. Launch the required ROS nodes:
   - `roslaunch freenect_launch freenect.launch`
   - To test the program, run:
     - `roslaunch pointcloud_filter filter_kinect.launch`  
<em>PointCloud_Filter is a program that reduces the amount of point cloud data to provide a cleaner representation of your model, removing unnecessary data such as the wall behind the object. </em>

### RViz Configuration

- The program should automatically open **RViz** for you.
- If you want to add a camera:
  - Set the topic to `/camera/rgb/image_color` for a color image.
- To add **PointCloud2** for the LiDAR scanner:
  - Use the topic `/filtered_points`.

![PointCloud2](./images/simulation.png)


## Sensors

- **BNO055**
  - [Documentation for BNO055](https://cdn-learn.adafruit.com/downloads/pdf/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black.pdf)
  - Note: I had to change from using `Adafruit.GPIO` to the `RPIO` library.

- **INA260**
  - [Setup Video](https://www.youtube.com/watch?v=ym5ioJFsh4M)
  - Make sure to install the Adafruit INA260 library.
  - Wiring is the same for Raspberry Pi 5.

- **Hall Effect Sensor**
  - [Guide for Interfacing with Raspberry Pi](https://circuitdigest.com/microcontroller-projects/interfacing-hall-sensor-with-raspberry-pi)

- **Angular Servo**
  - [Setup Video](https://www.youtube.com/watch?v=40tZQPd3z8g&t=351s)
 
## Conclusion and Future Works

Our client envisioned a project that would bring their wind turbine operations to life through a digital twin capable of responding in real time to sensor data. They wanted to track critical aspects like RPM, orientation, voltage, current, and environmental conditions, providing a detailed and dynamic view of the turbine's performance. Beyond simple monitoring, their aim was to use this data to make informed decisions during emergencies and evaluate risks in damaged structures, ensuring both safety and efficiency in challenging scenarios. They also saw the potential for AI algorithms to play a significant role in improving how the turbine operates and ensuring proactive maintenance to prevent costly downtime and safety issues. By achieving this, the client hoped to take a big step forward in managing wind turbines efficiently while exploring innovative approaches to maintenance and performance optimization.






