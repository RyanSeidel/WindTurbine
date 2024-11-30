import os
import time
import logging
import paho.mqtt.client as mqtt

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# MQTT Configuration
MQTT_BROKER = os.getenv("MQTT_BROKER", "mosquitto")  # Default: Mosquitto broker
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
RPS_INPUT_TOPIC = "rpsinputform"  # Topic for listening to form data

# Initialize MQTT client
mqtt_client = mqtt.Client()

# MQTT Callback Functions
def on_connect(client, userdata, flags, rc):
    """Callback for when the client connects to the broker."""
    if rc == 0:
        logging.info("Connected to MQTT broker.")
        client.subscribe(RPS_INPUT_TOPIC)
        logging.info(f"Subscribed to topic: {RPS_INPUT_TOPIC}")
    else:
        logging.error(f"Failed to connect to MQTT broker. Return code: {rc}")

def on_message(client, userdata, msg):
    """Callback for handling received messages."""
    topic = msg.topic
    payload = msg.payload.decode()
    if topic == RPS_INPUT_TOPIC:
        logging.info(f"Received message on {topic}: {payload}")
        # Process the incoming message as needed
        process_rps_input(payload)

def process_rps_input(payload):
    """Process the received form data."""
    try:
        logging.info(f"Processing payload: {payload}")
        # Example of parsing the payload (assuming JSON-like data in the future)
        # Add specific processing logic if needed
    except Exception as e:
        logging.error(f"Error processing payload: {e}")

# Main function
def main():
    try:
        # Attach callbacks
        mqtt_client.on_connect = on_connect
        mqtt_client.on_message = on_message

        # Connect to the MQTT broker
        logging.info(f"Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}...")
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()  # Start the loop to process messages
        logging.info("MQTT client loop started.")

        # Keep the script running
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        logging.info("Stopped by user.")
    finally:
        # Gracefully disconnect the client
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        logging.info("Disconnected from MQTT broker.")

if __name__ == "__main__":
    main()
