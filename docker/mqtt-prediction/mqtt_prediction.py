import paho.mqtt.client as mqtt
import os
import time
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# Load environment variables or use defaults
MQTT_BROKER = os.getenv("MQTT_BROKER", "host.docker.internal")  # Connect to the Docker host
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))  # Default MQTT port
PUBLISH_TOPIC = os.getenv("PUBLISH_TOPIC", "wind_turbine/predictions")  # MQTT topic to publish to
PUBLISH_INTERVAL = int(os.getenv("PUBLISH_INTERVAL", 5))  # Publish interval in seconds

# Initialize the MQTT client
client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    """Callback for when the client connects to the broker."""
    if rc == 0:
        logging.info(f"Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
    else:
        logging.error(f"Failed to connect to MQTT broker. Return code: {rc}")

def on_publish(client, userdata, mid):
    """Callback for when a message is published."""
    logging.info(f"Message with mid {mid} published to topic '{PUBLISH_TOPIC}'")

def publish_messages():
    """Continuously publish messages to the MQTT topic."""
    counter = 1
    while True:
        message = f"Prediction data message #{counter}"  # Example message
        try:
            result = client.publish(PUBLISH_TOPIC, message)
            result.wait_for_publish()  # Ensure message delivery
            logging.info(f"Published: {message}")
        except Exception as e:
            logging.error(f"Failed to publish message: {e}")
        counter += 1
        time.sleep(PUBLISH_INTERVAL)  # Wait before sending the next message

def main():
    """Main function to set up the MQTT client and start publishing."""
    try:
        # Attach callbacks
        client.on_connect = on_connect
        client.on_publish = on_publish

        # Connect to the MQTT broker
        logging.info(f"Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)

        # Start the loop in a separate thread
        client.loop_start()

        # Publish messages continuously
        logging.info(f"Publishing to topic '{PUBLISH_TOPIC}' every {PUBLISH_INTERVAL} seconds.")
        publish_messages()
    except KeyboardInterrupt:
        logging.info("Publishing stopped by user.")
    finally:
        # Stop the loop and disconnect
        client.loop_stop()
        client.disconnect()
        logging.info("Disconnected from MQTT broker.")

if __name__ == "__main__":
    main()
