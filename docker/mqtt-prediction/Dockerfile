# Base Python image
FROM python:3.9-slim

# Set the working directory
WORKDIR /app

# Install Mosquitto utilities (optional, useful for debugging MQTT)
RUN apt-get update && apt-get install -y --no-install-recommends mosquitto-clients && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Copy the requirements file and install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the Python script and model
COPY . .

# Expose MQTT port (optional, mostly for clarity)
EXPOSE 1883

# Command to run the MQTT publishing script
CMD ["python", "mqtt_prediction.py"]
