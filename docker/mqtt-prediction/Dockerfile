# Base Python image
FROM python:3.9-slim

# Set the working directory
WORKDIR /app

# Copy the requirements file and install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the Python script and model
COPY . .

# Set environment variables
ENV MQTT_BROKER=127.0.0.1
ENV MQTT_PORT=1883
ENV MODEL_FILENAME=wind_turbine_model.pkl

# Command to run the Python script
CMD ["python", "mqtt_prediction.py"]