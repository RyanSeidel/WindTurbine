#!/bin/bash

# Start mqtt_prediction.py and wait for it to initialize
python mqtt_prediction.py &
PID=$!
sleep 5  # Wait for Mosquitto and mqtt_prediction.py to be ready

# Start anomalypredict.py
python anomalypredict.py
wait $PID