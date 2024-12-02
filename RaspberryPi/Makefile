# Define the Python interpreter
PYTHON = python3

# Define the Python scripts
HALLEFFECT_SCRIPT = HallEffectMqtt.py
BME055_SCRIPT = bme0055.py
INA260_SCRIPT = ina260.py
SERVO_SCRIPT = servoMQTT.py
WEATHER_SCRIPT = getweather.py

# Define a rule to run the Hall effect script
run_halleffect:
	$(PYTHON) $(HALLEFFECT_SCRIPT)

# Define a rule to run the BME055 sensor script
run_bme055:
	$(PYTHON) $(BME055_SCRIPT)

# Define a rule to run the ina260 sensor script
run_ina260:
	$(PYTHON) $(INA260_SCRIPT)

# Define a rule to run the servoMQTT sensor script
run_servo:
	$(PYTHON) $(SERVO_SCRIPT)
	
run_weather:
	$(PYTHON) $(WEATHER_SCRIPT)

# Define a rule to run all scripts in parallel
run_all:
	@echo "Running all scripts in parallel..."
	$(PYTHON) $(HALLEFFECT_SCRIPT) &
	$(PYTHON) $(BME055_SCRIPT) &
	$(PYTHON) $(INA260_SCRIPT) &
	$(PYTHON) $(SERVO_SCRIPT) &
	$(PYTHON) $(WEATHER_SCRIPT) &

# Define a rule to stop all background processes
stop_all:
	@echo "Stopping all scripts..."
	pkill -f $(HALLEFFECT_SCRIPT)
	pkill -f $(BME055_SCRIPT)
	pkill -f $(INA260_SCRIPT)
	pkill -f $(SERVO_SCRIPT)
	pkill -f $(WEATHER_SCRIPT)
