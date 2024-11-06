from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
import time

# InfluxDB Configuration
INFLUXDB_URL = "http://localhost:8086"
INFLUXDB_TOKEN = "iNLROvcnYQmb6CNVmUyrNuB6CG2EiKOjUrT-F13uF-x1pSYLZGcGS-rbgj9J1cS-zaUwMB6UPd8_SJgVl3KFdQ=="
INFLUXDB_ORG = "TAMUCC"
INFLUXDB_BUCKET = "WindTurbine"

# Initialize the InfluxDB client
client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
write_api = client.write_api(write_options=SYNCHRONOUS)


# Now let's retrieve and print the data for each of these measurements
query_api = client.query_api()

# Query and print RPM data
query_rpm = f'''
from(bucket: "{INFLUXDB_BUCKET}")
|> range(start: -1h)
|> filter(fn: (r) => r["_measurement"] == "rpm")
|> filter(fn: (r) => r["_field"] == "value")
'''
result_rpm = query_api.query(org=INFLUXDB_ORG, query=query_rpm)
print("\nRPM Data:")
for table in result_rpm:
    for record in table.records:
        print(f"Time: {record.get_time()}, RPM Value: {record.get_value()}")

# Query and print Orientation data
query_orientation = f'''
from(bucket: "{INFLUXDB_BUCKET}")
|> range(start: -1h)
|> filter(fn: (r) => r["_measurement"] == "orientation")
'''
result_orientation = query_api.query(org=INFLUXDB_ORG, query=query_orientation)
print("\nOrientation Data:")
for table in result_orientation:
    for record in table.records:
        print(f"Time: {record.get_time()}, Field: {record.get_field()}, Value: {record.get_value()}")

# Query and print Temperature data
query_temperature = f'''
from(bucket: "{INFLUXDB_BUCKET}")
|> range(start: -1h)
|> filter(fn: (r) => r["_measurement"] == "temperature")
|> filter(fn: (r) => r["_field"] == "value")
'''
result_temperature = query_api.query(org=INFLUXDB_ORG, query=query_temperature)
print("\nTemperature Data:")
for table in result_temperature:
    for record in table.records:
        print(f"Time: {record.get_time()}, Temperature Value: {record.get_value()}")

# Query and print Magnetometer data
query_magnetometer = f'''
from(bucket: "{INFLUXDB_BUCKET}")
|> range(start: -1h)
|> filter(fn: (r) => r["_measurement"] == "magnetometer")
'''
result_magnetometer = query_api.query(org=INFLUXDB_ORG, query=query_magnetometer)
print("\nMagnetometer Data:")
for table in result_magnetometer:
    for record in table.records:
        print(f"Time: {record.get_time()}, Field: {record.get_field()}, Value: {record.get_value()}")
        
# Query and print Gyroscope data
query_gyroscope = f'''
from(bucket: "{INFLUXDB_BUCKET}")
|> range(start: -1h)
|> filter(fn: (r) => r["_measurement"] == "gyroscope")
'''
result_gyroscope = query_api.query(org=INFLUXDB_ORG, query=query_gyroscope)
print("\ngyroscope Data:")
for table in result_gyroscope:
    for record in table.records:
        print(f"Time: {record.get_time()}, Field: {record.get_field()}, Value: {record.get_value()}")
        

#Query and print accelerometer data
query_accelerometer = f'''
from(bucket: "{INFLUXDB_BUCKET}")
|> range(start: -1h)
|> filter(fn: (r) => r["_measurement"] == "accelerometer")
'''
result_accelerometer = query_api.query(org=INFLUXDB_ORG, query=query_accelerometer)
print("\nAccelerometer Data:")
for table in result_accelerometer:
    for record in table.records:
        print(f"Time: {record.get_time()}, Field: {record.get_field()}, Value: {record.get_value()}")        
        
# Query and print accelerometer data
query_linear_acceleration = f'''
from(bucket: "{INFLUXDB_BUCKET}")
|> range(start: -1h)
|> filter(fn: (r) => r["_measurement"] == "linear_acceleration")
'''
result_linear_acceleration = query_api.query(org=INFLUXDB_ORG, query=query_linear_acceleration)
print("\nLinear Acceleration Data:")
for table in result_linear_acceleration:
    for record in table.records:
        print(f"Time: {record.get_time()}, Field: {record.get_field()}, Value: {record.get_value()}")      

        
# Query and print Gravity data
query_gravity = f'''
from(bucket: "{INFLUXDB_BUCKET}")
|> range(start: -1h)
|> filter(fn: (r) => r["_measurement"] == "gravity")
'''
result_gravity = query_api.query(org=INFLUXDB_ORG, query=query_gravity)
print("\nLinear Acceleration Data:")
for table in result_gravity:
    for record in table.records:
        print(f"Time: {record.get_time()}, Field: {record.get_field()}, Value: {record.get_value()}")     
  

# # Query and print voltage data
query_voltage  = f'''
from(bucket: "{INFLUXDB_BUCKET}")
|> range(start: -1h)
|> filter(fn: (r) => r["_measurement"] == "voltage")
|> filter(fn: (r) => r["_field"] == "value")
'''
result_voltage  = query_api.query(org=INFLUXDB_ORG, query=query_voltage)
print("\nVoltage  Data:")
for table in result_voltage:
    for record in table.records:
        print(f"Time: {record.get_time()}, Voltage Value: {record.get_value()}")
        
# # Query and print current data
query_current = f'''
from(bucket: "{INFLUXDB_BUCKET}")
|> range(start: -1h)
|> filter(fn: (r) => r["_measurement"] == "current")
|> filter(fn: (r) => r["_field"] == "value")
'''
result_current = query_api.query(org=INFLUXDB_ORG, query=query_current)
print("\nCurrent Data:")
for table in result_current:
    for record in table.records:
        print(f"Time: {record.get_time()}, Current Value: {record.get_value()}")
        
# # Query and print power data
query_power = f'''
from(bucket: "{INFLUXDB_BUCKET}")
|> range(start: -1h)
|> filter(fn: (r) => r["_measurement"] == "power")
|> filter(fn: (r) => r["_field"] == "value")
'''
result_power = query_api.query(org=INFLUXDB_ORG, query=query_power)
print("\nPower Data:")
for table in result_current:
    for record in table.records:
        print(f"Time: {record.get_time()}, Power Value: {record.get_value()}")
        
# Close the client
client.close()
