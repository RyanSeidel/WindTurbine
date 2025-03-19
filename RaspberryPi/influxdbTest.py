from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
import os
import time

# InfluxDB Configuration
INFLUXDB_URL = os.getenv("INFLUXDB_URL", "http://localhost:8086")
INFLUXDB_TOKEN = os.getenv("INFLUXDB_TOKEN", "YK5PVqE0aI9pwLBdZMDE5qt_jDvfYv4m2psX6tTQ13unsruDRf8JSzRq2y1cKgVastehinPYlgpDTNu0x0zQ2g==")
INFLUXDB_ORG = os.getenv("INFLUXDB_ORG", "TAMUCC")
INFLUXDB_BUCKET = os.getenv("INFLUXDB_BUCKET", "WindTurbine")

# Initialize the InfluxDB client
influx_client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
write_api = influx_client.write_api(write_options=SYNCHRONOUS)
query_api = influx_client.query_api()

# Function to write a test data point to InfluxDB
def write_test_data():
    # Create a test data point
    point = Point("test_measurement").tag("location", "test").field("value", 42.0).time(time.time_ns(), write_precision="ns")
    
    # Write the point to InfluxDB
    write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
    print("Test data point written to InfluxDB.", flush=True)

# Function to query the test data point from InfluxDB
def query_test_data():
    # Define the query
    query = f'''
    from(bucket: "{INFLUXDB_BUCKET}")
      |> range(start: -1h)
      |> filter(fn: (r) => r._measurement == "test_measurement")
      |> filter(fn: (r) => r.location == "test")
    '''
    
    # Execute the query
    result = query_api.query(query)
    
    # Print the query results
    print("Querying test data from InfluxDB...", flush=True)
    for table in result:
        for record in table.records:
            print(f"Time: {record.get_time()}, Measurement: {record.get_measurement()}, Field: {record.get_field()}, Value: {record.get_value()}", flush=True)

# Main function to test InfluxDB connection
def test_influxdb_connection():
    try:
        # Write test data
        write_test_data()
        
        # Query test data
        query_test_data()
        
        print("InfluxDB connection test successful!", flush=True)
    except Exception as e:
        print(f"Error during InfluxDB connection test: {e}", flush=True)
    finally:
        # Close the InfluxDB client
        influx_client.close()

# Run the test
test_influxdb_connection()