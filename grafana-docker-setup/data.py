from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# Replace with your actual InfluxDB settings
url = "http://localhost:8086"
token = "QKI_RLoNe4TL1EveFQuc_ObdaZ4xplfE5qVMKQwKIWxBlNxUsdErkb_6umDTIogtHmxFQeGXOxRde73FIOFP0w=="  # Replace this with your actual token
org = "TAMUCC"           # Your organization name
bucket = "Test"   # The bucket (similar to database in 1.x)

# Create the InfluxDB client
client = InfluxDBClient(url=url, token=token, org=org)

try:
    # Check if the client is able to connect by querying the health endpoint
    health = client.health()
    if health.status == "pass":
        print("Connected to InfluxDB successfully")
    else:
        print("Failed to connect to InfluxDB:", health.message)
    
    # Define a data point (measurement)
    point = (
        Point("WindTurbine")  # The measurement name
        .tag("sensor", "hall_effect")  # Tags (optional)
        .field("rpm", 123.45)  # Fields (the actual data)
    )
    
    # Write the data point to the InfluxDB bucket
    write_api = client.write_api(write_options=SYNCHRONOUS)
    write_api.write(bucket=bucket, org=org, record=point)
    print("Data point written successfully")
    
except Exception as e:
    print(f"Error: {e}")
finally:
    # Close the client after the operation
    client.close()
