from influxdb_client import InfluxDBClient

# Replace with your actual InfluxDB settings
url = "http://localhost:8086"
token = "R9UinuSyfDSyZ-6U1roVz3DsQuyR89p03tgMnq-ZKJnBPLSskpDfYbUE0dpJtibILU9onGR_0Pf-MQfD4iWvTw=="  # Replace this with your actual token
org = "TAMUCC"            # Your organization name
bucket = "windturbine_realtime"            # The bucket (similar to database in 1.x)

# Create the InfluxDB client
client = InfluxDBClient(url=url, token=token, org=org)

try:
    # Check if the client is able to connect by querying the health endpoint
    health = client.health()
    if health.status == "pass":
        print("Connected to InfluxDB successfully")
    else:
        print("Failed to connect to InfluxDB:", health.message)
except Exception as e:
    print(f"Error: {e}")
finally:
    # Close the client after the operation
    client.close()
