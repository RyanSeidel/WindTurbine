from influxdb_client import InfluxDBClient
import pandas as pd

# InfluxDB Configuration
INFLUXDB_URL = "http://localhost:8086"
INFLUXDB_TOKEN = "iNLROvcnYQmb6CNVmUyrNuB6CG2EiKOjUrT-F13uF-x1pSYLZGcGS-rbgj9J1cS-zaUwMB6UPd8_SJgVl3KFdQ=="
INFLUXDB_ORG = "TAMUCC"
INFLUXDB_BUCKET = "WindTurbine"

# Initialize the InfluxDB client
client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
query_api = client.query_api()

# Define measurements to retrieve
measurements = {
    "rpm": ["value", "blade_1", "blade_2", "blade_3"],
    "orientation": ["heading", "roll", "pitch"],
    "temperature": ["value"],
    "magnetometer": ["mx", "my", "mz"],
    "gyroscope": ["gx", "gy", "gz"],
    "accelerometer": ["ax", "ay", "az"],
    "linear_acceleration": ["lx", "ly", "lz"],
    "gravity": ["grx", "gry", "grz"],
    "voltage": ["value"],
    "current": ["value"],
    "power": ["value"],
    "servo": ["value"],
    "speed": ["value"],
    "direction": ["value"],  # Add for wind direction
    "pressure": ["value"],  # Add for atmospheric pressure
    "humidity": ["value"],  # Add for relative humidity
    "altitude": ["value"]   # Add for altitude
}

# Retrieve data and store in a DataFrame
data = {}
for measurement, fields in measurements.items():
    for field in fields:
        query = f'''
        from(bucket: "{INFLUXDB_BUCKET}")
        |> range(start: -1m)
        |> filter(fn: (r) => r["_measurement"] == "{measurement}")
        |> filter(fn: (r) => r["_field"] == "{field}")
        '''
        result = query_api.query(org=INFLUXDB_ORG, query=query)
        
        times, values = [], []
        for table in result:
            for record in table.records:
                times.append(record.get_time())
                values.append(record.get_value())
        
        data[f"{measurement}_{field}"] = pd.Series(data=values, index=times)

client.close()

# Convert to DataFrame and handle missing values
df = pd.DataFrame(data).ffill().bfill()

# Optionally, save data locally to avoid re-querying InfluxDB
df.to_csv("wind_60_45DegreeLowFan.csv")
