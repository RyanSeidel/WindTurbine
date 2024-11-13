from influxdb_client import InfluxDBClient
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
import pandas as pd
import numpy as np
import joblib

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
    "rpm": ["value"],
    "current": ["value"],
    "power": ["value"],
    "voltage": ["value"],
    "orientation": ["heading"], # Assuming heading represents wind direction alignment
    "servo": ["value"]
}

# Retrieve data and store it in a DataFrame
data = {}
for measurement, fields in measurements.items():
    for field in fields:
        query = f'''
        from(bucket: "{INFLUXDB_BUCKET}")
        |> range(start: -1h)
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

# Convert to DataFrame and handle missing values
df = pd.DataFrame(data).ffill().bfill()

# Close the InfluxDB client
client.close()

# Define features (RPM, Current, Power) and target (Voltage)
X = df[['rpm_value', 'current_value', 'power_value']].fillna(0)  # Predictors
y = df['voltage_value'].fillna(0)  # Target variable (Voltage)

# Step 1: Polynomial Features Transformation
degree = 2  # Adjust degree based on data exploration and performance
poly_features = PolynomialFeatures(degree=degree)
X_poly = poly_features.fit_transform(X)

# Step 2: Train-Test Split
X_train, X_test, y_train, y_test = train_test_split(X_poly, y, test_size=0.2, random_state=42)

# Step 3: Train the Polynomial Regression Model
model = LinearRegression()
model.fit(X_train, y_train)

# Save the model and transformer for later use
joblib.dump(model, 'polynomial_model.pkl')
joblib.dump(poly_features, 'poly_features.pkl')

# Optionally, print model performance
train_score = model.score(X_train, y_train)
test_score = model.score(X_test, y_test)
print(f"Training R^2 Score: {train_score:.2f}")
print(f"Testing R^2 Score: {test_score:.2f}")

# Define function to predict voltage with new user RPM input
def predict_voltage(rpm_input):
    # Load model and polynomial features transformer
    model = joblib.load('polynomial_model_rpm_only.pkl')
    poly_features = joblib.load('poly_features_rpm_only.pkl')
    
    # Prepare input data for prediction
    input_data = np.array([[rpm_input]])
    input_poly = poly_features.transform(input_data)
    
    # Predict voltage
    predicted_voltage = model.predict(input_poly)
    return predicted_voltage[0]

# Example: Predict voltage based on user-provided RPM value
user_rpm = 200  # Example RPM value
predicted_voltage = predict_voltage(user_rpm)
print(f"Predicted Voltage for RPM {user_rpm}: {predicted_voltage:.2f} volts")
