import pandas as pd
import numpy as np
from sklearn.ensemble import IsolationForest
from sklearn.preprocessing import StandardScaler

# Load your data
data = pd.read_csv('wind_turbine_data_with_wind_speed.csv')

# Calculate vibration magnitude if not already present
data['vibration_magnitude'] = np.sqrt(data['accelerometer_ax']**2 + data['accelerometer_ay']**2 + data['accelerometer_az']**2)

# Features for anomaly detection, now including wind speed
features = data[['rpm_value', 'vibration_magnitude', 'wind_speed']]

# Scale features
scaler = StandardScaler()
features_scaled = scaler.fit_transform(features)

# Isolation Forest model for anomaly detection
model = IsolationForest(contamination=0.05)  # Assuming 5% of the data could be anomalies
model.fit(features_scaled)

# Predict anomalies
data['anomaly'] = model.predict(features_scaled)  # -1 for anomalies, 1 for normal

# Filter anomalies
anomalies = data[data['anomaly'] == -1]

print("Potential anomalies detected with wind speed context:")
print(anomalies[['rpm_value', 'vibration_magnitude', 'wind_speed']])
