import numpy as np
import pandas as pd
from sklearn.ensemble import IsolationForest, RandomForestRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error
import joblib

# Load and process data
data = pd.read_csv('wind_turbine_data.csv')

# Calculate vibration magnitude from accelerometer data
data['vibration_magnitude'] = np.sqrt(data['accelerometer_ax']**2 + data['accelerometer_ay']**2 + data['accelerometer_az']**2)

# Assuming each blade angle (in degrees) is captured as a feature
data['blade1_angle'] = 60  # Example data - replace with actual values if available
data['blade2_angle'] = 60
data['blade3_angle'] = 60

# Define features for Isolation Forest and Threshold Prediction
features = [
    'vibration_magnitude', 'wind_speed', 'rpm_value', 'temperature_value',
    'orientation_heading', 'orientation_roll', 'orientation_pitch', 'servo_value',
    'blade1_angle', 'blade2_angle', 'blade3_angle'
]

# Use Isolation Forest for Anomaly Detection
iso_forest = IsolationForest(contamination=0.01, random_state=42)
data['anomaly'] = iso_forest.fit_predict(data[features])  # -1 indicates anomaly

# Threshold Prediction Model using Random Forest
X = data[['wind_speed', 'rpm_value', 'temperature_value', 'orientation_heading', 'servo_value', 'blade1_angle', 'blade2_angle', 'blade3_angle']]
y = data['vibration_magnitude']
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Train model and evaluate
threshold_model = RandomForestRegressor(n_estimators=100, random_state=42)
threshold_model.fit(X_train, y_train)
y_pred_train = threshold_model.predict(X_train)
y_pred_test = threshold_model.predict(X_test)
train_mae = mean_absolute_error(y_train, y_pred_train)
test_mae = mean_absolute_error(y_test, y_pred_test)
print(f"Training MAE: {train_mae:.2f}")
print(f"Testing MAE: {test_mae:.2f}")

# Save models
joblib.dump(iso_forest, 'anomaly_detector.pkl')
joblib.dump(threshold_model, 'vibration_threshold_predictor.pkl')

# Example of predicting a safe vibration threshold with new conditions
new_conditions = pd.DataFrame([[12, 100, 25, 360, 45, 60, 60, 60]], columns=['wind_speed', 'rpm_value', 'temperature_value', 'orientation_heading', 'servo_value', 'blade1_angle', 'blade2_angle', 'blade3_angle'])
predicted_threshold = threshold_model.predict(new_conditions)
print(f"Predicted Safe Vibration Threshold: {predicted_threshold[0]}")
