import pandas as pd
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import make_pipeline

# Load data
data = pd.read_csv('wind_turbine_data3.csv')
X = data[['rpm_value']]
y = data['voltage_value']

# Create polynomial model (degree 2 or 3)
model = make_pipeline(PolynomialFeatures(degree=2), LinearRegression())
model.fit(X, y)

# Predict voltage for higher RPM
new_rpm = [[80]]  # Replace with a higher RPM value
predicted_voltage = model.predict(new_rpm)
print(f"Predicted Voltage for RPM: {predicted_voltage[0]}")
