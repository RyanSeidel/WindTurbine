import pandas as pd
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_absolute_error
import joblib

# Load data
data = pd.read_csv('wind_turbine_data3.csv')

# Check for any missing or unexpected values in 'rpm_value' or 'voltage_value'
print("RPM range:", data['rpm_value'].min(), "-", data['rpm_value'].max())
print("Voltage range:", data['voltage_value'].min(), "-", data['voltage_value'].max())

# Prepare the features and target variable
X = data[['rpm_value']]
y = data['voltage_value']

# Split the data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Create and train the Gradient Boosting model
model = GradientBoostingRegressor(n_estimators=200, learning_rate=0.1, max_depth=3, random_state=42)
model.fit(X_train, y_train)

# Evaluate model performance
y_pred_train = model.predict(X_train)
y_pred_test = model.predict(X_test)
train_mae = mean_absolute_error(y_train, y_pred_train)
test_mae = mean_absolute_error(y_test, y_pred_test)

print(f"Training MAE: {train_mae:.2f}")
print(f"Testing MAE: {test_mae:.2f}")

# Save the model for future use
joblib.dump(model, 'gbm_voltage_predictor_model.pkl')

# Predict voltage for a higher RPM value
new_rpm = [[300]]  # Replace with any desired RPM value within or slightly beyond training range
predicted_voltage = model.predict(new_rpm)
print(f"Predicted Voltage for RPM 300: {predicted_voltage[0]:.2f}")

# Optional: Predict voltages across a range of RPMs to observe behavior
test_rpms = [[100], [200], [300], [400], [500]]
for rpm in test_rpms:
    predicted_voltage = model.predict([rpm])
    print(f"Predicted Voltage for RPM {rpm[0]}: {predicted_voltage[0]:.2f}")
