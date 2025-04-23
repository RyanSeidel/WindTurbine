import joblib
import pandas as pd
import numpy as np

# --- Configuration ---
# Ensure these match the names used when training the ORIENTATION model
# This model predicts 'orientation_heading'
model_filename = 'orientation_poly_regression_model.pkl'
feature_names = [
    # Features used to predict orientation heading
    'weatherstation_speed', 'weatherstation_direction', # Weather direction is an INPUT here
    'rpm_value',
    'accelerometer_ax', 'accelerometer_ay', 'accelerometer_az',
    'linear_acceleration_lx', 'linear_acceleration_ly', 'linear_acceleration_lz',
    'voltage_value'
]
target_name = 'Orientation Heading' # For print statements

# --- Input Values for Prediction ---
# Provide values for ALL features the model was trained on
# Using values gathered from the conversation
input_data = {
    'weatherstation_speed': 3.3,
    'weatherstation_direction': 0, # Input value for wind direction
    'rpm_value': 30,
    'accelerometer_ax': -.8,
    'accelerometer_ay': 0.13,
    'accelerometer_az': 9.45,
    'linear_acceleration_lx': -0.01,
    'linear_acceleration_ly': -0.12,
    'linear_acceleration_lz': -0.29,
    'voltage_value': 2
    # 'orientation_heading' is NOT an input here, it's the target we predict
}

# --- Load the Model Pipeline ---
try:
    print(f"Loading model pipeline from: {model_filename}")
    # Load the entire pipeline (scaler, polynomial features, linear model)
    model_pipeline = joblib.load(model_filename)
    print("Model loaded successfully.")
except FileNotFoundError:
    print(f"Error: Model file '{model_filename}' not found. Make sure the training script for orientation heading ran successfully and the file is in the correct directory.")
    exit()
except Exception as e:
    print(f"Error loading model: {e}")
    exit()

# --- Prepare Input Data for Prediction ---
# Check if all necessary keys are provided
missing_keys = [key for key in feature_names if key not in input_data]
if missing_keys:
    print(f"Error: Missing values for the following features: {missing_keys}")
    print("Please ensure all required keys are in the 'input_data' dictionary.")
    exit()

# Create a pandas DataFrame in the correct column order
try:
    input_df = pd.DataFrame([input_data])
    # Ensure column order matches the order used during training
    input_df = input_df[feature_names]
    print("\nInput Data for Prediction:")
    print(input_df)
except Exception as e:
    print(f"Error creating DataFrame from input data: {e}")
    exit()


# --- Make Prediction ---
try:
    print(f"\nMaking prediction for {target_name}...")
    # Use the pipeline's predict method. It handles scaling and polynomial transformation automatically.
    predicted_heading = model_pipeline.predict(input_df)

    # The prediction is returned as an array (even for a single input), get the first element
    prediction_value = predicted_heading[0]

    # --- Rule Override --- (Optional - keep or remove based on preference)
    # Check if the specific conditions for the rule are met
    # if input_data['weatherstation_direction'] == 0 and input_data['rpm_value'] == 60:
    #     print("Applying rule: Wind direction is 0 and RPM is 60. Overriding prediction to 0 degrees.")
    #     prediction_value = 0.0 # Override the prediction
    # --- End Rule Override ---


    print(f"\nPredicted {target_name}: {prediction_value:.2f} degrees")

    # Optional: Add reminder about circular nature and normalize if needed (applied to final value)
    if prediction_value < 0 or prediction_value > 360:
        print("Note: The prediction is outside the typical 0-360 degree range.")
        # Normalize the prediction to be within 0-360
        normalized_prediction = prediction_value % 360
        # Handle potential negative results from modulo if needed
        if normalized_prediction < 0:
             normalized_prediction += 360
        print(f"Normalized prediction (modulo 360): {normalized_prediction:.2f} degrees")

except Exception as e:
    print(f"Error during prediction: {e}")

