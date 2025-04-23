import joblib
import pandas as pd
import numpy as np

# --- Configuration ---
# Ensure these match the names used when training the model that predicts BOTH direction and RPM
model_filename = 'direction_rpm_poly_model.pkl' # <-- Loads the model trained to predict DIRECTION and RPM
feature_names = [
    # Features used to predict weather direction and RPM
    'orientation_heading', # Input feature
    'weatherstation_speed' # Input feature
]
# Define the names of the target variables this model predicts, IN ORDER
target_names = ['Weather Station Direction', 'RPM Value']

# --- Input Values for Prediction ---
# Provide values for ALL features the model was trained on
input_data = {
    'orientation_heading': 351, # <-- INPUT value for current turbine orientation
    'weatherstation_speed': 0    # Input value for speed
    # 'weatherstation_direction' and 'rpm_value' are NOT inputs here, they are the targets
}

# --- Load the Model Pipeline ---
try:
    print(f"Loading model pipeline from: {model_filename}")
    # Load the entire pipeline (scaler, polynomial features, linear model)
    model_pipeline = joblib.load(model_filename)
    print("Model loaded successfully.")
except FileNotFoundError:
    print(f"Error: Model file '{model_filename}' not found. Make sure the training script for direction and RPM ran successfully and the file is in the correct directory.")
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
    print(f"\nMaking prediction for {target_names}...")
    # Use the pipeline's predict method. It handles scaling and polynomial transformation automatically.
    # The prediction will be a 2D numpy array with shape (n_samples, n_targets)
    predictions = model_pipeline.predict(input_df)

    # Extract the individual predictions (assuming only one input row)
    predicted_direction = predictions[0, 0] # First column is direction
    predicted_rpm = predictions[0, 1]       # Second column is RPM

    print(f"\nPredicted {target_names[0]}: {predicted_direction:.2f} degrees")
    print(f"Predicted {target_names[1]}: {predicted_rpm:.2f}")

    # Optional: Normalize the direction prediction
    if predicted_direction < 0 or predicted_direction > 360:
        print("Note: The direction prediction is outside the typical 0-360 degree range.")
        # Normalize the prediction to be within 0-360
        normalized_prediction = predicted_direction % 360
        # Handle potential negative results from modulo if needed
        if normalized_prediction < 0:
             normalized_prediction += 360
        print(f"Normalized direction prediction (modulo 360): {normalized_prediction:.2f} degrees")

except Exception as e:
    print(f"Error during prediction: {e}")

