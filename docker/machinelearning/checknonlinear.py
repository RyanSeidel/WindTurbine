import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from sklearn.metrics import mean_squared_error

# Load and combine all datasets
data_files = [
    'wind_turbine_North_Zero_Degree_LowFan.csv',
    'wind_turbine_North_Zero_Degree_NoFan.csv',
    'wind_turbine_West_Zero_Degree_NoFan.csv',
    'wind_turbine_West_Zero_Degree_LowFan.csv',
    'wind_turbine_East_Zero_Degree_LowFan.csv',
    'wind_turbine_East_Zero_Degree_NoFan.csv',
    'wind_turbine_NorthWest_Zero_Degree_NoFan.csv',
    'wind_turbine_NorthWest_Zero_Degree_LowFan.csv',
    'wind_turbine_South_Zero_Degree_LowFan.csv',
    'wind_turbine_South_Zero_Degree_NoFan.csv',
    'wind_turbine_SouthEast_Zero_Degree_NoFan.csv',
    'wind_turbine_SouthEast_Zero_Degree_LowFan.csv',
    'wind_turbine_SouthWest_Zero_Degree_NoFan.csv',
    'wind_turbine_SouthWest_Zero_Degree_LowFan.csv',
    'wind_turbine_NorthEast_Zero_Degree_LowFan.csv',
    'wind_turbine_NorthEast_Zero_Degree_NoFan.csv'
]

data = pd.concat([pd.read_csv(f) for f in data_files], ignore_index=True)
print(f"Combined dataset contains {data.shape[0]} rows.")

# Convert RPM to RPS
data['rps_value'] = data['rpm_value'] / 60  # Convert RPM to RPS

# Features to analyze
features = ['speed_value', 'servo_value', 'direction_value', 
            'orientation_heading', 'orientation_roll', 'orientation_pitch', 
            'rpm_blade_1', 'rpm_blade_2', 'rpm_blade_3']

# Target variable
target = 'rps_value'

# 1. Scatter Plots: Visualize Feature vs Target Relationships
for feature in features:
    plt.figure(figsize=(8, 6))
    sns.scatterplot(data=data, x=feature, y=target, alpha=0.5)
    plt.title(f'{feature} vs {target}')
    plt.xlabel(feature)
    plt.ylabel(target)
    plt.show()

# 2. Correlation Matrix
correlation = data[features + [target]].corr()
print("Feature Correlation with RPS:")
print(correlation[target].sort_values(ascending=False))

# 3. Residual Analysis for Linearity
# Fit a simple linear regression model for each feature and analyze residuals
for feature in features:
    X = data[[feature]].values  # Single feature
    y = data[target].values  # Target variable

    # Train a Linear Regression model
    model = LinearRegression()
    model.fit(X, y)
    y_pred = model.predict(X)
    residuals = y - y_pred

    # Plot residuals
    plt.figure(figsize=(8, 6))
    plt.scatter(X, residuals, alpha=0.5)
    plt.axhline(0, color='red', linestyle='--')
    plt.title(f'Residuals for {feature}')
    plt.xlabel(feature)
    plt.ylabel('Residuals')
    plt.show()

# 4. Polynomial Fitting (Degree 2)
for feature in features:
    X = data[[feature]].values  # Single feature
    y = data[target].values  # Target variable

    # Polynomial Regression (degree=2)
    poly = PolynomialFeatures(degree=2, include_bias=False)
    X_poly = poly.fit_transform(X)

    # Train a Polynomial Regression model
    model = LinearRegression()
    model.fit(X_poly, y)
    y_pred = model.predict(X_poly)

    # Plot actual vs predicted
    plt.figure(figsize=(8, 6))
    plt.scatter(X, y, alpha=0.5, label='Actual')
    plt.scatter(X, y_pred, color='red', alpha=0.5, label='Predicted (Degree 2)')
    plt.title(f'Polynomial Fit for {feature} (Degree 2)')
    plt.xlabel(feature)
    plt.ylabel(target)
    plt.legend()
    plt.show()

    # Mean Squared Error
    mse = mean_squared_error(y, y_pred)
    print(f"Feature: {feature}, Degree 2 Polynomial MSE: {mse}")
