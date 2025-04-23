import pandas as pd
import numpy as np # Used for checking NaN values
from functools import reduce # For merging multiple dataframes

# --- Configuration ---
# !!! Step 1: Set your file path !!!
file_path = 'North_LowFan_0Degree.csv'
# Example for Excel: file_path = 'your_data_file.xlsx'

# !!! Step 2: Set your column names !!!
# Core Columns
timestamp_col = 'timestamp'       # Name of the column with timestamps
rpm_col = 'rpm_value'           # Name of the column with RPM values

# --- Sensor/Data Groups ---
# Orientation Columns
orient_heading_col = 'orientation_heading'
orient_roll_col = 'orientation_roll'
orient_pitch_col = 'orientation_pitch'
orientation_cols = [orient_heading_col, orient_roll_col, orient_pitch_col]

# Temperature Column
temp_col = 'temperature_temp_c'
temperature_cols = [temp_col]

# Magnetometer Columns
mag_mx_col = 'magnetometer_mx'
mag_my_col = 'magnetometer_my'
mag_mz_col = 'magnetometer_mz'
magnetometer_cols = [mag_mx_col, mag_my_col, mag_mz_col]

# Gyroscope Columns
gyro_gx_col = 'gyroscope_gx'
gyro_gy_col = 'gyroscope_gy'
gyro_gz_col = 'gyroscope_gz'
gyroscope_cols = [gyro_gx_col, gyro_gy_col, gyro_gz_col]

# Accelerometer Columns
accel_ax_col = 'accelerometer_ax'
accel_ay_col = 'accelerometer_ay'
accel_az_col = 'accelerometer_az'
accelerometer_cols = [accel_ax_col, accel_ay_col, accel_az_col]

# Linear Acceleration Columns
lin_accel_lx_col = 'linear_acceleration_lx'
lin_accel_ly_col = 'linear_acceleration_ly'
lin_accel_lz_col = 'linear_acceleration_lz'
linear_acceleration_cols = [lin_accel_lx_col, lin_accel_ly_col, lin_accel_lz_col]

# Gravity Columns
grav_grx_col = 'gravity_grx'
grav_gry_col = 'gravity_gry'
grav_grz_col = 'gravity_grz'
gravity_cols = [grav_grx_col, grav_gry_col, grav_grz_col]

# Voltage Column
voltage_col = 'voltage_value'
voltage_cols = [voltage_col]

# Current Column
current_col = 'current_value'
current_cols = [current_col]

# Power Column
power_col = 'power_value'
power_cols = [power_col]

# Weather Station Columns - Treated separately by default
# Consider grouping if they share timestamps in your data
ws_speed_col = 'weatherstation_speed'
ws_pressure_col = 'weatherstation_pressure'
ws_altitude_col = 'weatherstation_altitude'
ws_humidity_col = 'weatherstation_humidity' # NEW
weather_speed_cols = [ws_speed_col]
weather_pressure_cols = [ws_pressure_col]
weather_altitude_cols = [ws_altitude_col]
weather_humidity_cols = [ws_humidity_col] # NEW
# Example if grouped:
# weather_cols = [ws_speed_col, ws_pressure_col, ws_altitude_col, ws_humidity_col]


# List of *all* non-RPM sensor/data columns configured
all_sensor_data_cols = (orientation_cols + temperature_cols + magnetometer_cols +
                        gyroscope_cols + accelerometer_cols + linear_acceleration_cols +
                        gravity_cols + voltage_cols + current_cols + power_cols +
                        weather_speed_cols + weather_pressure_cols + weather_altitude_cols +
                        weather_humidity_cols) # Added humidity

# !!! Step 3: Set the time tolerance for matching !!!
max_time_difference_seconds = 800 # Max seconds difference allowed for a match

# !!! Step 4: Set the output Excel filename !!!
output_excel_file = 'matched_rpm_sensor_data.xlsx'


# --- Logic to Identify Row Types ---
# !!! Step 5: Choose and verify the correct logic for YOUR data !!!
# CRITICAL: Ensure the logic for each 'is_..._row' function is MUTUALLY EXCLUSIVE.
# You MUST adapt this if your data structure is different.

# === How to identify an RPM row? ===
# Method R1: RPM has data, primary Orientation col (heading) is NaN.
def is_rpm_row(df):
  required = [rpm_col, orient_heading_col]
  if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for RPM check: {[c for c in required if c not in df.columns]}")
  return df[rpm_col].notna() & df[orient_heading_col].isna()

# === How to identify an Orientation row (H/R/P ONLY)? ===
# Method O1: Orient Heading has data, RPM is NaN.
def is_orientation_row(df):
   required = orientation_cols + [rpm_col]
   if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for Orientation check: {[c for c in required if c not in df.columns]}")
   return df[orient_heading_col].notna() & df[rpm_col].isna()

# === How to identify a Temperature row (Temp ONLY)? ===
# Method T1: Temp has data, RPM is NaN, Orient Heading is NaN.
def is_temperature_row(df):
    required = temperature_cols + [rpm_col, orient_heading_col]
    if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for Temperature check: {[c for c in required if c not in df.columns]}")
    return df[temp_col].notna() & df[rpm_col].isna() & df[orient_heading_col].isna()

# === How to identify a Magnetometer row (Mag ONLY)? ===
# Method M1: Mag X has data, RPM is NaN, Orient Heading is NaN.
def is_magnetometer_row(df):
    required = magnetometer_cols + [rpm_col, orient_heading_col]
    if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for Magnetometer check: {[c for c in required if c not in df.columns]}")
    return df[mag_mx_col].notna() & df[rpm_col].isna() & df[orient_heading_col].isna()

# === How to identify a Gyroscope row (Gyro ONLY)? ===
# Method G1: Gyro X has data, RPM is NaN, Orient Heading is NaN.
def is_gyroscope_row(df):
    required = gyroscope_cols + [rpm_col, orient_heading_col]
    if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for Gyroscope check: {[c for c in required if c not in df.columns]}")
    return df[gyro_gx_col].notna() & df[rpm_col].isna() & df[orient_heading_col].isna()

# === How to identify an Accelerometer row (Accel ONLY)? ===
# Method A1: Accel X has data, RPM is NaN, Orient Heading is NaN.
def is_accelerometer_row(df):
    required = accelerometer_cols + [rpm_col, orient_heading_col]
    if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for Accelerometer check: {[c for c in required if c not in df.columns]}")
    return df[accel_ax_col].notna() & df[rpm_col].isna() & df[orient_heading_col].isna()

# === How to identify a Linear Acceleration row (Lin Accel ONLY)? ===
# Method L1: Lin Accel X has data, RPM is NaN, Orient Heading is NaN.
def is_linear_acceleration_row(df):
    required = linear_acceleration_cols + [rpm_col, orient_heading_col]
    if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for Linear Accel check: {[c for c in required if c not in df.columns]}")
    return df[lin_accel_lx_col].notna() & df[rpm_col].isna() & df[orient_heading_col].isna()

# === How to identify a Gravity row (Gravity ONLY)? ===
# Method Gr1: Gravity X has data, RPM is NaN, Orient Heading is NaN.
def is_gravity_row(df):
    required = gravity_cols + [rpm_col, orient_heading_col]
    if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for Gravity check: {[c for c in required if c not in df.columns]}")
    return df[grav_grx_col].notna() & df[rpm_col].isna() & df[orient_heading_col].isna()

# === How to identify a Voltage row (Voltage ONLY)? ===
# Method V1: Voltage has data, RPM is NaN, Orient Heading is NaN.
def is_voltage_row(df):
    required = voltage_cols + [rpm_col, orient_heading_col]
    if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for Voltage check: {[c for c in required if c not in df.columns]}")
    return df[voltage_col].notna() & df[rpm_col].isna() & df[orient_heading_col].isna()

# === How to identify a Current row (Current ONLY)? ===
# Method C1: Current has data, RPM is NaN, Orient Heading is NaN.
def is_current_row(df):
    required = current_cols + [rpm_col, orient_heading_col]
    if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for Current check: {[c for c in required if c not in df.columns]}")
    return df[current_col].notna() & df[rpm_col].isna() & df[orient_heading_col].isna()

# === How to identify a Power row (Power ONLY)? ===
# Method P1: Power has data, RPM is NaN, Orient Heading is NaN.
def is_power_row(df):
    required = power_cols + [rpm_col, orient_heading_col]
    if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for Power check: {[c for c in required if c not in df.columns]}")
    return df[power_col].notna() & df[rpm_col].isna() & df[orient_heading_col].isna()

# === How to identify a Weather Speed row (WS Speed ONLY)? ===
# Method WsS1: WS Speed has data, RPM is NaN, Orient Heading is NaN.
def is_ws_speed_row(df):
    required = weather_speed_cols + [rpm_col, orient_heading_col]
    if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for WS Speed check: {[c for c in required if c not in df.columns]}")
    return df[ws_speed_col].notna() & df[rpm_col].isna() & df[orient_heading_col].isna()

# === How to identify a Weather Pressure row (WS Pressure ONLY)? ===
# Method WsP1: WS Pressure has data, RPM is NaN, Orient Heading is NaN.
def is_ws_pressure_row(df):
    required = weather_pressure_cols + [rpm_col, orient_heading_col]
    if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for WS Pressure check: {[c for c in required if c not in df.columns]}")
    return df[ws_pressure_col].notna() & df[rpm_col].isna() & df[orient_heading_col].isna()

# === How to identify a Weather Altitude row (WS Altitude ONLY)? ===
# Method WsA1: WS Altitude has data, RPM is NaN, Orient Heading is NaN.
def is_ws_altitude_row(df):
    required = weather_altitude_cols + [rpm_col, orient_heading_col]
    if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for WS Altitude check: {[c for c in required if c not in df.columns]}")
    return df[ws_altitude_col].notna() & df[rpm_col].isna() & df[orient_heading_col].isna()

# === How to identify a Weather Humidity row (WS Humidity ONLY)? === (NEW)
# Method WsH1: WS Humidity has data, RPM is NaN, Orient Heading is NaN.
def is_ws_humidity_row(df):
    required = weather_humidity_cols + [rpm_col, orient_heading_col]
    if not all(c in df.columns for c in required): raise ValueError(f"Missing columns for WS Humidity check: {[c for c in required if c not in df.columns]}")
    return df[ws_humidity_col].notna() & df[rpm_col].isna() & df[orient_heading_col].isna()


# --- End Configuration ---

# Helper function for merging step
def merge_sensor_data(df_rpm, df_sensor, sensor_cols, sensor_suffix):
    """Merges RPM data with nearest sensor data within tolerance."""
    if df_rpm is None or df_rpm.empty:
        print(f"Error in merge_sensor_data: df_rpm is empty or None for {sensor_suffix}.")
        return pd.DataFrame(columns=[timestamp_col, rpm_col, timestamp_col + sensor_suffix] + sensor_cols)

    if df_sensor is None or df_sensor.empty:
        print(f"Skipping merge for {sensor_suffix} (no data).")
        merged = df_rpm[[timestamp_col, rpm_col]].copy()
        ts_col_renamed = timestamp_col + sensor_suffix
        merged[ts_col_renamed] = pd.NaT
        for col in sensor_cols: merged[col] = np.nan
        valid_sensor_cols = [c for c in sensor_cols if isinstance(c, str)]
        return merged[[timestamp_col, rpm_col, ts_col_renamed] + valid_sensor_cols]


    print(f"Matching RPM to nearest {sensor_suffix}...")
    ts_col_renamed = timestamp_col + sensor_suffix
    if timestamp_col not in df_sensor.columns: raise ValueError(f"Timestamp column '{timestamp_col}' not found in sensor data for {sensor_suffix}")
    df_sensor_renamed = df_sensor.rename(columns={timestamp_col: ts_col_renamed})
    if ts_col_renamed not in df_sensor_renamed.columns: raise ValueError(f"Renamed timestamp column '{ts_col_renamed}' failed for {sensor_suffix}")

    merged = pd.merge_asof(
        df_rpm,
        df_sensor_renamed,
        left_on=timestamp_col,
        right_on=ts_col_renamed,
        direction='nearest',
        tolerance=pd.Timedelta(seconds=max_time_difference_seconds)
    )
    print(f"Found {merged[ts_col_renamed].notna().sum()} {sensor_suffix} matches within tolerance.")
    final_sensor_cols = [col for col in sensor_cols if col in merged.columns]
    cols_to_keep = [timestamp_col, rpm_col, ts_col_renamed] + final_sensor_cols
    cols_to_keep = [col for col in cols_to_keep if col in merged.columns]
    return merged[cols_to_keep]


try:
    # --- 1. Load Data ---
    print(f"Attempting to load data from: {file_path}")
    try:
        df_raw = pd.read_csv(file_path, low_memory=False)
    except UnicodeDecodeError:
        print("UTF-8 decoding failed, trying latin1 encoding.")
        df_raw = pd.read_csv(file_path, encoding='latin1', low_memory=False)
    print(f"Successfully loaded data. Shape: {df_raw.shape}")

    # --- 2. Prepare Timestamps ---
    print(f"Converting timestamp column: '{timestamp_col}'")
    if timestamp_col not in df_raw.columns: raise ValueError(f"Timestamp column '{timestamp_col}' not found.")
    df_raw[timestamp_col] = pd.to_datetime(df_raw[timestamp_col], errors='coerce')
    initial_rows = len(df_raw)
    df_raw = df_raw.dropna(subset=[timestamp_col])
    rows_after_ts_dropna = len(df_raw)
    if initial_rows > rows_after_ts_dropna: print(f"Warning: Removed {initial_rows - rows_after_ts_dropna} rows due to invalid timestamps.")
    if df_raw.empty: raise ValueError("No valid data remaining after timestamp conversion/cleaning.")
    print("Timestamp conversion complete.")

    # --- 3. Separate Data into FIFTEEN Types ---
    print("Separating data by type...")
    data_frames = {}
    filters = {}
    # Define all data type configurations
    configs = {
        'rpm': {'func': is_rpm_row, 'cols': [rpm_col]},
        'orient': {'func': is_orientation_row, 'cols': orientation_cols},
        'temp': {'func': is_temperature_row, 'cols': temperature_cols},
        'mag': {'func': is_magnetometer_row, 'cols': magnetometer_cols},
        'gyro': {'func': is_gyroscope_row, 'cols': gyroscope_cols},
        'accel': {'func': is_accelerometer_row, 'cols': accelerometer_cols},
        'linaccel': {'func': is_linear_acceleration_row, 'cols': linear_acceleration_cols},
        'grav': {'func': is_gravity_row, 'cols': gravity_cols},
        'volt': {'func': is_voltage_row, 'cols': voltage_cols},
        'curr': {'func': is_current_row, 'cols': current_cols},
        'power': {'func': is_power_row, 'cols': power_cols},
        'ws_spd': {'func': is_ws_speed_row, 'cols': weather_speed_cols},
        'ws_press': {'func': is_ws_pressure_row, 'cols': weather_pressure_cols},
        'ws_alt': {'func': is_ws_altitude_row, 'cols': weather_altitude_cols},
        'ws_hum': {'func': is_ws_humidity_row, 'cols': weather_humidity_cols} # NEW
    }

    # Check existence of all columns needed by any filter upfront
    all_needed_primary_cols = set([timestamp_col, rpm_col, orient_heading_col, temp_col,
                                  mag_mx_col, gyro_gx_col, accel_ax_col, lin_accel_lx_col,
                                  grav_grx_col, voltage_col, current_col, power_col,
                                  ws_speed_col, ws_pressure_col, ws_altitude_col, ws_humidity_col]) # Added humidity
    missing_essential = [c for c in all_needed_primary_cols if c not in df_raw.columns]
    if missing_essential:
        print(f"Warning: Essential columns missing for some filtering logic, these types may not be separable: {missing_essential}")


    for key, config in configs.items():
        print(f"Applying filter for {key}...")
        try:
             primary_col_for_filter = config['cols'][0]
             if primary_col_for_filter not in df_raw.columns:
                  print(f"Skipping {key}: Primary column '{primary_col_for_filter}' not found.")
                  filters[key] = pd.Series([False] * len(df_raw))
                  data_frames[key] = pd.DataFrame(columns=[timestamp_col] + config['cols'])
                  continue

             filters[key] = config['func'](df_raw)
             cols_exist = [col for col in config['cols'] if col in df_raw.columns]
             if len(cols_exist) != len(config['cols']): print(f"Warning: Not all configured columns for {key} found: Missing {[c for c in config['cols'] if c not in cols_exist]}")
             cols_to_keep = [timestamp_col] + cols_exist
             data_frames[key] = df_raw.loc[filters[key], cols_to_keep].copy()
             print(f"Found {len(data_frames[key])} potential {key} rows.")
             if key != 'rpm': print(f"{key.capitalize()} columns selected: {data_frames[key].columns.tolist()}")
        except Exception as e:
             print(f"Error during separation for {key}: {e}")
             data_frames[key] = pd.DataFrame(columns=[timestamp_col] + config['cols'])
             filters[key] = pd.Series([False] * len(df_raw))


    # --- Sanity Check ---
    print("\nChecking for overlapping row identifications...")
    filter_keys = list(filters.keys())
    overlap_found = False
    for i in range(len(filter_keys)):
        for j in range(i + 1, len(filter_keys)):
            key1, key2 = filter_keys[i], filter_keys[j]
            if isinstance(filters.get(key1), pd.Series) and isinstance(filters.get(key2), pd.Series):
                 if filters[key1].any() and filters[key2].any(): # Avoid checking empty filters
                     overlap = df_raw[filters[key1] & filters[key2]]
                     if not overlap.empty:
                          print(f"Warning: {len(overlap)} rows identified as BOTH {key1.upper()} and {key2.upper()}.")
                          overlap_found = True
            else:
                 print(f"Warning: Could not check overlap between {key1} and {key2} due to filter issues.")
    if overlap_found:
        print("This indicates problems with the 'is_..._row' logic definitions. Results may be incorrect.")


    # --- Proceed only if we have RPM data ---
    df_rpm = data_frames['rpm']
    if df_rpm.empty:
        print("\nNo RPM data rows were identified. Cannot perform matching.")
    else:
        # --- 4. Sort Data ---
        print("Sorting data by timestamp...")
        df_rpm = df_rpm.sort_values(timestamp_col)
        for key in data_frames:
            if key != 'rpm' and data_frames.get(key) is not None and not data_frames[key].empty:
                data_frames[key] = data_frames[key].sort_values(timestamp_col)

        # --- 5. Perform Nearest Neighbor Merges ---
        merge_results = {}
        # Define sensor groups for merging
        sensor_groups = {
            'orient': {'df_key': 'orient', 'cols': orientation_cols, 'suffix': '_orient'},
            'temp': {'df_key': 'temp', 'cols': temperature_cols, 'suffix': '_temp'},
            'mag': {'df_key': 'mag', 'cols': magnetometer_cols, 'suffix': '_mag'},
            'gyro': {'df_key': 'gyro', 'cols': gyroscope_cols, 'suffix': '_gyro'},
            'accel': {'df_key': 'accel', 'cols': accelerometer_cols, 'suffix': '_accel'},
            'linaccel': {'df_key': 'linaccel', 'cols': linear_acceleration_cols, 'suffix': '_linaccel'},
            'grav': {'df_key': 'grav', 'cols': gravity_cols, 'suffix': '_grav'},
            'volt': {'df_key': 'volt', 'cols': voltage_cols, 'suffix': '_volt'},
            'curr': {'df_key': 'curr', 'cols': current_cols, 'suffix': '_curr'},
            'power': {'df_key': 'power', 'cols': power_cols, 'suffix': '_power'},
            'ws_spd': {'df_key': 'ws_spd', 'cols': weather_speed_cols, 'suffix': '_ws_spd'},
            'ws_press': {'df_key': 'ws_press', 'cols': weather_pressure_cols, 'suffix': '_ws_press'},
            'ws_alt': {'df_key': 'ws_alt', 'cols': weather_altitude_cols, 'suffix': '_ws_alt'},
            'ws_hum': {'df_key': 'ws_hum', 'cols': weather_humidity_cols, 'suffix': '_ws_hum'} # NEW
        }

        for key, group_config in sensor_groups.items():
             merge_results[key] = merge_sensor_data(df_rpm, data_frames.get(group_config['df_key']), group_config['cols'], group_config['suffix'])


        # --- 6. Combine All Merge Results ---
        print("Combining all matched results...")
        base_df = df_rpm[[timestamp_col, rpm_col]].copy()
        dfs_to_combine = [base_df]
        for key, group_config in sensor_groups.items():
            result_df = merge_results.get(key)
            if result_df is not None and not result_df.empty:
                 ts_col = timestamp_col + group_config['suffix']
                 cols_to_select = [timestamp_col, rpm_col] + [ts_col] + [c for c in group_config['cols'] if c in result_df.columns]
                 cols_to_select = [c for c in cols_to_select if c in result_df.columns]
                 cols_to_select = list(dict.fromkeys(cols_to_select))
                 if timestamp_col in result_df.columns and rpm_col in result_df.columns:
                      dfs_to_combine.append(result_df[cols_to_select])
                 else: print(f"Warning: Merge result for {key} missing key columns, cannot include in final combine.")
            else: print(f"Warning: Merge result for {key} is empty or None, cannot include in final combine.")

        if len(dfs_to_combine) > 1:
            try:
                final_df = reduce(lambda left, right: pd.merge(left, right, on=[timestamp_col, rpm_col], how='left'), dfs_to_combine)
                print("Combined all matches.")
            except KeyError as e: print(f"\nError combining results: Merge key missing - {e}"); final_df = pd.DataFrame()
            except Exception as e: print(f"\nError combining results: {e}"); final_df = pd.DataFrame()
        elif len(dfs_to_combine) == 1: final_df = dfs_to_combine[0]; print("Warning: Only base RPM data available, no sensor data could be merged.")
        else: final_df = pd.DataFrame(); print("Error: No data available to combine.")


        # --- 7. Display Results ---
        if not final_df.empty:
            print("-" * 200)

            # Define renaming for final output columns
            final_cols_rename_map = {
                timestamp_col: 'RPM Timestamp', rpm_col: 'RPM Value',
                # Orientation
                timestamp_col + '_orient': 'Nearest Orient TS', orient_heading_col: 'Orient H', orient_roll_col: 'Orient R', orient_pitch_col: 'Orient P',
                # Temperature
                timestamp_col + '_temp': 'Nearest Temp TS', temp_col: 'Temp C',
                # Magnetometer
                timestamp_col + '_mag': 'Nearest Mag TS', mag_mx_col: 'Mag X', mag_my_col: 'Mag Y', mag_mz_col: 'Mag Z',
                # Gyroscope
                timestamp_col + '_gyro': 'Nearest Gyro TS', gyro_gx_col: 'Gyro X', gyro_gy_col: 'Gyro Y', gyro_gz_col: 'Gyro Z',
                # Accelerometer
                timestamp_col + '_accel': 'Nearest Accel TS', accel_ax_col: 'Accel X', accel_ay_col: 'Accel Y', accel_az_col: 'Accel Z',
                # Linear Acceleration
                timestamp_col + '_linaccel': 'Nearest LinAccel TS', lin_accel_lx_col: 'LinAccel X', lin_accel_ly_col: 'LinAccel Y', lin_accel_lz_col: 'LinAccel Z',
                # Gravity
                timestamp_col + '_grav': 'Nearest Grav TS', grav_grx_col: 'Grav X', grav_gry_col: 'Grav Y', grav_grz_col: 'Grav Z',
                # Voltage
                timestamp_col + '_volt': 'Nearest Volt TS', voltage_col: 'Voltage',
                # Current
                timestamp_col + '_curr': 'Nearest Curr TS', current_col: 'Current',
                # Power
                timestamp_col + '_power': 'Nearest Power TS', power_col: 'Power',
                # Weather Station
                timestamp_col + '_ws_spd': 'Nearest WS Spd TS', ws_speed_col: 'WS Speed',
                timestamp_col + '_ws_press': 'Nearest WS Pres TS', ws_pressure_col: 'WS Pressure',
                timestamp_col + '_ws_alt': 'Nearest WS Alt TS', ws_altitude_col: 'WS Altitude',
                timestamp_col + '_ws_hum': 'Nearest WS Hum TS', ws_humidity_col: 'WS Humidity' # NEW
            }

            cols_to_select = [col for col in final_cols_rename_map.keys() if col in final_df.columns]
            output_df = final_df[cols_to_select].rename(columns=final_cols_rename_map)

            # Calculate time differences safely for each sensor type
            time_diff_configs = [
                 ('_orient', 'Orient'), ('_temp', 'Temp'), ('_mag', 'Mag'),
                 ('_gyro', 'Gyro'), ('_accel', 'Accel'), ('_linaccel', 'LinAccel'),
                 ('_grav', 'Grav'), ('_volt', 'Volt'), ('_curr', 'Curr'),
                 ('_power', 'Power'), ('_ws_spd', 'WS Spd'), ('_ws_press', 'WS Pres'),
                 ('_ws_alt', 'WS Alt'), ('_ws_hum', 'WS Hum') # NEW
            ]
            for suffix_key, display_name in time_diff_configs:
                sensor_ts_col = f'Nearest {display_name} TS'
                diff_col = f'{display_name} Time Diff (s)'
                if sensor_ts_col in output_df.columns and 'RPM Timestamp' in output_df.columns:
                    match_filter = output_df[sensor_ts_col].notna()
                    if diff_col not in output_df.columns: output_df[diff_col] = np.nan
                    if pd.api.types.is_datetime64_any_dtype(output_df[sensor_ts_col]) and \
                       pd.api.types.is_datetime64_any_dtype(output_df['RPM Timestamp']):
                        output_df.loc[match_filter, diff_col] = \
                            (output_df.loc[match_filter, sensor_ts_col] - output_df.loc[match_filter, 'RPM Timestamp']).dt.total_seconds()
                        print(f"Calculated {display_name} time difference.")
                    else: print(f"Warning: Could not calculate {display_name} time difference - timestamp columns have incorrect types.")
                else: print(f"Could not calculate {display_name} time difference (missing columns for {display_name}).")

            print("\nFinal Matched Data (NaN indicates no match within tolerance):")
            pd.set_option('display.max_columns', None)
            pd.set_option('display.width', 5000)
            print(output_df.to_string(index=False, na_rep='NaN'))
            pd.reset_option('display.max_columns')
            pd.reset_option('display.width')
            print("-" * 200)

            # --- 8. Save to Excel ---
            try:
                print(f"\nAttempting to save results to Excel file: {output_excel_file}")
                if 'output_df' in locals() and not output_df.empty:
                     for col in output_df.select_dtypes(include=['datetimetz']).columns:
                          print(f"Converting timestamp column '{col}' to offset-naive for Excel export.")
                          output_df[col] = output_df[col].dt.tz_localize(None)

                     output_df.to_excel(output_excel_file, index=False, engine='openpyxl')
                     print(f"Successfully saved results to {output_excel_file}")
                elif 'output_df' in locals() and output_df.empty: print("Skipping save to Excel: Final DataFrame is empty.")
                else: print("Skipping save to Excel: Final DataFrame ('output_df') was not created.")
            except ImportError: print("\nError: Could not save to Excel. The 'openpyxl' library is required (pip install openpyxl).")
            except Exception as e: print(f"\nError saving to Excel file {output_excel_file}: {e}")

        else: print("\nFinal DataFrame is empty, cannot display results or save to Excel.")


except FileNotFoundError: print(f"Error: The file was not found at specified path: {file_path}")
except ValueError as ve: print(f"Configuration or Data Error: {ve}")
except Exception as e:
    print(f"An unexpected error occurred: {type(e).__name__} - {e}")
    import traceback
    print(traceback.format_exc())

