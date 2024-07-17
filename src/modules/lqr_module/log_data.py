import pandas as pd
import numpy as np
from scipy.signal import savgol_filter

# Load the data
attitude_data = pd.read_csv('/home/pawelj/Git_repos/PX4-Autopilot/src/modules/lqr_module/vehicle_attitude.csv')
position_data = pd.read_csv('/home/pawelj/Git_repos/PX4-Autopilot/src/modules/lqr_module/vehicle_local_position.csv')
angular_velocity_data = pd.read_csv('/home/pawelj/Git_repos/PX4-Autopilot/src/modules/lqr_module/vehicle_angular_velocity.csv')

# Merge the data on 'timestamp'
combined_data = pd.merge_asof(attitude_data, position_data, on='timestamp')
combined_data = pd.merge_asof(combined_data, angular_velocity_data, on='timestamp')

# Ensure 'timestamp' is treated as an integer
combined_data['timestamp'] = combined_data['timestamp'].astype(np.int64)

# Convert NED to ENU for position
combined_data['x'], combined_data['y'], combined_data['z'] = combined_data['y'], combined_data['x'], -combined_data['z']

# Convert NED to ENU for quaternion
combined_data['q[0]'], combined_data['q[1]'], combined_data['q[2]'], combined_data['q[3]'] = combined_data['q[0]'], combined_data['q[2]'], combined_data['q[1]'], -combined_data['q[3]']

# Convert NED to ENU for angular velocity
combined_data['xyz[0]'], combined_data['xyz[1]'], combined_data['xyz[2]'] = combined_data['xyz[1]'], combined_data['xyz[0]'], -combined_data['xyz[2]']

# Apply Savitzky-Golay filter to smooth the data
window_length = 51  # Choose an odd number
polyorder = 3  # Polynomial order

for col in ['x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az', 'q[0]', 'q[1]', 'q[2]', 'q[3]', 'xyz[0]', 'xyz[1]', 'xyz[2]']:
    combined_data[col] = savgol_filter(combined_data[col], window_length, polyorder)

# Interpolate to ensure a consistent timestamp difference of 100000 microseconds
combined_data.set_index('timestamp', inplace=True)
new_timestamps = np.arange(combined_data.index.min(), combined_data.index.max(), 1000000)
combined_data = combined_data.reindex(new_timestamps)
combined_data = combined_data.interpolate(method='index')

# Reset index to move the timestamp back to a column
combined_data.reset_index(inplace=True)
combined_data.rename(columns={'index': 'timestamp'}, inplace=True)

# Select and rename the specific columns
columns_to_keep = [
    'timestamp', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az',
    'q[0]', 'q[1]', 'q[2]', 'q[3]', 'xyz[0]', 'xyz[1]', 'xyz[2]'
]
combined_data = combined_data[columns_to_keep]

# Save the combined and interpolated data to a new CSV file
combined_data.to_csv('/home/pawelj/Git_repos/PX4-Autopilot/src/modules/lqr_module/log_data_interpolated.csv', index=False, float_format='%.14f')
