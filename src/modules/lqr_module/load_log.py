from pyulog import ULog
import pandas as pd

# Path to your ULog file
ulog_file_path = '/home/pawelj/PX4_log_files/log_1_2024-7-15-22-19-23.ulg'

# Load the ULog file
ulog = ULog(ulog_file_path)

# List all message types (topics)
print("Available topics:")
for msg_type in ulog.data_list:
    print(msg_type.name)

# Extract specific topics
def extract_topic(ulog, topic_name):
    try:
        data = ulog.get_dataset(topic_name).data
        df = pd.DataFrame(data)
        return df
    except KeyError:
        print(f"Topic {topic_name} not found in the ULog file.")
        return None

# Extracting specific topics
angular_velocity_df = extract_topic(ulog, 'vehicle_angular_velocity')
attitude_df = extract_topic(ulog, 'vehicle_attitude')
local_position_df = extract_topic(ulog, 'vehicle_local_position')

# Save the data to CSV files
if angular_velocity_df is not None:
    angular_velocity_df.to_csv('/home/pawelj/PX4_log_files/vehicle_angular_velocity.csv', index=False)
if attitude_df is not None:
    attitude_df.to_csv('/home/pawelj/PX4_log_files/vehicle_attitude.csv', index=False)
if local_position_df is not None:
    local_position_df.to_csv('/home/pawelj/PX4_log_files/vehicle_local_position.csv', index=False)

print("Data extraction complete.")
