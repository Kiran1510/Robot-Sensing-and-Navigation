from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation
import pandas as pd
import numpy as np

# Directly reading from .mcap file for quaternion data

mcappath = Path('driving_data_0.mcap')
typestore = get_typestore(Stores.ROS2_HUMBLE)

with AnyReader([mcappath], default_typestore=typestore) as reader:
    imu_connections = [x for x in reader.connections if x.topic == '/imu']
    
    for connection, timestamp, rawdata in reader.messages(connections=imu_connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        
        print("msg.imu attributes:")
        print(dir(msg.imu))
        print("\nchecking for orientation in msg.imu...")
        
        if hasattr(msg.imu, 'orientation'):
            print("found orientation!")
            print(f"  x: {msg.imu.orientation.x}")
            print(f"  y: {msg.imu.orientation.y}")
            print(f"  z: {msg.imu.orientation.z}")
            print(f"  w: {msg.imu.orientation.w}")
            
            # Extracting all quaternion data
            imu_data = {'time': [], 'quat_x': [], 'quat_y': [], 'quat_z': [], 'quat_w': []}
            
            for connection, timestamp, rawdata in reader.messages(connections=imu_connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                
                time_s = msg.header.stamp.sec
                time_ns = msg.header.stamp.nanosec
                time = time_s + time_ns/1e9
                
                imu_data['time'].append(time)
                imu_data['quat_x'].append(msg.imu.orientation.x)
                imu_data['quat_y'].append(msg.imu.orientation.y)
                imu_data['quat_z'].append(msg.imu.orientation.z)
                imu_data['quat_w'].append(msg.imu.orientation.w)
            
            # Converting to yaw
            df_imu = pd.DataFrame(imu_data)
            quaternions = np.column_stack([df_imu['quat_x'], df_imu['quat_y'], df_imu['quat_z'], df_imu['quat_w']])
            rotations = Rotation.from_quat(quaternions)
            euler_angles = rotations.as_euler('xyz', degrees=False)
            
            imu_yaw = euler_angles[:, 2]
            imu_yaw_unwrapped = np.unwrap(imu_yaw)
            
            df_imu['imu_yaw'] = imu_yaw_unwrapped
            df_imu.to_csv('imu_heading_data.csv', index=False)
            
            print(f"\nextracted {len(imu_yaw)} IMU heading estimates")
            print(f"IMU yaw range: {imu_yaw_unwrapped.min():.2f} to {imu_yaw_unwrapped.max():.2f} rad")
            print("saved to imu_heading_data.csv")
        else:
            print("no orientation in msg.imu")
            print("msg.imu type:", type(msg.imu))
        
        break
