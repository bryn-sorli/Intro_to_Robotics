import setup_path
import airsim

import time

from pyquaternion import Quaternion
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

DRONES = ["Drone1", "Drone2", "Drone3", "Drone4"]

data = {"x": [], "y": [], "z": []}

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

try:
    while True:
        for drone in DRONES:
            lidarData = client.getLidarData(vehicle_name=drone)
            if len(lidarData.point_cloud) >= 3: # valid point cloud (maybe % 3 == 0 instead)
                q = Quaternion(lidarData.pose.orientation.w_val, lidarData.pose.orientation.x_val, 
                    lidarData.pose.orientation.y_val, lidarData.pose.orientation.z_val) # quaternion (orientation) object
                p = lidarData.pose.position.x_val, lidarData.pose.position.y_val, lidarData.pose.position.z_val # position values
                for i in range(0, len(lidarData.point_cloud), 3):
                    dx, dy, dz = lidarData.point_cloud[i], lidarData.point_cloud[i+1], lidarData.point_cloud[i+2] # point cloud disparity values
                    rotatedDisparity = q.rotate(np.array([dx, dy, dz])) # rotate the point cloud disparity values
                    ap = rotatedDisparity + p
                    data["x"].append(ap[0])
                    data["y"].append(ap[1])
                    data["z"].append(-ap[2])
        time.sleep(.5)
except KeyboardInterrupt:
    print("Interrupted! Plotting...")
    pass

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(data["x"], data["y"], data["z"], c=data["z"], depthshade=False, cmap=cm.copper)
plt.show()