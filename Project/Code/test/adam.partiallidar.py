import setup_path 
import airsim

import sys
import time
import argparse

from pyquaternion import Quaternion
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

DRONES = ["Drone1", "Drone2"]

class SurveyNavigator:
    def __init__(self, args):
        self.boxsize = args.size
        self.stripewidth = args.stripewidth
        self.altitude = args.altitude
        self.velocity = args.speed
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        for drone in DRONES:
            self.client.enableApiControl(True, drone)

    def start(self):
        print("arming the drone...")
        for drone in DRONES:
            self.client.armDisarm(True, drone)
        
        landed = [self.client.getMultirotorState(vehicle_name=drone).landed_state for drone in DRONES]
        if all(l == airsim.LandedState.Landed for l in landed):
            print("taking off...")
            fs = [self.client.takeoffAsync(vehicle_name=drone) for drone in DRONES]
            list(map(lambda f: f.join(), fs))
        
        landed = [self.client.getMultirotorState(vehicle_name=drone).landed_state for drone in DRONES]
        if all(l == airsim.LandedState.Landed for l in landed):
            print("takeoff failed - check Unreal message log for details")
            return
        
        # AirSim uses NED coordinates so negative axis is up.
        z = -self.altitude

        print("climbing to altitude: " + str(self.altitude))
        fs = [self.client.moveToPositionAsync(0, 0, z, self.velocity, vehicle_name=drone) for drone in DRONES]
        list(map(lambda f: f.join(), fs))

        print("flying to four corners of survey box")
        fs = []
        # Separate into four quadrants, start in lower left of each quadrant
        starting = [(0, 0)]#, (-self.boxsize, 0), (-self.boxsize, -self.boxsize), (0, -self.boxsize)] # each element is (x, y)
        for (drone, (x, y)) in zip(DRONES, starting):
            fs.append(self.client.moveToPositionAsync(x, y, z, self.velocity, vehicle_name=drone))
        list(map(lambda f: f.join(), fs))

        # let it settle there a bit.
        fs = [self.client.hoverAsync(vehicle_name=drone) for drone in DRONES]
        list(map(lambda f: f.join(), fs))
        time.sleep(2)

        # after hovering we need to re-enabled api control for next leg of the trip
        for drone in DRONES:
            self.client.enableApiControl(True, vehicle_name=drone)

        # now compute the survey path required to fill the specific quadrant
        path = {drone: [] for drone in DRONES}
        distance = 0
        x = -self.boxsize
        while x < 0: # Drone 2 and Drone 3
            distance += self.boxsize
            path[DRONES[1]].extend([(x, i, z) for i in range(0, self.boxsize, 2)]) # Drone 2
            # path[DRONES[2]].append(airsim.Vector3r(x, 0, z)) # Drone 3
            x += self.stripewidth
            distance += self.stripewidth
            path[DRONES[1]].extend([(x-self.stripewidth+i, self.boxsize, z) for i in range(0, self.stripewidth, 2)]) # Drone 2
            # path[DRONES[2]].append(airsim.Vector3r(x, 0, z)) # Drone 3
            distance += self.boxsize
            path[DRONES[1]].extend([(x, self.boxsize-i, z) for i in range(0, self.boxsize, 2)]) # Drone 2
            # path[DRONES[2]].append(airsim.Vector3r(x, -self.boxsize, z)) # Drone 3
            x += self.stripewidth
            distance += self.stripewidth
            path[DRONES[1]].extend([(x-self.stripewidth+i, 0, z) for i in range(0, self.stripewidth, 2)]) # Drone 2
            # path[DRONES[2]].append(airsim.Vector3r(x, -self.boxsize, z)) # Drone 3
            distance += self.boxsize
        while x < self.boxsize: # Drone 1 and Drone 4
            path[DRONES[0]].extend([(x, i, z) for i in range(0, self.boxsize, 2)]) # Drone 1
            # path[DRONES[3]].append(airsim.Vector3r(x, 0, z)) # Drone 4
            x += self.stripewidth
            path[DRONES[0]].extend([(x-self.stripewidth+i, self.boxsize, z) for i in range(0, self.stripewidth, 2)]) # Drone 1
            # path[DRONES[3]].append(airsim.Vector3r(x, 0, z)) # Drone 4
            # logical padding
            path[DRONES[0]].extend([(x, self.boxsize-i, z) for i in range(0, self.boxsize, 2)]) # Drone 1
            # path[DRONES[3]].append(airsim.Vector3r(x, -self.boxsize, z)) # Drone 4
            x += self.stripewidth
            path[DRONES[0]].extend([(x-self.stripewidth+i, 0, z) for i in range(0, self.stripewidth, 2)]) # Drone 1
            # path[DRONES[3]].append(airsim.Vector3r(x, -self.boxsize, z)) # Drone 4
        
        print("starting survey, estimated distance is " + str(distance))
        trip_time = distance / self.velocity
        print("estimated survey time is " + str(trip_time))
        try:
            print(path[DRONES[0]])
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            X, Y, Z = [], [], []
            for i in range(len(path[DRONES[0]])):
                fs = [self.client.moveToPositionAsync(*path[drone][i], self.velocity, vehicle_name=drone) for drone in DRONES]
                results = list(map(lambda f: f.join(), fs))
                lidarData = self.client.getLidarData()
                if len(lidarData.point_cloud) >= 3: # valid point cloud (maybe % 3 == 0 instead)
                    q = Quaternion(lidarData.pose.orientation.w_val, lidarData.pose.orientation.x_val, 
                        lidarData.pose.orientation.y_val, lidarData.pose.orientation.z_val) # quaternion (orientation) object
                    p = lidarData.pose.position.x_val, lidarData.pose.position.y_val, lidarData.pose.position.z_val # position values
                    for i in range(0, len(lidarData.point_cloud), 3):
                        dx, dy, dz = lidarData.point_cloud[i], lidarData.point_cloud[i+1], lidarData.point_cloud[i+2] # point cloud disparity values
                        rotatedDisparity = q.rotate(np.array([dx, dy, dz])) # rotate the point cloud disparity values
                        ap = rotatedDisparity + p
                        # ax.scatter(ap[0], ap[1], -ap[2], c=[(0, 0, np.abs(ap[2] / self.altitude))])
                        X.append(ap[0])
                        Y.append(ap[1])
                        Z.append(-ap[2])
            ax.scatter(X, Y, Z, c=Z, depthshade=False, cmap=cm.winter)
            plt.show()
        except:
            errorType, value, traceback = sys.exc_info()
            print("moveOnPath threw exception: " + str(value))
            pass

        print("flying back home")
        fs = [self.client.moveToPositionAsync(0, 0, z, self.velocity, vehicle_name=drone) for drone in DRONES]
        list(map(lambda f: f.join(), fs))
        
        if z < -1:
            print("descending")
            fs = [self.client.moveToPositionAsync(0, 0, -1, 2, vehicle_name=drone) for drone in DRONES]
            list(map(lambda f: f.join(), fs))

        print("landing...")
        fs = [self.client.landAsync(vehicle_name=drone) for drone in DRONES]
        list(map(lambda f: f.join(), fs))

        print("disarming.")
        for drone in DRONES:
            self.client.armDisarm(False, drone)

if __name__ == "__main__":
    args = sys.argv
    args.pop(0)
    arg_parser = argparse.ArgumentParser("Usage: survey boxsize stripewidth altitude")
    arg_parser.add_argument("--size", type=float, help="size of the box to survey", default=50)
    arg_parser.add_argument("--stripewidth", type=float, help="stripe width of survey (in meters)", default=10)
    arg_parser.add_argument("--altitude", type=float, help="altitude of survey (in positive meters)", default=30)
    arg_parser.add_argument("--speed", type=float, help="speed of survey (in meters/second)", default=5)
    args = arg_parser.parse_args(args)
    nav = SurveyNavigator(args)
    nav.start()