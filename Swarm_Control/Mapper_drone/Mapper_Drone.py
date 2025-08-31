import airsim
import numpy as np
import time
import os
import open3d as o3d
from Common_Defs import *
import json
from Mapper_Drone_assembly import assemble_drones_vertical
from Surface_Tracer import Map_3D_Structure
import multiprocessing as mp


def run_mapper(vehicle_name: str, lidar1: str, lidar2: str):
    """Process entry: create its own AirSim client and run mapping."""
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True, vehicle_name=vehicle_name)
    client.armDisarm(True, vehicle_name=vehicle_name)
    # Optional: takeoff if your mapper expects air-start
    # client.takeoffAsync(vehicle_name=vehicle_name).join()

    try:
        Map_3D_Structure(client, vehicle_name, lidar1, lidar2)
    finally:
        # Optional tidy-up
        # client.landAsync(vehicle_name=vehicle_name).join()
        client.armDisarm(False, vehicle_name=vehicle_name)
        client.enableApiControl(False, vehicle_name=vehicle_name)




# Usage example:
if __name__ == "__main__":



    # Connect to AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    mp.freeze_support()      # safe on Windows executables

    configs = [
        ("Mapper_drone1", "Mapper_drone1_LiDAR1", "Mapper_drone1_LiDAR2"),
        ("Mapper_drone2", "Mapper_drone2_LiDAR1", "Mapper_drone2_LiDAR2"),
    ]



    FILE_PATH = r"C:\Users\yendh\AirSim\PythonClient\multirotor\shared_map\structure_flags.json"
    Structure_height_reached = True
    i=0

    # Load JSON file
    with open(FILE_PATH, "r") as f:
        data = json.load(f)

    # Loop through indices in order
    for idx in sorted(data.keys(), key=int):
          # sort ensures "0","1","2",... order
        point = data[idx]
        while Structure_height_reached:

            print(assemble_drones_vertical(client, point[0], point[1], point[2], upper_drone="Mapper_drone1",iteration=i))
            process = []
            for vehicle_name, l1, l2 in configs:
                p = mp.Process(target=run_mapper, args=(vehicle_name, l1, l2), daemon=False)
                p.start()
                process.append(p)

            # Wait for both to finish
            for p in process:
                p.join()
            




