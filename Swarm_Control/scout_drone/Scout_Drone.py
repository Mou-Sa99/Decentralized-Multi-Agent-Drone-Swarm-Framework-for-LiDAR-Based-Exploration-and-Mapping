import airsim
import numpy as np
import time
import os
import open3d as o3d
from Point_cloud_sorter import sort_points_by_distance
from Obstacle_Detector import scan_obstacles_at_altitude
from Perimeter_Mapper import map_obstacle_perimeter
from Common_Defs import lidar_to_global_ned,Normal_point_from_surface
from Write_to_JSON import write_point_to_json



def remove_nearby_points(list1, list2, tolerance=1e-6):
    
    filtered_list = []
    
    for point1 in list1:
        is_near_any = False
        
        for point2 in list2:
            # Calculate Euclidean distance between two 3D points
            distance = ((point1[0] - point2[0])**2 + 
                       (point1[1] - point2[1])**2 + 
                       (point1[2] - point2[2])**2)**0.5
            
            if distance <= tolerance:
                is_near_any = True
                break
        
        if not is_near_any:
            filtered_list.append(point1)
    
    return filtered_list

# Usage example:
if __name__ == "__main__":
    # Connect to AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True, vehicle_name = "Scout_drone")
    client.armDisarm(True, vehicle_name = "Scout_drone")

    obstacle_point_cloud_array=[]

    PLY_FILE = r"C:\Users\mouni\AirSim\PythonClient\multirotor\shared_map\scout_drone1.ply"

    if not os.path.exists(PLY_FILE):
        print("No .ply file found, creating one...")

        # Example points (replace with LiDAR data)
        points = np.random.rand(100, 3)  # 100 random (x, y, z) points

        # Create point cloud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Save as .ply
        o3d.io.write_point_cloud(PLY_FILE, pcd)
    
    Obstacle_present = True
    Obstacle_perimeter_list = []
    

    while Obstacle_present:
    # Scan for obstacles
        obstacle_point_cloud = scan_obstacles_at_altitude(
            client=client,
            drone_name="Scout_drone",
            lidar_name="Scout_drone_LiDAR",
            target_altitude=5,
            max_velocity=25,
            ply_filename= PLY_FILE,
            max_range=100.0,
            scan_360=True,
            scan_step=30
        )
        state = client.getMultirotorState(vehicle_name="Scout_drone")

        # Extract coordinates
        x = state.kinematics_estimated.position.x_val
        y = state.kinematics_estimated.position.y_val
        z = state.kinematics_estimated.position.z_val

        obstacle_point_cloud_array.extend(sort_points_by_distance(obstacle_point_cloud, (x, y, z)))

        print(f"Scanning complete! Found {len(obstacle_point_cloud)} obstacle points.")

        first_point = np.array(obstacle_point_cloud_array[0], dtype=float).tolist()
        
        #move_to_point_with_standoff(client, first_point, standoff=5.0, speed=3.0, keep_current_altitude=True)
        standoff_coordinate = lidar_to_global_ned(client, Normal_point_from_surface(np.array(obstacle_point_cloud_array[0], dtype=float).tolist(),np.array(obstacle_point_cloud_array[1], dtype=float).tolist()))
        standoff_coordinate[2] = -5.0
        client.moveToPositionAsync(standoff_coordinate[0] ,standoff_coordinate[1],standoff_coordinate[2], 3).join()

        write_point_to_json(standoff_coordinate, r"C:\Users\mouni\AirSim\PythonClient\multirotor\shared_map\structure_flags.json")

        print(f"Moving to first point: {first_point}")
        Obstacle_perimeter_coordinates = map_obstacle_perimeter(client,PLY_FILE)
       
        Obstacle_perimeter_list.append(Obstacle_perimeter_coordinates)

        remove_nearby_points(obstacle_point_cloud_array, Obstacle_perimeter_coordinates)
    



        