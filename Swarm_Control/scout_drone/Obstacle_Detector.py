import airsim
import numpy as np
import time
import os
import open3d as o3d
from Common_Defs import get_lidar_obstacles, save_to_ply_file, load_from_ply_file


def scan_obstacles_at_altitude(client, drone_name="Scout_drone", lidar_name="Scout_drone_LiDAR", 
                             target_altitude=5, max_velocity=25, 
                             ply_filename="scout.drone.ply", max_range=100.0,
                             scan_360=True, scan_step=30):
    """
    Move drone to specified altitude and scan for obstacles using lidar.
    
    Args:
        client: AirSim MultirotorClient instance
        drone_name (str): Name of the drone (default: "drone1")
        lidar_name (str): Name of the lidar sensor (default: "lidar1")
        target_altitude (float): Target altitude in meters (default: 5)
        max_velocity (float): Maximum velocity for movement (default: 25)
        ply_filename (str): Output PLY file name (default: "scout.drone.ply")
        max_range (float): Maximum lidar detection range (default: 100.0)
        scan_360 (bool): Whether to perform 360-degree scan (default: True)
        scan_step (int): Angle step for 360-degree scan in degrees (default: 30)
    
    Returns:
        list: obstacle_point_cloud containing all detected obstacle points as [x, y, z]
    """
    
    # Initialize obstacle point cloud list
    obstacle_point_cloud = []
    
    try:
        # Setup drone
        print(f"Setting up {drone_name}...")
        
        # Take off and move to target altitude
        print(f"Taking off and moving to z={target_altitude}...")
        #client.takeoffAsync(vehicle_name=drone_name).join()
        client.moveToPositionAsync(0,0,-5,2).join()
        print(f"Reached target altitude z={target_altitude}")
        
        # Initial lidar scan
        print(f"Scanning for obstacles with {lidar_name}...")
        obstacle_points = get_lidar_obstacles(client, drone_name, lidar_name, max_range)
        obstacle_point_cloud.extend(obstacle_points)
        print(f"Initial scan detected {len(obstacle_points)} obstacle points")
        
        # Perform 360-degree scan if requested
        if scan_360:
            print("Performing 360-degree scan...")
            scan_angles = np.arange(0, 360, scan_step)
            
            for angle in scan_angles:
                # Rotate drone to scan different directions
                client.rotateToYawAsync(angle, vehicle_name=drone_name).join()
                time.sleep(0.5)  # Allow time for rotation and stabilization
                
                # Get lidar data at this angle
                new_points = get_lidar_obstacles(client, drone_name, lidar_name, max_range)
                
                # Add new obstacle points (avoid duplicates)
                for point in new_points:
                    if not is_duplicate_point(point, obstacle_point_cloud, threshold=0.1):
                        obstacle_point_cloud.append(point)
                
                print(f"Scan at {angle}°: {len(new_points)} new points, total: {len(obstacle_point_cloud)}")
        
        # Load existing PLY file and combine with new data
        existing_points = load_from_ply_file(ply_filename)
        if existing_points:
            print(f"Loaded {len(existing_points)} existing points from {ply_filename}")
            # Combine with new points (remove duplicates)
            for point in existing_points:
                if not is_duplicate_point(point, obstacle_point_cloud, threshold=0.1):
                    obstacle_point_cloud.append(point)
        
        # Save final point cloud to PLY file
        save_to_ply_file(obstacle_point_cloud, ply_filename)
        
        print(f"Total obstacle points detected: {len(obstacle_point_cloud)}")
        
        # Display sample points
        if obstacle_point_cloud:
            print("\nSample obstacle points:")
            for i, point in enumerate(obstacle_point_cloud[:5]):
                print(f"Point {i+1}: x={point[0]:.2f}, y={point[1]:.2f}, z={point[2]:.2f}")
        
        return obstacle_point_cloud
        
    except Exception as e:
        print(f"Error during obstacle scanning: {e}")
        return obstacle_point_cloud


def is_duplicate_point(new_point, existing_points, threshold=0.1):
    """
    Check if a point is too close to existing points (duplicate).
    
    Args:
        new_point (list): New point [x, y, z]
        existing_points (list): List of existing points
        threshold (float): Distance threshold for duplicate detection
    
    Returns:
        bool: True if duplicate, False if new point
    """
    new_point_array = np.array(new_point)
    for existing_point in existing_points:
        if np.linalg.norm(new_point_array - np.array(existing_point)) < threshold:
            return True
    return False




