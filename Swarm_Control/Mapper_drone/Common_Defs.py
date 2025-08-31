import open3d as o3d
import numpy as np
import os
import airsim
import numpy as np
import math



def get_lidar_obstacles(client, drone_name, lidar_name, max_range):
    """
    Get obstacle points from lidar data.
    
    Args:
        client: AirSim client
        drone_name (str): Name of the drone
        lidar_name (str): Name of the lidar sensor
        max_range (float): Maximum detection range
    
    Returns:
        list: List of obstacle points as [x, y, z]
    """
    obstacle_points = []
    
    try:
        # Get lidar data
        lidar_data = client.getLidarData(lidar_name=lidar_name, vehicle_name=drone_name)
        
        if len(lidar_data.point_cloud) >= 3:
            # Convert lidar data to numpy array and reshape
            points = np.array(lidar_data.point_cloud, dtype=np.float32)
            points = points.reshape(-1, 3)  # Reshape to (N, 3) for x,y,z coordinates
            
            # Filter out invalid points
            valid_points = points[~np.isnan(points).any(axis=1)]
            
            # Filter points within range
            distances = np.linalg.norm(valid_points, axis=1)
            filtered_points = valid_points[distances < max_range]
            
            # Convert to list format
            for point in filtered_points:
                obstacle_points.append([float(point[0]), float(point[1]), float(point[2])])
    
    except Exception as e:
        print(f"Error getting lidar data: {e}")
    
    return obstacle_points







def save_to_ply_file(points, filename):
    """
    Save point cloud to PLY file format.
    
    Args:
        points (list): List of points as [x, y, z]
        filename (str): Output filename
    """
    try:
        with open(filename, 'w') as f:
            # PLY header
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            
            # Write points
            for point in points:
                f.write(f"{point[0]} {point[1]} {point[2]}\n")
        
        print(f"Obstacle point cloud saved to {filename}")
    
    except Exception as e:
        print(f"Error saving PLY file: {e}")






def load_from_ply_file(filename):
    """
    Load point cloud from PLY file.
    
    Args:
        filename (str): PLY file name
    
    Returns:
        list: List of points as [x, y, z]
    """
    points = []
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
            
            # Find end of header
            header_end = 0
            for i, line in enumerate(lines):
                if line.strip() == "end_header":
                    header_end = i + 1
                    break
            
            # Read points
            for line in lines[header_end:]:
                coords = line.strip().split()
                if len(coords) >= 3:
                    points.append([float(coords[0]), float(coords[1]), float(coords[2])])
    
    except FileNotFoundError:
        print(f"File {filename} not found")
    except Exception as e:
        print(f"Error loading PLY file: {e}")
    
    return points





def lidar_to_global_ned(client, lidar_coordinates):
    """
    Convert LIDAR coordinates to global NED coordinates.
    
    Args:
        client: AirSim client object
        lidar_coordinates: numpy array [x, y, z] in LIDAR/drone body frame
        
    Returns:
        numpy array [x, y, z] in global NED coordinates
    """
    # Get current drone pose
    pose = client.simGetVehiclePose()
    
    # Extract position (already in NED)
    drone_position = np.array([
        pose.position.x_val,
        pose.position.y_val, 
        pose.position.z_val
    ])
    
    # Extract orientation quaternion
    q = pose.orientation
    quat = np.array([q.w_val, q.x_val, q.y_val, q.z_val])
    
    # Convert quaternion to rotation matrix
    w, x, y, z = quat
    rotation_matrix = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])
    
    # Transform LIDAR coordinates to global frame
    global_coordinates = drone_position + rotation_matrix @ lidar_coordinates
    
    return global_coordinates




def check_points_present(list1, list2, tolerance=1e-6):
    """
    Check if 95% or more of 3D points in list1 are present in list2.
    
    Args:
        list1: List of 3D points to check
        list2: List of 3D points to search in
        tolerance: Tolerance for coordinate comparison (default: 1e-6)
    
    Returns:
        bool: True if 95% or more points from list1 are found in list2, False otherwise
    """
    import math
    
    def points_equal(p1, p2, tol):
        # Handle both tuple/list and AirSim Vector3r objects
        if hasattr(p1, 'x_val'):
            x1, y1, z1 = p1.x_val, p1.y_val, p1.z_val
        else:
            x1, y1, z1 = p1[0], p1[1], p1[2]
            
        if hasattr(p2, 'x_val'):
            x2, y2, z2 = p2.x_val, p2.y_val, p2.z_val
        else:
            x2, y2, z2 = p2[0], p2[1], p2[2]
            
        return (abs(x1 - x2) < tol and 
                abs(y1 - y2) < tol and 
                abs(z1 - z2) < tol)
    
    if not list1 or not list2:
        return False
    
    found_count = 0
    for point1 in list1:
        if any(points_equal(point1, point2, tolerance) for point2 in list2):
            found_count += 1
    
    match_percentage = (found_count / len(list1)) * 100
    return match_percentage >= 95.0



def filter_front_0_negative_45(obstacle_points, max_distance=None):
    """
    Keep LiDAR points with bearing in [0°, 60°] relative to +X (forward).
    AirSim body frame: +X=fwd, +Y=right, +Z=down.

    obstacle_points: list/array of [x,y,z]
    max_distance: optional cap on XY range (meters)
    Returns: list of [x,y,z] with plain Python floats
    """
    if not obstacle_points:
        return []

    pts = np.asarray(obstacle_points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("obstacle_points must be (N,3)")

    xy = pts[:, :2]
    ang_deg = np.degrees(np.arctan2(xy[:, 1], xy[:, 0]))  # [-180, 180], 0° = +X
    mask = (ang_deg <= 0.0) & (ang_deg >= -45.0)

    if max_distance is not None:
        mask &= (np.linalg.norm(xy, axis=1) <= float(max_distance))

    return [list(map(float, p)) for p in pts[mask]]



def filter_front_0_45(obstacle_points, max_distance=None):
    """
    Keep LiDAR points with bearing in [0°, 60°] relative to +X (forward).
    AirSim body frame: +X=fwd, +Y=right, +Z=down.

    obstacle_points: list/array of [x,y,z]
    max_distance: optional cap on XY range (meters)
    Returns: list of [x,y,z] with plain Python floats
    """
    if not obstacle_points:
        return []

    pts = np.asarray(obstacle_points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("obstacle_points must be (N,3)")

    xy = pts[:, :2]
    ang_deg = np.degrees(np.arctan2(xy[:, 1], xy[:, 0]))  # [-180, 180], 0° = +X
    mask = (ang_deg >= 0.0) & (ang_deg <= 45.0)

    if max_distance is not None:
        mask &= (np.linalg.norm(xy, axis=1) <= float(max_distance))

    return [list(map(float, p)) for p in pts[mask]]


def filter_front_0_180(obstacle_points, max_distance=None):
    """
    Keep LiDAR points with bearing in [0°, 60°] relative to +X (forward).
    AirSim body frame: +X=fwd, +Y=right, +Z=down.

    obstacle_points: list/array of [x,y,z]
    max_distance: optional cap on XY range (meters)
    Returns: list of [x,y,z] with plain Python floats
    """
    if not obstacle_points:
        return []

    pts = np.asarray(obstacle_points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("obstacle_points must be (N,3)")

    xy = pts[:, :2]
    ang_deg = np.degrees(np.arctan2(xy[:, 1], xy[:, 0]))  # [-180, 180], 0° = +X
    mask = (ang_deg <= 90) & (ang_deg >= -90.0)

    if max_distance is not None:
        mask &= (np.linalg.norm(xy, axis=1) <= float(max_distance))

    return [list(map(float, p)) for p in pts[mask]]




def Normal_point_from_surface(point1, point2, distance=5.0):
    """
    Given two 3D lidar points, finds a point 5m away from the first point
    along the perpendicular line, in the direction closer to [0,0].
    
    Args:
        point1: tuple (x, y, z) - first 3D point
        point2: tuple (x, y, z) - second 3D point
        distance: distance from first point (default 5.0 meters)
    
    Returns:
        tuple (x, y, z) where z=0 - single point 5m away toward origin
    """
    # Extract x, y coordinates from the 3D points
    x1, y1, _ = point1
    x2, y2, _ = point2
    
    # Calculate the direction vector of the original line
    dx = x2 - x1
    dy = y2 - y1
    
    # Handle special cases and calculate perpendicular direction vector
    if dx == 0:  # Original line is vertical
        # Perpendicular line is horizontal
        direction_x = 1.0
        direction_y = 0.0
    elif dy == 0:  # Original line is horizontal
        # Perpendicular line is vertical  
        direction_x = 0.0
        direction_y = 1.0
    else:
        # Calculate perpendicular slope: m_perp = -1/m
        slope = dy / dx
        perp_slope = -1 / slope
        
        # Create direction vector and normalize it
        direction_x = 1.0
        direction_y = perp_slope
        magnitude = math.sqrt(direction_x**2 + direction_y**2)
        direction_x /= magnitude
        direction_y /= magnitude
    
    # Calculate both possible points along the perpendicular line
    point_option1 = (x1 + distance * direction_x, y1 + distance * direction_y)
    point_option2 = (x1 - distance * direction_x, y1 - distance * direction_y)
    
    # Calculate distances to origin [0,0] for both options
    dist1_to_origin = math.sqrt(point_option1[0]**2 + point_option1[1]**2)
    dist2_to_origin = math.sqrt(point_option2[0]**2 + point_option2[1]**2)
    
    # Choose the point closer to origin [0,0]
    if dist1_to_origin <= dist2_to_origin:
        chosen_point = point_option1
    else:
        chosen_point = point_option2
    
    # Return with z=0
    return (chosen_point[0], chosen_point[1], 0)









def rotate_drone_toward_normal(client, current_position, point1,turn=0, duration=3.0, vehicle_name="Scout_drone"):
    
    
    try:
        # Direction vector drone -> target
        dx = point1[0] - current_position[0]
        dy = point1[1] - current_position[1]

        # Desired yaw (AirSim moveToPositionAsync expects degrees when yaw_mode given)
        target_yaw_deg = math.degrees(math.atan2(dy, dx))

        if target_yaw_deg > 10:

            # Command drone to stay in place but rotate
                client.moveByVelocityZAsync(
                    vx=0.0, vy=0.0, z=current_position[2],
                    duration=duration,
                    yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=target_yaw_deg),
                    vehicle_name=vehicle_name
                ).join()

        if turn == 1:

            client.moveByVelocityZAsync(
                    vx=0.0, vy=0.0, z=current_position[2],
                    duration=duration,
                    yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=90),
                    vehicle_name=vehicle_name
                ).join()


        return True
    except Exception as e:
        print(f"Error during drone rotation: {e}")
        return False








def Corner_turn_coordinates(point1, point2):
    """
    Takes two 3D LiDAR points, finds the line between them using x,y coordinates,
    and returns a point 5m away from point1 in the direction opposite to point2 
    (towards point1) with z=0.
    
    Args:
        point1: [x, y, z] - First 3D point
        point2: [x, y, z] - Second 3D point
    
    Returns:
        [x, y, 0] - Point 5m away from point1 opposite to point2 with z=0
    """
    # Extract x,y
    x1, y1 = point1[0], point1[1]
    x2, y2 = point2[0], point2[1]
    
    # Direction vector (reversed: from point2 -> point1)
    dx = x1 - x2
    dy = y1 - y2
    
    # Distance between points
    distance = math.sqrt(dx**2 + dy**2)
    
    if distance == 0:
        return [x1, y1, 0]
    
    # Unit direction vector (reversed)
    unit_dx = dx / distance
    unit_dy = dy / distance
    
    # Move 5 units from point1 backwards along line
    new_x = x1 + 3* unit_dx
    new_y = y1 + 3* unit_dy
    
    return [new_x, new_y, 0]



def get_two_furthest_points_from_drone(points_list, client):
    """
    Find two points with the furthest distance from the drone position.
    
    Args:
        points_list: List of 3D coordinates [(x1,y1,z1), (x2,y2,z2), ...]
        client: AirSim client object
    
    Returns:
        Tuple of two 3D coordinates (furthest_point, second_furthest_point)
    """
    if len(points_list) < 2:
        return None
    
    # Get current drone position
    drone_state = client.getMultirotorState()
    drone_pos = drone_state.kinematics_estimated.position
    drone_position = (drone_pos.x_val, drone_pos.y_val, drone_pos.z_val)
    
    # Calculate distances from drone to each point
    distances = []
    for point in points_list:
        distance = ((point[0] - drone_position[0])**2 + 
                   (point[1] - drone_position[1])**2 + 
                   (point[2] - drone_position[2])**2)**0.5
        distances.append((distance, point))
    
    # Sort by distance (descending order)
    distances.sort(key=lambda x: x[0], reverse=True)
    
    # Return the two furthest points
    furthest_point = distances[0][1]
    second_furthest_point = distances[1][1]
    
    return (furthest_point, second_furthest_point)



def is_point_in_list(target_point, point_list, tolerance=0.1):
    
    
    # Extract coordinates from target point
    if hasattr(target_point, 'x_val'):  # Vector3r object
        tx, ty, tz = target_point.x_val, target_point.y_val, target_point.z_val
    else:  # tuple or list
        tx, ty, tz = target_point[0], target_point[1], target_point[2]
    
    for point in point_list:
        # Extract coordinates from list point
        if hasattr(point, 'x_val'):  # Vector3r object
            px, py, pz = point.x_val, point.y_val, point.z_val
        else:  # tuple or list
            px, py, pz = point[0], point[1], point[2]
        
        # Calculate Euclidean distance
        distance = math.sqrt((tx - px)**2 + (ty - py)**2 + (tz - pz)**2)
        
        if distance <= tolerance:
            return True
    
    return False