import math

def sort_points_by_distance(points, reference_point):
    """
    Sort a list of 3D points based on distance from a reference point, nearest first.
    
    Args:
        points (list): List of points as [(x,y,z), (x,y,z), ...] or [[x,y,z], [x,y,z], ...]
        reference_point (tuple/list): Reference point as (x0,y0,z0) or [x0,y0,z0]
    
    Returns:
        list: Sorted points by distance from reference point, nearest first
    """
    
    if not points:
        return []
    
    if not reference_point or len(reference_point) != 3:
        return points
    
    # Calculate distances and create list of (point, distance) tuples
    points_with_distances = []
    
    for point in points:
        if len(point) != 3:
            continue
            
        # Calculate Euclidean distance
        dx = float(point[0]) - float(reference_point[0])
        dy = float(point[1]) - float(reference_point[1])
        dz = float(point[2]) - float(reference_point[2])
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        points_with_distances.append((point, distance))
    
    # Sort by distance (nearest first)
    points_with_distances.sort(key=lambda x: x[1])
    
    # Extract sorted points
    sorted_points = [item[0] for item in points_with_distances]
    
    return sorted_points