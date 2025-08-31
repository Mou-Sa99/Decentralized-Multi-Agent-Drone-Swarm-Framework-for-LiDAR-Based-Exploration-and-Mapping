import airsim
import asyncio

def assemble_drones_vertical(client, target_x, target_y, base_altitude, upper_drone="mapperdrone1", iteration=None):
    """
    Moves two drones to assemble 5m apart vertically at the same x,y coordinates in AirSim.
    
    Args:
        client: AirSim MultirotorClient instance
        target_x: X coordinate for both drones
        target_y: Y coordinate for both drones  
        base_altitude: Z coordinate for the lower drone (negative value for altitude)
        upper_drone: Which drone should be positioned higher ("mapperdrone1" or "mapperdrone2")
        iteration: Iteration number to adjust altitude
    
    Returns:
        dict: Status of positioning for both drones
    """
    
    # Calculate positions - 5m apart in height
    base_altitude = base_altitude - 10*iteration if iteration else base_altitude
    if upper_drone == "Mapper_drone1":
        pos1 = [target_x, target_y, base_altitude - 5]  # Higher (more negative z)
        pos2 = [target_x, target_y, base_altitude]      # Lower
    else:
        pos1 = [target_x, target_y, base_altitude]      # Lower  
        pos2 = [target_x, target_y, base_altitude - 5]  # Higher (more negative z)
    
    # Move both drones to positions
    client.moveToPositionAsync(pos1[0], pos1[1], pos1[2], 5, vehicle_name="Mapper_drone1").join()
    client.moveToPositionAsync(pos2[0], pos2[1], pos2[2], 5, vehicle_name="Mapper_drone2").join()
    
    return {
        "mapperdrone1_position": pos1,
        "mapperdrone2_position": pos2,
        "vertical_separation": 5.0,
        "status": "assembled"
    }