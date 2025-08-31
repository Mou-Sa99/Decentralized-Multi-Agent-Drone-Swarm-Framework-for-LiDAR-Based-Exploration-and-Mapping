from Common_Defs import *

def Map_3D_Structure(client,Drone_Name,LiDar_Name1,LiDar_Name2,altitude):

    wall_coordinates = []
    ned_wall_coordinates = []
    standoff_coordinate = []
    ned_front_obstacle_points = []
    i=0
    perimeter_check = True
    turn = 0
    
    while perimeter_check:
                
        obstacles_points = get_lidar_obstacles(client, "Drone_Name", "LiDar_Name1", max_range=10.0)
        Full_Vertical_points = get_lidar_obstacles(client, "Drone_Name", "LiDar_Name2", max_range=10.0)
        Surface_points = filter_front_0_45(Full_Vertical_points,max_distance=10.0)
        front_obstacle_points = filter_front_0_negative_45(obstacles_points,max_distance=10.0)
        if front_obstacle_points:
            ned_front_obstacle_points = [lidar_to_global_ned(client, p) for p in front_obstacle_points]
        if Surface_points:
            ned_Surface_points = [lidar_to_global_ned(client, p) for p in Surface_points]
        wall_coordinates = list(front_obstacle_points) + wall_coordinates
        if check_points_present(ned_front_obstacle_points, ned_wall_coordinates):
            print("Perimeter complete!")
            perimeter_check = False
            break
        ned_wall_coordinates = ned_front_obstacle_points + ned_wall_coordinates
        save_to_ply_file(ned_Surface_points, r"C:\Users\yendh\AirSim\PythonClient\multirotor\shared_map\global_map.ply")

        if front_obstacle_points == []:

            Ned_coordinates = Corner_turn_coordinates(ned_wall_coordinates[0], ned_wall_coordinates[1])
            Ned_coordinates[2] = altitude
            turn = 1
            

        else:
                
            Lidar_standoff_coordinate= Normal_point_from_surface(wall_coordinates[0],wall_coordinates[1]) 
            Ned_coordinates = lidar_to_global_ned(client, Lidar_standoff_coordinate)  
            Ned_coordinates[2] = altitude    

            
        standoff_coordinate = list(Ned_coordinates) + standoff_coordinate 
        client.moveToPositionAsync(Ned_coordinates[0] ,Ned_coordinates[1],Ned_coordinates[2], 3).join()
        print(f"Next standpoint coordinate: {Ned_coordinates} Wall coordinates: {lidar_to_global_ned(client, wall_coordinates[0])}")
        rotate_drone_toward_normal(client,Ned_coordinates, ned_wall_coordinates[0],turn)
    
        




