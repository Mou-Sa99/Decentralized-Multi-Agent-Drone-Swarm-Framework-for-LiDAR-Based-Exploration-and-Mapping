from Common_Defs import *


def map_obstacle_perimeter(client,PLY_FILE):

    ned_wall_coordinates = []
    standoff_coordinate = []
    ned_front_obstacle_points = []
    i=0
    perimeter_check = True
    while perimeter_check:
                
        obstacles_points = get_lidar_obstacles(client, "Scout_drone", "Scout_drone_LiDAR", max_range=5.0)
        front_obstacle_points = filter_front_0_45(obstacles_points,max_distance=5.0)
        if front_obstacle_points:
            ned_front_obstacle_points = [lidar_to_global_ned(client, p) for p in front_obstacle_points]
        if i==0:
            first_Front_obstacle_point = ned_front_obstacle_points[0]

        if is_point_in_list(first_Front_obstacle_point, ned_front_obstacle_points) and i> 0:
            print("Perimeter complete!")
            perimeter_check = False
            break
        ned_wall_coordinates = ned_front_obstacle_points + ned_wall_coordinates
        if front_obstacle_points == []:

            Ned_coordinates = Corner_turn_coordinates(ned_wall_coordinates[0], ned_wall_coordinates[1])
            Ned_coordinates[2] = -5.0
            

        else:
            p1,p2=get_two_furthest_points_from_drone(front_obstacle_points)    
            Lidar_standoff_coordinate= Normal_point_from_surface(p1,p2)
            Ned_coordinates = lidar_to_global_ned(client, Lidar_standoff_coordinate)  
            Ned_coordinates[2] = -5.0    

            
        standoff_coordinate = list(Ned_coordinates) + standoff_coordinate 
        client.moveToPositionAsync(Ned_coordinates[0] ,Ned_coordinates[1],Ned_coordinates[2], 3).join()
        print(f"Next standpoint coordinate: {Ned_coordinates} Wall coordinates: { ned_wall_coordinates[0]}")
        rotate_drone_toward_normal(client,Ned_coordinates, ned_wall_coordinates[0])
        #save_global_ned_points_to_ply(ned_front_obstacle_points, PLY_FILE)
        i= i+1
    
    save_global_ned_points_to_ply(ned_wall_coordinates, PLY_FILE)
    return ned_wall_coordinates
        




