'''
    Convert all lidar frame from lidar sensor coordinate system into global world map coordinate system
'''
from operator import gt
from webbrowser import get
import carla
import random
import os
from queue import Queue
from queue import Empty
import shutil
import pygame
import numpy as np
import open3d as o3d
import copy




output_path = '/home/feng/testData'
if not os.path.exists(output_path):
    os.mkdir(output_path)



output_path_lidar = os.path.join(output_path,'lidar')
output_path_image = os.path.join(output_path,'image')
gnss_file_path = os.path.join(output_path,'gnss.txt')
imu_file_path = os.path.join(output_path,'imu.txt')
gt_location_file_path = os.path.join(output_path,'gt_location.txt')
lidar_transform_file_path = os.path.join(output_path, 'lidar_transform.txt')

LIDAR_ROTATION_FREQUENCY = 10
FIXED_TIME_STEP = 0.025
IMU_SENSOR_TICK = 0.025
GNSS_SENSOR_TICK = 0.025
CAMERA_SENSOR_TICK = 0.025
LIDAR_SENSOR_TICK = 0.025


# LIDAR_ROTATION_FREQUENCY = 20
# FIXED_TIME_STEP = 0.05
# IMU_SENSOR_TICK = 0.05
# GNSS_SENSOR_TICK = 0.05
# CAMERA_SENSOR_TICK = 0.05
# LIDAR_SENSOR_TICK = 0.05

lidar_transform_numpy = np.empty((0,4,4))

#############################################################Create all storage file#############################################################
def makedir(dir):
    if os.path.exists(dir):
        shutil.rmtree(dir)
    os.makedirs(dir)

# def makefile(file_path):
#     if os.path.exists(file_path):
#         with open(file_path,'w') as f:
#             f.truncate(0)
#         f.close()
#     fp = open(file_path, 'a')
#     return fp


# create directory for outputs
if not os.path.exists(output_path):
    os.makedirs(output_path)

makedir(output_path_image)
makedir(output_path_lidar)


# GNSS save file
if os.path.exists(gnss_file_path):
    with open(gnss_file_path,'w') as f:
        f.truncate(0)
    f.close()
fp_gnss = open(gnss_file_path, 'a')
fp_gnss.write('frame,timestamp,latitude,longitude,altitude\n')



# IMU save file
if os.path.exists(imu_file_path):
    with open(imu_file_path, 'w') as f:
        f.truncate(0)
    f.close()
fp_imu = open(imu_file_path, 'a')
fp_imu.write('frame,timestamp,acc_x,acc_y,acc_z,gy_x,gy_y,gy_z,compass\n')



# Vehicle ground truth location save file
if os.path.exists(gt_location_file_path):
    with open(gt_location_file_path,'w') as f:
        f.truncate(0)
    f.close()
fp_gt_location = open(gt_location_file_path, 'a')
fp_gt_location.write('frame,timestamp,elasped_seconds,platform_timestamp,loc_x,loc_y,loc.z\n')



# Lidar sensor transform save file
if os.path.exists(lidar_transform_file_path):
    with open(lidar_transform_file_path,'w') as f:
        f.truncate(0)
    f.close()
fp_lidar_transform = open(lidar_transform_file_path, 'a')
fp_lidar_transform.write('transformation matrix T that represents the transformation from the LiDAR coordinate system to the global world map coordinate system\n')

#############################################################sensors callback#############################################################
class GTloction:
    def __init__(self, world_timestamp, vehicle_location):
        self.frame = world_timestamp.frame
        self.elasped_seconds = world_timestamp.delta_seconds
        self.delta_seconds = world_timestamp.delta_seconds
        self.platform_timestamp = world_timestamp.platform_timestamp
        self.x = vehicle_location.x
        self.y = vehicle_location.y
        self.z = vehicle_location.z

class GNSS:
    def __init__(self, sensor_data):
        self.frame = sensor_data.frame
        self.timestamp = sensor_data.timestamp
        self.lat = sensor_data.latitude
        self.lon = sensor_data.longitude
        self.alt = sensor_data.altitude

class IMU:
    def __init__(self, sensor_data):
        self.frame = sensor_data.frame
        self.timestamp = sensor_data.timestamp
        self.acc_x = sensor_data.accelerometer.x
        self.acc_y = sensor_data.accelerometer.y
        self.acc_z = sensor_data.accelerometer.z
        self.gy_x = sensor_data.gyroscope.x
        self.gy_y = sensor_data.gyroscope.y
        self.gy_z = sensor_data.gyroscope.z
        self.compass = sensor_data.compass
    


def _parse_semantic_lidar_data(sensor_data):
    coordinates = []
    incident_angles = []
    semantic_tags = []
    distances_2d = []
    for detection in sensor_data:
        x = detection.point.x
        y = detection.point.y
        z = detection.point.z
        coordinates.append([x, y, z])
        incident_angles.append(detection.cos_inc_angle)
        semantic_tags.append(detection.object_tag)
        
        # Calculate the 2D distance between the detected object and the sensor
        distance_2d = np.sqrt(x**2 + y**2)
        distances_2d.append(distance_2d)

    coordinates = np.array(coordinates)
    incident_angles = np.array(incident_angles)
    semantic_tags = np.array(semantic_tags)
    distances_2d = np.array(distances_2d)

    return coordinates, incident_angles, semantic_tags, distances_2d

def sensor_callback(sensor, sensor_data, sensor_queue, sensor_name):
    '''
    Function: sensor's callback() to retrieve sensor data
    '''
    if 'lidar' in sensor_name:
        
        # Get the transform matrix from lidar sensor coordinate system to global world map coordinate system
        lidar_transform = sensor.get_transform()
        lidar_matrix = np.array(lidar_transform.get_matrix())

        # Parse the semantic lidar data to get coordinates, cosine angles, semantic tags, and 2D distances
        coords, incident_angles, semantic_tags, distances_2d = _parse_semantic_lidar_data(sensor_data)

        # Create an open3d point cloud object from the numpy array
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(coords)
      
        # Assign colors based on the semantic tag, intensity, or any other criteria
        # Example: using grayscale based on incident angle for color here
        # Normalize incident_angles to range [0, 1] for valid color values
        normalized_incident_angles = (incident_angles - np.min(incident_angles)) / (np.max(incident_angles) -np.min(incident_angles))
        colors = np.tile(normalized_incident_angles[:, np.newaxis], [1, 3])
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # Transform lidar point clouds from lidar sensor coordinate to global world map coordinate
        pcd.transform(lidar_matrix)
    
        # Save the .ply file
        ply_output_path = os.path.join(output_path, 'lidar', f'{sensor_data.frame}_semantic.ply')
        o3d.io.write_point_cloud(ply_output_path, pcd)
    
        # Get the transformed coordinates in the global world map coordinate system
        transformed_coords = np.asarray(pcd.points)
        
        # Calculate distances in the global world map coordinate system
        distances_global = np.sqrt(transformed_coords[:, 0]**2 + transformed_coords[:, 1]**2 + transformed_coords[:, 2]**2)
        
        # Save additional information (coordinates, semantic tags, and distances) in the global world map coordinate system
        additional_info = np.hstack((transformed_coords, semantic_tags.reshape(-1, 1), distances_global.reshape(-1, 1)))
        info_output_path = os.path.join(output_path, 'lidar', f'{sensor_data.frame}_semantic_info.txt')
        np.savetxt(info_output_path, additional_info, header='x, y, z, semantic_tag, distance', comments='', fmt=['%3f', '%3f', '%3f', '%d', '%3f'])

    if 'camera' in sensor_name:
        sensor_data.save_to_disk(os.path.join(output_path,'image', '%d.png' % sensor_data.frame))

    if 'imu' in sensor_name:
        fp_imu.write(str(sensor_data)+'\n')
        imu_data = IMU(sensor_data)
        fp_imu.write("{},{},{},{},{},{},{},{}\n".format(imu_data.frame,imu_data.timestamp,
            imu_data.acc_x,imu_data.acc_y,imu_data.acc_z,
            imu_data.gy_x, imu_data.gy_y, imu_data.gy_z))

    if 'gnss' in sensor_name:
        fp_gnss.write(str(sensor_data)+'\n')
        gnss_data = GNSS(sensor_data)
        fp_gnss.write("{},{},{},{},{}\n".format(gnss_data.frame, gnss_data.timestamp,
            gnss_data.lat, gnss_data.lon, gnss_data.alt))

    sensor_queue.put((sensor_data, sensor_name))



def get_vehicle_location(vehicle):
    '''
    Function: Obtain the ground truth of the vehicle in carla simulator
    '''
    return vehicle.get_location()


def main():
    actor_list = []
    sensor_list = []

    try:

        client = carla.Client('localhost', 2000)
        client.set_timeout(20.0)
        
        #world = client.get_world()
        world = client.load_world('Town05')
        blueprint_library = world.get_blueprint_library()
        # # Set weather for your world
        # weather = carla.WeatherParameters(cloudiness=10.0,
        #                                   precipitation=10.0,
        #                                   fog_density=10.0)
        # world.set_weather(weather)

        # set synchorinized mode
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.fixed_delta_seconds = FIXED_TIME_STEP
        settings.synchronous_mode = True
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        # create sensor queue
        sensor_queue = Queue()


        ego_vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
        ego_vehicle_bp.set_attribute('role_name','xwj_vehicle')
        # black color
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')

        # get a random valid occupation in the world
        #transform = random.choice(world.get_map().get_spawn_points())
        #transform = world.get_map().get_spawn_points()[2]
        #transform = carla.Transform(carla.Location(x=29.30,y=82.80, z=0.3),carla.Rotation(pitch=0.000000, yaw=-0.139465, roll=0.000000))
        
        
        # #VehicleSpawnPoint222
        # transform = carla.Transform(carla.Location(x=-19.7,y=-0.88, z=0.3),carla.Rotation(pitch=0.000000, yaw=-0.139465, roll=0.000000))


        #VehicleSpawnPoint182
        transform = carla.Transform(carla.Location(x=-36.14,y=2.63,z=0.3), carla.Rotation(pitch=0, yaw=0, roll=0))


        # #VehicleSpawnPoint187
        # transform = carla.Transform(carla.Location(x=-90,y=60.26,z=0.3), carla.Rotation(pitch=0, yaw=0, roll=90))

        
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)
        #ego_vehicle.set_autopilot(True)

        # collect all actors to destroy when we quit the script
        actor_list.append(ego_vehicle)

      
        # add a camera
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('sensor_tick',str(FIXED_TIME_STEP))
        camera_bp.set_attribute('image_size_x','800')
        camera_bp.set_attribute('image_size_x','600')
        camera_bp.set_attribute('fov','90.0')
        camera_transform = carla.Transform(carla.Location(x=2.0, z=2.8))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
        camera.listen(lambda data: sensor_callback(camera, data, sensor_queue, "camera"))
        sensor_list.append(camera)


        # add a lidar
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')
        lidar_bp.set_attribute('rotation_frequency', str(LIDAR_ROTATION_FREQUENCY))
        lidar_bp.set_attribute('sensor_tick',str(LIDAR_SENSOR_TICK))
        lidar_bp.set_attribute('range','20')
        lidar_bp.set_attribute('channels','128')
        lidar_bp.set_attribute('points_per_second','80000000')
        lidar_bp.set_attribute('upper_fov','45')
        lidar_bp.set_attribute('lower_fov','-26.8')
        lidar_bp.set_attribute('horizontal_fov','360')
        
        lidar_location = carla.Location(0, 0, 2.4)
        lidar_rotation = carla.Rotation(0, 0, 0)
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
        lidar.listen(lambda data: sensor_callback(lidar, data, sensor_queue, "lidar"))
        sensor_list.append(lidar)


        # add a imu
        imu_bp = blueprint_library.find("sensor.other.imu")
        imu_bp.set_attribute('sensor_tick',str(IMU_SENSOR_TICK))
        imu_location = carla.Location(2,0,2)
        imu_rotation = carla.Rotation(0,0,0)
        imu_transform = carla.Transform(imu_location, imu_rotation)
        imu = world.spawn_actor(imu_bp, imu_transform, attach_to=ego_vehicle)
        imu.listen(lambda data: sensor_callback(imu, data, sensor_queue, "imu"))
        sensor_list.append(imu)


        # add a gnss
        gnss_bp = blueprint_library.find("sensor.other.gnss")
        gnss_bp.set_attribute('sensor_tick',str(GNSS_SENSOR_TICK))
        gnss_location = carla.Location(1,0,2)
        gnss_transform = carla.Transform(gnss_location)
        gnss = world.spawn_actor(gnss_bp, gnss_transform, attach_to=ego_vehicle)
        gnss.listen(lambda data: sensor_callback(gnss, data, sensor_queue, "gnss"))
        sensor_list.append(gnss)



        # Create a Pygame window
        pygame.init()
        screen = pygame.display.set_mode((800,600))
        pygame.display.set_caption("Ego Vehicle")
        #pygame.display.set_mode((1,1), pygame.NOFRAME)

        pygame.key.set_repeat(1,50)

        control = carla.VehicleControl()


        while True:
            world.tick()
            # set the spectator to follow the ego vehicle
            spectator = world.get_spectator()
            transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=30),carla.Rotation(pitch=-90)))
            #spectator.set_transform(carla.Transform(transform.location + carla.Location(x=-4.5, y=0, z=2.8),carla.Rotation(pitch=20)))

            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        control.throttle = 1.0
                    elif event.key == pygame.K_DOWN:
                        control.brake = 1.0
                    elif event.key == pygame.K_LEFT:
                        control.steer = -0.5
                    elif event.key == pygame.K_RIGHT:
                        control.steer = 0.5
                    elif event.key == pygame.K_SPACE:
                        control.brake = 1.0
                    else:
                        control.throttle = 1.0
                        control.brake = 0
                        control.steer = 0
                    ego_vehicle.apply_control(control)

                elif event.type == pygame.KEYUP:
                    if event.key in (pygame.K_RIGHT, pygame.K_LEFT):
                        control.steer = 0.0
                    elif event.key in (pygame.K_UP, pygame.K_DOWN,pygame.K_SPACE):
                        control.throttle = 0
                        control.brake = 0
                    ego_vehicle.apply_control(control)
                        
                    
            
            world_snapshot = world.get_snapshot()
            world_frame = world_snapshot.frame
            world_timestamp = world_snapshot.timestamp

            # Obtain the ground truth vehicle position
            gt_location = get_vehicle_location(ego_vehicle)
            #fp_gt_location.write('frame:'+str(world_frame)+',' + 'timestamp:'+ str(world_timestamp)+',' + str(gt_location)+'\n')
            gt_data = GTloction(world_timestamp,gt_location)
            fp_gt_location.write("{},{},{},{},{},{},{}\n".format(gt_data.frame, gt_data.elasped_seconds, gt_data.delta_seconds, gt_data.platform_timestamp,
                gt_data.x, gt_data.y, gt_data.z))


            # As the queue is blocking, we will wait in the queue.get() methods
            # until all the information is processed and we continue with the next frame.
            try:
                for i in range(0, len(sensor_list)):
                    sensor_data = sensor_queue.get(True, 1.0)
                    print("   Frame: %d   Sensor_data: %s" % (world_frame, sensor_data))
                    #print("sensor_data:",s_frame)

            except Empty:
                print("   Some of the sensor information is missed")

    finally:
        world.apply_settings(original_settings)
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        for sensor in sensor_list:
            sensor.destroy()
        fp_gnss.close()
        fp_imu.close()
        print('done.')


if __name__ == '__main__':
    main()
    # try:
    #     main()
    # except KeyboardInterrupt:
    #     print(' - Exited by user.')
