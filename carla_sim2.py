# Code based on Carla examples, which are authored by 
# Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).

# How to run: 
# cd into the parent directory of the 'code' directory and run
# python -m code.tests.control.carla_sim
import carla
import random
from pathlib import Path
import numpy as np
import pygame
from util.carla_util import carla_vec_to_np_array, carla_img_to_array, CarlaSyncMode, find_weather_presets, draw_image_np, should_quit
from util.geometry_util import dist_point_linestring
from control.MPC import MPCPlusPID
import argparse
import cv2
import copy
import csv


main_image_shape = (1280, 720)
FPS = 30
plan_fre = 3    # 实际上这里的频率指的是，实际程序运行多少次，mpc规划控制运行一次
mpc_sample_time = 1./FPS * plan_fre


def get_trajectory_from_lane_detector(ld, image):
    # get lane boundaries using the lane detector
    img = carla_img_to_array(image)
    poly_left, poly_right, left_mask, right_mask = ld.get_fit_and_probs(img)
    # trajectory to follow is the mean of left and right lane boundary
    # note that we multiply with -0.5 instead of 0.5 in the formula for y below
    # according to our lane detector x is forward and y is left, but
    # according to Carla x is forward and y is right.
    x = np.arange(-2,60,1.0)
    y = -0.5*(poly_left(x)+poly_right(x))
    # x,y is now in coordinates centered at camera, but camera is 0.5 in front of vehicle center
    # hence correct x coordinates
    x += 0.5
    traj = np.stack((x,y)).T
    return traj, ld_detection_overlay(img, left_mask, right_mask)

def ld_detection_overlay(image, left_mask, right_mask):
    res = copy.copy(image)
    res[left_mask > 0.5, :] = [0,0,255]
    res[right_mask > 0.5, :] = [255,0,0]
    return res


def get_trajectory_from_map(m, vehicle,transform2vehicle = 1,contain_yaw = 0):
    '''
    get 80 waypoints each 1m apart. If multiple successors choose the one with lower waypoint.id

    Parameters
    ----------
    m : carla.Map
        map from carla
    vehicle : carla.vehicle
        vechile from carla, trajectory will be planned base on its location
    transform2vehicle : bool,optional
        if true, will return world coordinates, else object coordinates
    contain_yaw : bool,optional
        if true, trajectory will contain yaw as last column
    
    Returns
    -------
    traj : ndarray
        trajectory, if contain_yaw is true ,it will be n*3 [x, y, yaw].
    '''

    wp = m.get_waypoint(vehicle.get_transform().location)
    wps = [wp]
    
    for _ in range(20):
        next_wps = wp.next(1.0)
        if len(next_wps) > 0:
            wp = sorted(next_wps, key=lambda x: x.id)[0]
        wps.append(wp)

    if( transform2vehicle ):
        # transform waypoints to vehicle ref frame
        traj = np.array(
            [np.array([*carla_vec_to_np_array(x.transform.location), 1.]) for x in wps]
        ).T
        # 
        trafo_matrix_world_to_vehicle = np.array(vehicle.get_transform().get_inverse_matrix())

        traj = trafo_matrix_world_to_vehicle @ traj
        traj = traj.T
        traj = traj[:,:2]
    else:
        traj = np.array( [np.array([*carla_vec_to_np_array(x.transform.location)]) for x in wps] )
        traj = traj[:,:2]
    
    if( contain_yaw ):
        yaw = np.array( [x.transform.rotation.yaw for x in wps] )
        traj = np.concatenate( (traj,yaw.reshape(-1,1)),axis=1 )
    
    return traj

def send_control(vehicle, throttle, steer, brake,
                 hand_brake=False, reverse=False):
    throttle = np.clip(throttle, 0.0, 1.0)
    steer = np.clip(steer, -1.0, 1.0)
    brake = np.clip(brake, 0.0, 1.0)
    control = carla.VehicleControl(throttle, steer, brake, hand_brake, reverse)
    vehicle.apply_control(control)

def display_info(display,texts,font,throttle,steer,dy=18):
    info_surface = pygame.Surface((220, main_image_shape[1]))
    info_surface.set_alpha(100)
    display.blit(info_surface, (0, 0))    
    
    for it,t in enumerate(texts):
        display.blit(
            font.render(t, True, (255,255,255)), (5, 20+dy*it))
    
    v_offset =  20+dy*it + dy
    display.blit( font.render("Throttle:", True, (255,255,255)), (5, v_offset))

    
    throttle_rate = np.clip(throttle, 0.0, 1.0)
    # throttle_rate = vehicle.get_control().throttle
    bar_width = 106
    bar_h_offset = 100
    rect_border = pygame.Rect((bar_h_offset, v_offset+6), (bar_width, 6))
    pygame.draw.rect(display, (255, 255, 255), rect_border, 1)

    rect = pygame.Rect((bar_h_offset, v_offset+6), (throttle_rate * bar_width, 6))
    pygame.draw.rect(display, (255, 255, 255), rect)

    v_offset = v_offset + dy

    display.blit( font.render("Steer:", True, (255,255,255)), (5, v_offset))
    steer_rate = np.clip(steer, -1.0, 1.0)
    # steer_rate = vehicle.get_control().steer
    bar_width = 106
    bar_h_offset = 100
    rect_border = pygame.Rect((bar_h_offset, v_offset+6), (bar_width, 6))
    pygame.draw.rect(display, (255, 255, 255), rect_border, 1)

    rect = pygame.Rect(((bar_h_offset+(steer_rate+1)/2*bar_width), v_offset+6), (6, 6))
    pygame.draw.rect(display, (255, 255, 255), rect)



    pygame.display.flip() # 将绘制的图像显示在屏幕上   


def main(use_lane_detector=False, ex=False, save_video=False, half_image=False):
    if save_video:
        import atexit
        import imageio
        #import time
        images = []
        from tqdm import tqdm
        video_writer = imageio.get_writer('my_video.mp4', format='FFMPEG', mode='I', fps=30)
        
        def write_images_to_video(images, video_writer):
            print("Writing images to video file...")
            for img in tqdm(images): 
                video_writer.append_data(img)
            video_writer.close()
        atexit.register(lambda: write_images_to_video(images, video_writer))

    actor_list = []
    pygame.init()
    
    # window
    display = pygame.display.set_mode(
        main_image_shape,
        pygame.HWSURFACE | pygame.DOUBLEBUF)    # 创建游戏窗口
    font = pygame.font.SysFont('ubuntumono', 14) # 设置字体
    clock = pygame.time.Clock()                 # 创建时钟，时钟对象可以用来控制游戏的帧率

    client = carla.Client('localhost', 2000)
    client.set_timeout(80.0)

    #client.load_world('Town06')
    client.load_world('Town04')
    world = client.get_world()
    # world.unload_map_layer(carla.MapLayer.Buildings)

    weather_preset, _ = find_weather_presets()[1]
    world.set_weather(weather_preset)

    controller = MPCPlusPID()

    try:
        m = world.get_map()

        blueprint_library = world.get_blueprint_library()

        veh_bp = random.choice(blueprint_library.filter('vehicle.audi.tt'))  # 选择车辆
        veh_bp.set_attribute('color','64,81,181')                            # 上色
        vehicle = world.spawn_actor(                                         # 生成车辆
            veh_bp,
            m.get_spawn_points()[90])                                        # 车辆位置选择为生成点[90]
        actor_list.append(vehicle)

        # 相机输出的画面大小
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x',str(main_image_shape[0]))
        camera_bp.set_attribute('image_size_y',str(main_image_shape[1]))

        # visualization cam (no functionality)
        camera_rgb = world.spawn_actor(
                                    camera_bp,
                                    carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-10)),
                                    attach_to=vehicle)
        
        actor_list.append(camera_rgb)
        sensors = [camera_rgb]

        frame = 0
        max_error = 0
        
        plan_count = 0
        # Create a synchronous mode context.
        with CarlaSyncMode(world, *sensors, fps=FPS) as sync_mode:
            while True:
                spectator = world.get_spectator()
                transform = vehicle.get_transform()
                spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
                                                                    carla.Rotation(pitch=-90)))
                if should_quit():
                    return
                clock.tick()          
                
                # Advance the simulation and wait for the data. 
                tick_response = sync_mode.tick(timeout=2.0)

                snapshot, image_rgb = tick_response
                traj = get_trajectory_from_map(m, vehicle,transform2vehicle=0,contain_yaw=1)
                traj_object = get_trajectory_from_map(m, vehicle,transform2vehicle=1,contain_yaw=1)

                for index,point in enumerate(traj):
                    if index % 5 ==0:
                        world.debug.draw_point(carla.Location( point[0], point[1], 1),life_time = 0.5)

                # get velocity and angular velocity
                # vel = carla_vec_to_np_array(vehicle.get_velocity())
                # forward = carla_vec_to_np_array(vehicle.get_transform().get_forward_vector())
                # right = carla_vec_to_np_array(vehicle.get_transform().get_right_vector())
                # up = carla_vec_to_np_array(vehicle.get_transform().get_up_vector())
                # vx = vel.dot(forward)
                # vy = vel.dot(right)
                # vz = vel.dot(up)
                # ang_vel = carla_vec_to_np_array(vehicle.get_angular_velocity())
                # w = ang_vel.dot(up)
                # print("vx vy vz w {:.2f} {:.2f} {:.2f} {:.5f}".format(vx,vy,vz,w))

                speed = np.linalg.norm( carla_vec_to_np_array(vehicle.get_velocity()))
                
                state = [vehicle.get_transform().location.x, 
                         vehicle.get_transform().location.y, 
                         vehicle.get_transform().rotation.yaw,
                         speed,
                         0.5*(vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)+vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel))]
                
                if plan_count%plan_fre == 0:
                    throttle, steer = controller.get_control(traj,traj_object, speed, desired_speed=15, dt=mpc_sample_time,state = state)
                
                plan_count = plan_count + 1
                send_control(vehicle, throttle, steer, 0)
                
                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                dist = dist_point_linestring(np.array([0,0]), traj_object[:,:2])

                cross_track_error = int(dist*100)
                max_error = max(max_error, cross_track_error)

                # Draw the display.
                image_rgb = copy.copy(carla_img_to_array(image_rgb))
                draw_image_np(display, image_rgb) # 在Pygame显示中绘制图像

                # draw txt
                dy = 18
                texts = ["FPS (real):                % 3.0f "%int(clock.get_fps()),
                         "FPS (simulated):           % 3.0f "%fps,
                         "speed (m/s):               % 3.0f" %speed,
                         "lateral error (cm):        % 3.0f" %cross_track_error,
                         "max lat. error (cm):       % 3.0f" %max_error,
                         'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (state[0], state[1])),
                        ]
                display_info(display,texts,font,throttle,steer,dy=dy)

                frame += 1
                if save_video and frame > 0:
                    print("frame=",frame)
                    imgdata = pygame.surfarray.array3d(pygame.display.get_surface())
                    imgdata = imgdata.swapaxes(0,1)
                    images.append(imgdata)
                

    finally:

        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()
        with open('data.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(controller.MPC.record)
        pygame.quit()
        print('done.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs Carla simulation with your control algorithm.')
    parser.add_argument("--vid", action="store_true", help="Save video after simulation")
    parser.add_argument("--half_image", action="store_true", help="Pass images with (width, height) = (512,256) to lane detector instead of the default (1024,512). This will speed up the simulation, but might hurt accuracy.")
    args = parser.parse_args()

    try:
        main(save_video=args.vid, half_image=args.half_image)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
