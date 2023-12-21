from agents.navigation.behavior_agent import BehaviorAgent
from util.carla_util import CarlaSyncMode, should_quit,draw_image_np,carla_img_to_array
import random
import carla
import pygame
import copy
import numpy as np
from pygame.locals import K_p

color_bar = [(0,   255,  0 , 0),   # Green  LEFT
             (255, 0,    0 , 0),   # Red    RIGHT
             (0,   0,   0  , 0),   # Black  STRAIGHT
             (255, 255, 255, 0),   # White  LANEFOLLOW
             (0,   0,   255, 0),   # Blue   CHANGELANELEFT
             (255, 255,  0 , 0),   # Yellow CHANGELANERIGHT
             ]
FPS = 30
main_image_shape = (1280, 720)
driver_real_view = [[0.1, -0.25 ,1.25],[0,0,0]]    # 正常开车视角
driver_sim_view1 = [[1.25, 0,    1.25],[0,0,0]]    # 模拟视角，前方视角
driver_sim_view2 = [[-10, 0,     2.8], [-10,0,0]]  # 模拟视角，后看视角

driver_view = driver_sim_view2
autopilot_on = False


def autopilot(vehicle,destination):
    # 输入对应车辆，目的地，返回路径点，控制等
    agent = BehaviorAgent(vehicle)
    route = agent.set_destination(destination)
    
    return route ,agent

def draw_waypoint(route,world):
    for point in route:
        world.debug.draw_point(point[0].transform.location,
                                color = carla.Color(*color_bar[point[1]-1]),
                                life_time = 0)
    world.debug.draw_box(carla.BoundingBox( point[0].transform.location, carla.Vector3D(5,5,5)),
                        point[0].transform.rotation,
                        thickness=0.5)  
    
    x = point[0].transform.location.x
    y = point[0].transform.location.y
    z = point[0].transform.location.z
    
    world.debug.draw_arrow(carla.Location(x,y,z+10),
                           carla.Location(x,y,z+5),
                           arrow_size=0.5,)  

def display_info(display,texts,font,vehicle,speed,font_speed,dy=18):
    info_surface = pygame.Surface((220, main_image_shape[1]/4))
    info_surface.set_alpha(100)
    display.blit(info_surface, (0, 0))    
    
    for it,t in enumerate(texts):
        display.blit(
            font.render(t, True, (255,255,255)), (5, 20+dy*it))
    
    v_offset =  20+dy*it + dy
    display.blit( font.render("Throttle:", True, (255,255,255)), (5, v_offset))

    
    # throttle_rate = np.clip(throttle, 0.0, 1.0)
    throttle_rate = vehicle.get_control().throttle
    bar_width = 106
    bar_h_offset = 100
    rect_border = pygame.Rect((bar_h_offset, v_offset+6), (bar_width, 6))
    pygame.draw.rect(display, (255, 255, 255), rect_border, 1)

    rect = pygame.Rect((bar_h_offset, v_offset+6), (throttle_rate * bar_width, 6))
    pygame.draw.rect(display, (255, 255, 255), rect)

    v_offset = v_offset + dy

    display.blit( font.render("Steer:", True, (255,255,255)), (5, v_offset))
    # steer_rate = np.clip(steer, -1.0, 1.0)
    steer_rate = vehicle.get_control().steer
    bar_width = 106
    bar_h_offset = 100
    rect_border = pygame.Rect((bar_h_offset, v_offset+6), (bar_width, 6))
    pygame.draw.rect(display, (255, 255, 255), rect_border, 1)

    rect = pygame.Rect(((bar_h_offset+(steer_rate+1)/2*bar_width), v_offset+6), (6, 6))
    pygame.draw.rect(display, (255, 255, 255), rect)

    speed_str = "%4.1fkm/h" %(speed*3.6)
    display.blit( font_speed.render(speed_str, True, (255,255,255)), (main_image_shape[0]-400,main_image_shape[1]-90) )
    if autopilot_on:
        autopilot_on_str = "Auto"
        display.blit( font_speed.render(autopilot_on_str, True, (255,0,0)), (10,main_image_shape[1]-90 ))

    pygame.display.flip() # 将绘制的图像显示在屏幕上  

def carla_vec_to_np_array(vec):
    return np.array([vec.x,
                     vec.y,
                     vec.z])

def demo_toggle_autopilot():
    global autopilot_on
    for event in pygame.event.get():
        if event.type == pygame.KEYUP:   #TODO 以后把DualControl对应部分给改了，用G29上的按键event.type == pygame.JOYBUTTONDOWN:
            if event.key == K_p:    
                autopilot_on = not autopilot_on
                return 1
        return 0

def send_control(vehicle, throttle, steer, brake,
                 hand_brake=False, reverse=False):
    throttle = np.clip(throttle, 0.0, 1.0)
    steer = np.clip(steer, -1.0, 1.0)
    brake = np.clip(brake, 0.0, 1.0)
    control = carla.VehicleControl(throttle, steer, brake, hand_brake, reverse)
    vehicle.apply_control(control)

def main():
    pygame.init()
    display = pygame.display.set_mode(
        main_image_shape,
        pygame.HWSURFACE | pygame.DOUBLEBUF)    # 创建游戏窗口
    font = pygame.font.SysFont('ubuntumono', 14) # 设置字体
    font_speed = pygame.font.SysFont('ubuntumono', 100) # 设置字体
    clock = pygame.time.Clock()   


    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(80.0)
    client.load_world('Town04')
    world = client.get_world()
    try:
        m = world.get_map()

        blueprint_library = world.get_blueprint_library()

        veh_bp = random.choice(blueprint_library.filter('vehicle.audi.tt'))  # 选择车辆
        veh_bp.set_attribute('color','64,81,181')                            # 上色
        vehicle = world.spawn_actor(                                         # 生成车辆
            veh_bp,
            m.get_spawn_points()[288])                                        # 车辆位置选择为生成点[90]
        actor_list.append(vehicle)

        # 相机输出的画面大小
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x',str(main_image_shape[0]))
        camera_bp.set_attribute('image_size_y',str(main_image_shape[1]))

        # visualization cam (no functionality)
        camera_rgb = world.spawn_actor(
                                    camera_bp,
                                    carla.Transform(carla.Location(*driver_view[0]), carla.Rotation(*driver_view[1])),
                                    attach_to=vehicle)
        
        actor_list.append(camera_rgb)
        sensors = [camera_rgb]
        
        # 绘制auto规划路点
        destination = m.get_spawn_points()[190].location
        route,agent = autopilot(vehicle,destination)
        draw_waypoint(route,world)

        # 放置观测者
        spectator = world.get_spectator()
        transform = vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
                                                            carla.Rotation(pitch=-90)))

        all_finish = 0

        with CarlaSyncMode(world, *sensors,fps=FPS) as sync_mode: 
            while(1):

                if should_quit():
                    return
                clock.tick() 
                tick_response = sync_mode.tick(timeout=2.0)
                
                toggle2auto = demo_toggle_autopilot()
                # 如果重新开启了自动驾驶，则重新设定目标路线，重新规划
                if toggle2auto: 
                    agent.set_destination(destination)
                
                if autopilot_on:
                    control = agent.run_step()
                    control.manual_gear_shift = False
                    vehicle.apply_control(control)
                else:
                    send_control(vehicle,0,0,0) #TODO: 此处填上手动控制逻辑
                
                
                snapshot, image_rgb = tick_response
                image_rgb = copy.copy(carla_img_to_array(image_rgb))
                draw_image_np(display, image_rgb)                   # 在Pygame显示中绘制图像
                
                speed = np.linalg.norm( carla_vec_to_np_array(vehicle.get_velocity()))
                fps = round(1.0 / snapshot.timestamp.delta_seconds)
                
                texts = ["FPS (real):                % 3.0f "%int(clock.get_fps()),
                         "FPS (simulated):           % 3.0f "%fps,
                         "speed (m/s):               % 3.0f" %speed,
                         'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (vehicle.get_transform().location.x, vehicle.get_transform().location.y)),
                         'Yaw:                       % 3.0f' %vehicle.get_transform().rotation.yaw
                        ]                
                display_info(display,texts,font,vehicle,speed,font_speed,dy=18)
                
                if agent.done():
                    if all_finish:
                        for actor in actor_list:
                            if actor is not None:
                                actor.destroy()
                        break
                                
                    if not all_finish:
                        destination = m.get_spawn_points()[288].location
                        route = agent.set_destination(destination)
                        draw_waypoint(route,world)
                        all_finish = 1
    finally:
        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()
        pygame.quit()
        print('done.')



if __name__ == '__main__':
    main()