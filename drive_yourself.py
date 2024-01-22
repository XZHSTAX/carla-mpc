from agents.navigation.behavior_agent import BehaviorAgent
from util.carla_util import CarlaSyncMode, should_quit,draw_image_np,carla_img_to_array
import random
import carla
import pygame
import copy
import numpy as np
import math
from pygame.locals import K_p
from configparser import ConfigParser

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

driver_view = driver_sim_view1

class Control_with_G29(object):
    '''
    设计一个类，用于检测方向盘的输入，并且转换为对应的控制值，存储在成员变量中
    '''
    def __init__(self):
        self._control = carla.VehicleControl()
        self.autopilot_on = False
        self._control.steer = 0.0
        self._control.throttle = 0.0
        self._control.brake = 0.0
        self._control.hand_brake = False
        self._control.reverse = False

        self._autopilot_enabled = False
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")

        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()

        self._parser = ConfigParser()
        self._parser.read('wheel_config.ini')
        self._steer_idx     = int( self._parser.get( 'G29 Racing Wheel', 'steering_wheel' ) )
        self._throttle_idx  = int( self._parser.get( 'G29 Racing Wheel', 'throttle' ) )
        self._brake_idx     = int( self._parser.get( 'G29 Racing Wheel', 'brake' ) )
        self._reverse_idx   = int( self._parser.get( 'G29 Racing Wheel', 'reverse' ) )
        self._handbrake_idx = int( self._parser.get( 'G29 Racing Wheel', 'handbrake' ) )
    
    def parse_events(self,agent,destination):
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 23:
                    self.autopilot_on = not self.autopilot_on
                    # 如果重新开启了自动驾驶，则重新设定目标路线，重新规划
                    if self.autopilot_on:
                        agent.set_destination(destination)
                
                elif event.button == 4:
                    self._control.gear = 1 if self._control.reverse else -1

        if not self._autopilot_enabled:
            self._parse_vehicle_wheel()
            self._control.reverse = self._control.gear < 0
    
    def _parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._joystick.get_numbuttons())]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 1.0  # 0.55
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

        K2 = 1.6  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1

        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd

        #toggle = jsButtons[self._reverse_idx]

        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

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

def display_info(display,texts,font,vehicle,speed,font_speed,autopilot_on,dy=18):
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
    controller = Control_with_G29()
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
        route,agent = autopilot(vehicle,destination) # 注意，这里我修改了carla的源码，才使得autopilot能返回路点
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
                
                controller.parse_events(agent,destination)
                
                if controller.autopilot_on:
                    control = agent.run_step()
                    control.manual_gear_shift = False
                    vehicle.apply_control(control)
                else:
                    send_control(vehicle,controller._control.throttle,
                                        controller._control.steer,
                                        controller._control.brake,
                                        controller._control.hand_brake,
                                        controller._control.reverse) #TODO: 此处填上手动控制逻辑
                
                
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
                display_info(display,texts,font,vehicle,speed,font_speed,controller.autopilot_on,dy=18)
                
                if agent.done():
                    # 如果完成了一圈
                    if all_finish:
                        for actor in actor_list:
                            if actor is not None:
                                actor.destroy()
                        break
                    # 如果才跑完半圈            
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