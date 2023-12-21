from agents.navigation.behavior_agent import BehaviorAgent
import random
import carla

color_bar = [(0,   255,  0 , 0),   # Green  LEFT
             (255, 0,    0 , 0),   # Red    RIGHT
             (0,   0,   0  , 0),   # Black  STRAIGHT
             (255, 255, 255, 0),   # White  LANEFOLLOW
             (0,   0,   255, 0),   # Blue   CHANGELANELEFT
             (255, 255,  0 , 0),   # Yellow CHANGELANERIGHT
             ]

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


if __name__ == '__main__':
    actor_list = []
    client = carla.Client('localhost', 2000)
    client.set_timeout(80.0)
    client.load_world('Town04')
    world = client.get_world()

    m = world.get_map()

    blueprint_library = world.get_blueprint_library()

    veh_bp = random.choice(blueprint_library.filter('vehicle.audi.tt'))  # 选择车辆
    veh_bp.set_attribute('color','64,81,181')                            # 上色
    vehicle = world.spawn_actor(                                         # 生成车辆
        veh_bp,
        m.get_spawn_points()[288])                                        # 车辆位置选择为生成点[90]
    actor_list.append(vehicle)
    
    # spawn_points = m.get_spawn_points()
    # destination = random.choice(spawn_points).location
    destination = m.get_spawn_points()[190].location
    route,agent = autopilot(vehicle,destination)
    # for index,point in enumerate(route):
    #     world.debug.draw_point(point[0].transform.location,
    #                             color = carla.Color(*color_bar[point[1]-1]),
    #                             life_time = 0)
    draw_waypoint(route,world)


    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
                                                        carla.Rotation(pitch=-90)))

    all_finish = 0
    while(1): 
        control = agent.run_step()
        control.manual_gear_shift = False
        vehicle.apply_control(control)
        if agent.done():
            if all_finish:
                for actor in actor_list:
                    if actor is not None:
                        actor.destroy()
                break
                        
            if not all_finish:
                route = agent.set_destination(m.get_spawn_points()[288].location)
                draw_waypoint(route,world)
                all_finish = 1