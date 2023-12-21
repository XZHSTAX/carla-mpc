import carla
from util.carla_util import  CarlaSyncMode, find_weather_presets


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(80.0)

    client.load_world('Town04')
    world = client.get_world()
    map = world.get_map()

    spawn_points_list = map.get_spawn_points()
    i = 0
    while(1):
        for index,point in enumerate(spawn_points_list):
            # world.debug.draw_point(carla.Location( point.location.x, point.location.y, point.location.z),life_time = 0)
            text_location = carla.Location(point.location.x, point.location.y, point.location.z)
            world.debug.draw_string(text_location, str(index), draw_shadow=False, color=carla.Color(r=255, g=0, b=0),life_time=999)
        i = i+1

if __name__ == '__main__':

    main()
