import carla
from util.carla_util import  CarlaSyncMode, find_weather_presets


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(80.0)

    client.load_world('Town04')
    world = client.get_world()

    # weather_preset, _ = find_weather_presets()[1]
    # world.set_weather(weather_preset)


if __name__ == '__main__':

    main()
