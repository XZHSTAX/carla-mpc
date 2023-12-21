import pygame
from pygame.locals import *
import math

def main():
    pygame.init()

    # 设置窗口的大小
    window_width, window_height = 800, 600
    screen = pygame.display.set_mode((window_width, window_height))
    pygame.display.set_caption("速度仪表盘示例")

    clock = pygame.time.Clock()
    running = True

    # 初始化车辆时速
    speed = 90

    # 仪表盘参数
    dial_radius = 100
    dial_center = (window_width-150, window_height-150)
    dial_color = (200, 200, 200)
    pointer_length = 80
    pointer_width = 10
    pointer_color = (255, 0, 0)

    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
                else:
                    print("按下了键盘按键：%s" % pygame.key.name(event.key))

        # 窗口背景为白色
        screen.fill((255, 255, 255))

        # 绘制仪表盘背景圆形
        pygame.draw.circle(screen, dial_color, dial_center, dial_radius, 0)

        # 计算指针角度
        max_speed = 200  # 最大速度
        angle = math.pi / 3  # 弧度（指针范围）
        speed_angle = speed / max_speed * angle  # 当前速度对应的角度

        # 计算指针位置
        pointer_x = dial_center[0] + int(math.sin(speed_angle) * dial_radius)
        pointer_y = dial_center[1] - int(math.cos(speed_angle) * dial_radius)
        pointer_end_x = dial_center[0] + int(math.sin(speed_angle) * (dial_radius - pointer_length))
        pointer_end_y = dial_center[1] - int(math.cos(speed_angle) * (dial_radius - pointer_length))

        # 绘制仪表盘指示器
        pygame.draw.line(screen, pointer_color, (pointer_x, pointer_y), (pointer_end_x, pointer_end_y), pointer_width)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()