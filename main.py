import pygame as pg
from pygame.locals import *

from utils.B_spline import *
from utils.configs import *
from utils.generate_trajectory import *
from utils.rrt_algorithm import *

[START, GOAL] = [START_INIT, GOAL_INIT]


def add_obstacle(position, obstacles):
    obstacles.append(position)


def update_screen(tree, obstacles):
    pg.draw.circle(SCREEN, RED, [int(START[0]), int(START[1])], 10)
    pg.draw.circle(SCREEN, RED, [int(GOAL[0]), int(GOAL[1])], 10)
    if len(obstacles) > 0:
        for obstacle in obstacles:
            pg.draw.circle(SCREEN, BLACK, obstacle, OBSTACLES_RADIUS)
    for point in tree.points:
        pg.draw.circle(SCREEN, GREY, [int(point[0]), int(point[1])], 2)
        if point in tree.parent:
            if tree.parent[point] != None:
                pg.draw.aaline(
                    SCREEN, GREY, [int(point[0]), int(point[1])], tree.parent[point])


if __name__ == "__main__":
    pg.init()
    SCREEN = pg.display.set_mode((WIDTH, HEIGHT))
    clock = pg.time.Clock()
    flag_running = True
    flag_start = False
    flag_add_obstacles = False
    obstacles = []
    obstacles_ = []
    [trajectory, tree, path, path_modified] = generate_trajectory(
        START, GOAL, obstacles)
    while flag_running:
        SCREEN.fill(WHITE)
        pg.display.set_caption('S-RRT')
        for event in pg.event.get():
            # 持续捕捉鼠标轨迹。
            if event.type == MOUSEMOTION:
                mouse_position = pg.mouse.get_pos()
            # 按esc退出模拟。
            if event.type == pg.KEYDOWN and event.key == K_ESCAPE:
                flag_running = False
            # 单击鼠标左键添加障碍物。
            elif event.type == MOUSEBUTTONDOWN and event.button == 1:
                flag_add_obstacles = not flag_add_obstacles
            # 单击鼠标中键清空障碍物数据。
            elif event.type == MOUSEBUTTONDOWN and event.button == 2:
                obstacles = []
            # 单击鼠标右键开始模拟。
            elif event.type == MOUSEBUTTONDOWN and event.button == 3:
                flag_start = True
            if flag_add_obstacles:
                obstacles.append(mouse_position)
                flag_start = False
        # 开始模拟。
        if flag_start:
            obstacles_ = obstacles
            # B样条曲线生成的路径。
            [trajectory, tree, path, path_modified] = generate_trajectory(
                START, GOAL, obstacles)
            flag_start = False
        update_screen(tree, obstacles)
        # 绘制初始路径。
        if path != None:
            for i in range(len(path)-1):
                pg.draw.aaline(SCREEN, GREEN, path[i], path[i+1])
        # 绘制优化后的路径，同时这也是控制点。
        if path_modified != None:
            for i in range(len(path_modified)-1):
                pg.draw.aaline(
                    SCREEN, RED, path_modified[i], path_modified[i+1])
            for i in range(len(path_modified)):
                pg.draw.circle(
                    SCREEN, RED, [int(path_modified[i][0]), int(path_modified[i][1])], 4)
        # 绘制采用B样条曲线拟合后的路径。
        for i in range(len(trajectory)-1):
            pg.draw.aaline(SCREEN, BLUE, trajectory[i], trajectory[i+1])
        pg.display.flip()
        clock.tick(144)
