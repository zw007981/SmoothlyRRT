from utils.B_spline import *
from utils.configs import *
from utils.rrt_algorithm import *


# 生成一条从start到goal的路径并避开obstacles中的障碍物。
def generate_trajectory(start, goal, obstacles):
    tree = rrt(start, goal, obstacles)
    path = tree.find_path(start, goal)
    path_modified = tree.modify_path(path, obstacles)
    # 利用控制点生成B样条曲线作为运动轨迹。
    # 因为我们采用的是三阶B样条曲线，要求控制点的数量至少为3，
    # 所以在只有起点和终点时添加它们的中点作为补充点。
    if len(path_modified) < 3:
        point_added = (0.5 * (start[0] + goal[0]), 0.5 * (start[1] + goal[1]))
        path_modified.append(point_added)
    trajectory = generate_B_spline(path_modified)
    return [trajectory, tree, path, path_modified]
