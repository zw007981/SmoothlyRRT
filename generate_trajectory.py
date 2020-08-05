from B_spline import *
from configs import *
from rrt_algorithm import *


# 生成一条从start到goal的路径并避开obstacles中的障碍物。
def generate_trajectory(start, goal, obstacles):
    # 样条曲线曲线必正切于第一条与最后一条控制线段，利用此性质将
    # 起始点和终止点向各自延申一定长度，使得生成的样条曲线满足在两端的切线边界条件。
    [theta0, length0] = [THETA0, LENGTH0]
    [theta1, length1] = [THETA1, LENGTH1]
    # 用于生成随机树的临时起点和终点。
    start1 = (start[0]+length0*math.cos(theta0),
              start[1]+length0*math.sin(theta0))
    goal1 = (goal[0]+length1*math.cos(theta1),
             goal[1]+length1*math.sin(theta1))
    tree = rrt(start1, goal1, obstacles)
    path = tree.find_path(start1, goal1)
    path_modified = tree.modify_path(path, obstacles)
    # 添加真正的起点和终点生成一系列控制点。
    start0 = (start[0]+0.5*length0*math.cos(theta0),
              start[1]+0.5*length0*math.sin(theta0))
    goal0 = (goal[0]+0.5*length1*math.cos(theta1),
             goal[1]+0.5*length1*math.sin(theta1))
    path_modified.insert(0, start0)
    path_modified.append(goal0)
    path_modified.insert(0, start)
    path_modified.append(goal)
    # 利用控制点生成B样条曲线作为运动轨迹。
    trajectory = generate_B_spline(path_modified)
    return [trajectory, tree, path, path_modified]
