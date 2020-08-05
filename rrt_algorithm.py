import math
import random

from configs import *


# 计算point1[x1, y1]到point2[x2, y2]的欧式距离。
def cal_dist(point1, point2):
    dist = math.hypot(point1[0] - point2[0], point1[1] - point2[1])
    return dist


# theta为∠point0-point1-point2。考虑到车辆的运动学模型，这个值不宜太小，若太小则转弯太急。
def cal_theta(point0, point1, point2):
    theta1 = math.atan2(point1[1] - point0[1], point1[0] - point0[0])
    theta2 = math.atan2(point2[1] - point1[1], point2[0] - point1[0])
    if theta1 * theta2 >= 0:
        theta = abs(theta1-theta2)
    else:
        theta = abs(theta1) + abs(theta2)
        if theta > math.pi:
            theta = 2 * math.pi - theta
    theta = math.pi - theta
    return theta


# 检测从point1到point2的直线是否与障碍物相撞。
def check_collision(point1, point2, obstacles):
    # 把从point1到point2的直线分成num段。
    num = max(int(abs(point1[0] - point2[0])),
              int(abs(point1[1] - point2[1])), 2)
    [delta_x, delta_y] = [(point2[0] - point1[0])/num,
                          (point2[1] - point1[1])/num]
    # 检测从point1到point2这条直线上的各个点是否在障碍物之内。
    is_collision = False
    for i in range(num+1):
        # 希望生成的点离障碍物较远。
        point = [int(point1[0]+i*delta_x), int(point1[1]+i*delta_y)]
        for obstacle in obstacles:
            if cal_dist(point, obstacle) <= 2.0 * OBSTACLES_RADIUS:
                is_collision = True
                break
        if is_collision:
            break
    return is_collision


# 用于存储生成的随机树。
class ClassTree:
    # 树上的节点以及各个节点的父节点。
    def __init__(self):
        self.points = []
        self.parent = {}

    # 随机生成一个点，有p的概率横坐标在[0,x]之间，纵坐标在[0,y]之间。
    # 有1-p的概率为目标点。
    def generate_random_point(self, goal, x=WIDTH, y=HEIGHT):
        p = 0.95
        if random.random() < p:
            point = (random.randrange(x), random.randrange(y))
        else:
            point = goal
        return point

    # 为树添加节点，如果在这之前树上没有节点，则将之设置为树的第一个节点。
    def add_point(self, point):
        self.points.append(point)
        if len(self.points) == 1:
            self.parent[point] = None

    # 为树外一点找到树上的一个合适的点作为父节点。
    def find_proper_point(self, point):
        [w1, w2] = [0.5, 0.5]
        num = len(self.points)
        [list_val, list_dist, list_theta] = [
            list(range(num)), list(range(num)), list(range(num))]
        for i in range(num):
            point_on_tree = self.points[i]
            dist = cal_dist(point_on_tree, point)
            theta = cal_theta(point_on_tree, point, self)
            list_dist[i] = dist
            list_theta[i] = theta
        # 找到最大的距离和最大的角度，方便进行归一化处理。
        dist_max = max(list_dist)
        theta_max = max(list_theta)
        [dist_max, theta_max] = [max(dist_max, 1e-3), max(theta_max, 1e-3)]
        # 综合考虑距离和转弯角度选择合适的点作为父节点。
        # 合适的父节点应该同时具有欧式距离较短和转弯角度较大两个特点。
        for i in range(num):
            val = w1*((dist_max-list_dist[i])/dist_max) - \
                w2*((theta_max-list_theta[i])/theta_max)
            list_val[i] = val
        index = list_val.index(max(list_val))
        proper_point = self.points[index]
        return proper_point

    # 找到树上离point最近的点。
    def find_nearest_point(self, point):
        [nearest_point, nearest_dist] = [self.points[0], math.inf]
        for point_candidate in self.points:
            dist = cal_dist(point, point_candidate)
            if dist < nearest_dist:
                [nearest_point, nearest_dist] = [point_candidate, dist]
        return nearest_point

    # 找到一条从start到goal的路径。
    def find_path(self, start, goal):
        path = [goal]
        # 直到找到了start。
        while path[-1] != start:
            if path[-1] in self.parent:
                path.append(self.parent[path[-1]])
            # 在追溯的时候若发现一个点没有父节点则返回None。
            else:
                path = [goal]
                break
        if len(path) == 1:
            return None
        else:
            path.reverse()
            return path

    # 调整路径，剔除不必要的节点，修正过小的转角。
    def modify_path(self, path, obstacles):
        start_index = 0
        goal_index = 0
        # path_modified0为剔除了不必要节点后的路径。
        path_modified0 = [path[start_index]]
        while goal_index != len(path)-1:
            goal_index += 1
            if not check_collision(path[start_index], path[goal_index], obstacles):
                if goal_index == len(path)-1 or check_collision(path[start_index], path[goal_index+1], obstacles):
                    path_modified0.append(path[goal_index])
                    start_index = goal_index
        # path_modified为修正了角度小于THETA_THRESHOLD后的路径。
        path_modified = path_modified0
        if len(path_modified0) >= 3:
            for i in range(len(path_modified0)-2):
                # 如果theta太小找到point_candidate使得∠point_candidate-point1-point2足够大。
                [point0, point1, point2] = [path_modified0[i],
                                            path_modified0[i+1], path_modified0[i+2]]
                theta = cal_theta(point0, point1, point2)
                if theta < THETA_THRESHOLD:
                    for j in range(len(path)):
                        if path[j] == point0:
                            index0 = j
                        if path[j] == point1:
                            index1 = j
                            break
                    point_insert = None
                    for j in range(len(path)):
                        point_candidate = path[j]
                        if j > index0 and j < index1 and cal_theta(point_candidate, point1, point2) >= THETA_THRESHOLD:
                            point_insert = point_candidate
                            # 将point_insert插入到point0和point1之中以修正过小的夹角。
                            path_modified.insert(i+1, point_insert)
                            break
        return path_modified



# 生成一棵从start到goal的树且避开obstacles中的障碍物。
def rrt(start, goal, obstacles):
    # 初始化一棵树并加入起点。
    tree = ClassTree()
    tree.add_point(start)
    current_point = start
    step = 20
    num_initiation = 1
    # 扩展这棵树直到这棵树上有一点到终点足够近且到终点的路上没有障碍物。
    while cal_dist(current_point, goal) >= 10 or check_collision(current_point, goal, obstacles):
        # 随机生成一个sample_point并从current_point到这个点延申一个固定的步长生成一个新的点，
        # 若这个新的点不在树上且到current_point的直线不与障碍物相交，则将其添加到树上。
        sample_point = tree.generate_random_point(goal)
        #proper_point = tree.find_proper_point(sample_point)
        proper_point = tree.find_nearest_point(sample_point)
        if sample_point != proper_point:
            # 新生成的点。
            theta = math.atan2(sample_point[1]-proper_point[1],
                               sample_point[0]-proper_point[0])
            point = (int(proper_point[0] + step * math.cos(theta)),
                     int(proper_point[1] + step * math.sin(theta)))
            # 碰撞检测。
            is_collision = check_collision(
                point, proper_point, obstacles)
            if not is_collision:
                tree.add_point(point)
                tree.parent[point] = proper_point
                current_point = point
        # 若生成的点的个数超过1e3个，则初始化这棵树重新开始采样。
        if len(tree.points) >= 1e3:
            tree = ClassTree()
            tree.add_point(start)
            current_point = start
            num_initiation += 1
        # 若尝试了3次之后仍然不能生成一颗合适的树则终止任务。
        if num_initiation >= 3:
            print('Failed to generate a tree from start to goal.')
            break
    # 如果终点不在树上则手动添加。
    if goal not in tree.parent:
        tree.add_point(goal)
        tree.parent[goal] = current_point
    # 输出生成的树。
    return tree


if __name__ == "__main__":
    pass
