import math
# 状态空间的范围。
[WIDTH, HEIGHT] = [800, 800]
# 障碍物的半径。
OBSTACLES_RADIUS = 20

# 起点和终点。
START_INIT = (30, 400)
GOAL_INIT = (WIDTH-30, 400)

# 随机树修正系数。
THETA_THRESHOLD = 0.5*math.pi
NUM_POINTS = 100

# 各种颜色对应的RGB值。
BLACK = [0, 0, 0]
WHITE = [255, 255, 255]
GREY = [190, 190, 190]
RED = [240, 65, 85]
BLUE = [0, 0, 255]
GREEN = [0, 255, 0]
PINK = [255, 192, 203]
