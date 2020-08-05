import matplotlib.pyplot as plt
import numpy as np

from configs import *
from rrt_algorithm import *


# 根据控制点生成B样条曲线。
def generate_B_spline(control_points):
    def de_Boor_x(r, t, i):
        if r == 0:
            return control_points[i][0]
        else:
            if T[i+k-r]-T[i] == 0 and T[i+k-r]-T[i] != 0:
                return ((T[i+k-r]-t)/(T[i+k-r]-T[i]))*de_Boor_x(r-1, t, i-1)
            elif T[i+k-r]-T[i] != 0 and T[i+k-r]-T[i] == 0:
                return ((t-T[i])/(T[i+k-r]-T[i]))*de_Boor_x(r-1, t, i)
            elif T[i+k-r]-T[i] == 0 and T[i+k-r]-T[i] == 0:
                return 0
            else:
                return ((t-T[i])/(T[i+k-r]-T[i]))*de_Boor_x(r-1, t, i)+((T[i+k-r]-t)/(T[i+k-r]-T[i]))*de_Boor_x(r-1, t, i-1)

    def de_Boor_y(r, t, i):
        if r == 0:
            return control_points[i][1]
        else:
            if T[i+k-r]-T[i] == 0 and T[i+k-r]-T[i] != 0:
                return ((T[i+k-r]-t)/(T[i+k-r]-T[i]))*de_Boor_y(r-1, t, i-1)
            elif T[i+k-r]-T[i] != 0 and T[i+k-r]-T[i] == 0:
                return ((t-T[i])/(T[i+k-r]-T[i]))*de_Boor_y(r-1, t, i)
            elif T[i+k-r]-T[i] == 0 and T[i+k-r]-T[i] == 0:
                return 0
            else:
                return ((t-T[i])/(T[i+k-r]-T[i]))*de_Boor_y(r-1, t, i)+((T[i+k-r]-t)/(T[i+k-r]-T[i]))*de_Boor_y(r-1, t, i-1)

    # 阶数和控制点个数。
    [k, n] = [3, len(control_points)]
    # T范围0到1，生成均匀B样条曲线。
    T = [i / (NUM_POINTS-1) for i in list(range(NUM_POINTS))]
    data_points = []
    if n > k-1:
        for j in range(k-1, n):
            for t in np.linspace(T[j], T[j+1]):
                data_points.append(
                    (de_Boor_x(k-1, t, j), de_Boor_y(k-1, t, j)))
    else:
        print('Failed to generate the Beta spline curve, check input.')
    return data_points


# 根据采样点生成B样条曲线。
def generate_B_spline_from_samples(sample_points):
    # B样条曲线的次数和其上点的个数。
    [k, num_points] = [2, NUM_POINTS]
    # 采样点的个数和具体的坐标。
    num_samples = len(sample_points)
    [sample_x, sample_y] = [[], []]
    for i in range(len(sample_points)):
        sample_x.append(sample_points[i][0])
        sample_y.append(sample_points[i][1])
    sample_points = [sample_x, sample_y]
    # 参数计算。
    p_centripetal = centripetal(num_samples, sample_points)
    # knot向量计算。
    knot = knot_vector(p_centripetal, k, num_samples)
    # 控制点计算。
    control_points = curve_interpolation(
        sample_points, num_samples, k, p_centripetal, knot)
    # 得到B样条曲线上点的确切位置。
    params = np.linspace(0, 1, num_points)
    data_points = curve(control_points, num_samples, k, params, knot)
    return [data_points, control_points]


# 生成knot向量，供基函数使用。
def knot_vector(param, k, N):
    m = N + k
    knot = np.zeros((1, m+1))
    for i in range(k + 1):
        knot[0][i] = 0
    for i in range(m - k, m + 1):
        knot[0][i] = 1
    for i in range(k + 1, m - k):
        for j in range(i - k, i):
            knot[0][i] = knot[0][i] + param[j]
        knot[0][i] = knot[0][i] / k
    return knot[0]


# 由Cox-deBoor递推公式得到的B样条曲线的基函数。
# i；基函数的序号，k：次数，u：参数，knot：knot向量。
def base_function(i, k, u, knot):
    Nik_u = 0.0
    if k == 1:
        if u >= knot[i] and u < knot[i + 1]:
            Nik_u = 1.0
    else:
        length1 = knot[i + k - 1] - knot[i]
        length2 = knot[i + k] - knot[i + 1]
        if not length1 and not length2:
            Nik_u = 0
        elif not length1:
            Nik_u = (knot[i + k] - u) / length2 * \
                base_function(i + 1, k - 1, u, knot)
        elif not length2:
            Nik_u = (u - knot[i]) / length1 * base_function(i, k - 1, u, knot)
        else:
            Nik_u = (u - knot[i]) / length1 * base_function(i, k - 1, u, knot) + \
                    (knot[i + k] - u) / length2 * \
                base_function(i + 1, k - 1, u, knot)
    return Nik_u


# 对于已有的数据点，需要将其映射到参数域[0, 1]，此处根据数据点弦长的指数结果分割参数域。
def centripetal(n, P):
    a = 0.5
    parameters = np.zeros((1, n))
    for i in range(1, n):
        dis = 0
        for j in range(len(P)):
            dis = dis + (P[j][i]-P[j][i-1])**2
        dis = np.sqrt(dis)
        parameters[0][i] = parameters[0][i-1] + np.power(dis, a)
    for i in range(1, n):
        parameters[0][i] = parameters[0][i] / parameters[0][n-1]
    return parameters[0]


# 计算B样条曲线的控制点。
# sample_points：待拟合的采样点；N：数据点的个数；k：次数；param：参数；knot：knot向量。
def curve_interpolation(sample_points, N, k, param, knot):
    Nik = np.zeros((N, N))
    for i in range(N):
        for j in range(N):
            Nik[i][j] = base_function(j, k+1, param[i], knot)
    Nik[N-1][N-1] = 1
    Nik_inv = np.linalg.inv(Nik)
    control_points = []
    for i in range(len(sample_points)):
        control_points.append(np.dot(Nik_inv, sample_points[i]).tolist())
    return control_points


# 根据控制点等参数生成B样条曲线。
# control_points：控制点；N：控制点的个数；k：次数；param：参数；knot：knot向量。
def curve(control_points, N, k, param, knot):
    Nik = np.zeros((len(param), N))
    for i in range(len(param)):
        for j in range(N):
            Nik[i][j] = base_function(j, k+1, param[i], knot)
    Nik[len(param)-1][N - 1] = 1
    data_points = []
    for i in range(len(control_points)):
        data_points.append(np.dot(Nik, control_points[i]).tolist())
    return data_points


if __name__ == "__main__":
    # 待拟合的采样点数据。
    sample_points = [(0, 0), (1, 1), (2, 2), (6, 2), (7, 1), (8, 0)]
    # 拟合得到的数据点和控制点。
    [data_points, control_points] = generate_B_spline_from_samples(
        sample_points)
    # 绘制采样点和生成的控制点。
    num_samples = len(sample_points)
    fig = plt.figure()
    for i in range(num_samples):
        plt.scatter(sample_points[i][0], sample_points[i][1], color='r')
        plt.scatter(control_points[0][i], control_points[1][i], color='b')
    for i in range(num_samples - 1):
        tmp_x = [control_points[0][i], control_points[0][i+1]]
        tmp_y = [control_points[1][i], control_points[1][i+1]]
        plt.plot(tmp_x, tmp_y, color='b')
    # 绘制生成的B样条曲线。
    for i in range(len(data_points[0]) - 1):
        tmp_x = [data_points[0][i], data_points[0][i+1]]
        tmp_y = [data_points[1][i], data_points[1][i+1]]
        plt.plot(tmp_x, tmp_y, color='g')
    plt.axis('equal')
    plt.grid(True)
    plt.show()
