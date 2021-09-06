import numpy as np
from scipy.spatial.distance import cdist


# a = [(0, 0), (100, 20), (80, 40), (110, 0), (100, - 20), (50, 60)]
# print(a)
# b = cdist(a, a).astype(np.int)
# print(b)
#
# c = np.zeros((6, 6), np.int)
# for i in range(6):
#     for j in range(6):
#         c[i][j] = b[0][i] + b[j][0] - b[i][j]
# print(c)


def solomon(n, dis_matrix, time_matrix, demand, time_lower_bound, time_upper_bound, service_time, Q):
    points_2_insert = set(range(1, n))
    alpha = 0.9
    mu = 1
    lam = 1
    routes = []
    current_r = [0, 0]

    def find_best(_r, _i):
        if sum([demand[m] for m in _r]) + demand[_i] > Q:
            return False, None, None
        _best_loc = None
        _point = None
        __min_f_1 = float('inf')
        __s = 0
        __e = 0
        _old_b = [0]
        for _k in range(1, len(_r)):
            __s = max(0, __e + time_matrix[_r[_k - 1]][_r[_k]] - time_lower_bound[_r[_k]]) + \
                  time_lower_bound[_r[_k]]
            __e = __s + service_time[_r[_k]]
            _old_b.append(__s)
        for _j in range(len(_r) - 1):
            _new_r = _r[:_j + 1] + [_i] + _r[_j + 1:]
            _flag = True
            _s = 0
            _e = 0
            _temp_b = [0]
            for _k in range(1, len(_new_r)):
                _s = max(0, _e + time_matrix[_new_r[_k - 1]][_new_r[_k]] - time_lower_bound[_new_r[_k]]) + \
                     time_lower_bound[_new_r[_k]]
                _e = _s + service_time[_new_r[_k]]
                _temp_b.append(_s)
                if _s > time_upper_bound[_new_r[_k]]:
                    _flag = False
                    break
            if _flag is True:
                value1 = alpha * (
                        dis_matrix[_new_r[_j]][_new_r[_j + 1]] + dis_matrix[_new_r[_j + 1]][_new_r[_j + 2]] - mu *
                        dis_matrix[_new_r[_j]][_new_r[_j + 2]])
                value2 = (1 - alpha) * (_temp_b[_j + 2] - _old_b[_j + 1])
                value = value1 + value2
                if value < __min_f_1:
                    _best_loc = _j
                    __min_f_1 = value
        if _best_loc is not None:
            return True, __min_f_1, _best_loc
        else:
            return False, None, None

    while len(points_2_insert) > 0:
        if len(current_r) == 2:
            min_start = float('inf')
            point = None
            for i in points_2_insert:
                if time_upper_bound[i] < min_start:
                    min_start = time_upper_bound[i]
                    point = i
            print(point)
            points_2_insert.remove(point)
            current_r = [0] + [point] + [0]
        else:
            best_loc = None
            point = None
            max_f_2 = -float('inf')
            for i in points_2_insert:
                _feas, _min_f_1, p = find_best(current_r, i)
                if _feas is True:
                    f_2 = lam * dis_matrix[0][i] - _min_f_1
                    print(i,_min_f_1,current_r[p],current_r[p+1],f_2)
                    if f_2 > max_f_2:
                        max_f_2 = f_2
                        point = i
                        best_loc = p
            if point is not None:
                points_2_insert.remove(point)
                current_r = current_r[:best_loc + 1] + [point] + current_r[best_loc + 1:]
            else:
                routes.append(current_r)
                current_r = [0, 0]
    routes.append(current_r)
    print(routes)


if __name__ == "__main__":
    dis = [[0, 28.4, 58.1, 84.7, 83.4, 41.1, 63, 59.5, 31.8, 51.5, 56.7, 62.1, 55.1],
           [28.4, 0, 48.1, 89.7, 76.1, 52.7, 67.9, 75.3, 36.8, 82.5, 87.8, 47.5, 66.7],
           [58.1, 48.1, 0, 46.3, 34.1, 48.6, 24.5, 66.1, 48.6, 95.3, 80, 13, 51.9],
           [84.7, 89.7, 46.3, 0, 36.8, 34.9, 23.2, 35.1, 55.2, 70, 47, 53.3, 21],
           [83.4, 76.1, 34.1, 36.8, 0, 53.8, 21.8, 63.4, 64.4, 100.4, 77.3, 41, 49.2],
           [41.1, 52.7, 48.6, 34.9, 53.8, 0, 33.4, 22.7, 21, 50.2, 34.8, 55.5, 14.2],
           [63, 67.9, 24.5, 23.2, 21.8, 33.4, 0, 42.9, 43.9, 79.9, 56.8, 31.4, 28.7],
           [59.5, 75.3, 66.1, 35.1, 63.4, 22.7, 42.9, 0, 43.5, 39.8, 16.9, 72.9, 15.1],
           [31.8, 36.8, 48.6, 55.2, 64.4, 21, 43.9, 43.5, 0, 50.3, 43.9, 52.7, 34.9],
           [51.5, 82.5, 95.3, 70, 100.4, 50.2, 79.9, 39.8, 50.3, 0, 24.7, 100.6, 47.8],
           [56.7, 87.8, 80, 47, 77.3, 34.8, 56.8, 16.9, 43.9, 24.7, 0, 86.9, 28.4],
           [62.1, 47.5, 13, 53.3, 41, 55.5, 31.4, 72.9, 52.7, 100.6, 86.9, 0, 58.8],
           [55.1, 66.7, 51.9, 21, 49.2, 14.2, 28.7, 15.1, 34.9, 47.8, 28.4, 58.8, 0]]
    _time = np.array([[0, 187, 206, 161, 181, 151, 190, 160, 181, 201, 168, 196, 381],
                      [187, 0, 20, 36, 30, 42, 51, 54, 70, 73, 102, 100, 262],
                      [206, 20, 0, 50, 36, 58, 51, 64, 73, 70, 108, 100, 250],
                      [161, 36, 50, 0, 20, 10, 36, 20, 45, 57, 70, 76, 255],
                      [181, 30, 36, 20, 0, 30, 22, 28, 40, 45, 73, 71, 240],
                      [151, 42, 58, 10, 30, 0, 45, 22, 50, 64, 71, 81, 262],
                      [190, 51, 51, 36, 22, 45, 0, 30, 22, 22, 58, 50, 219],
                      [160, 54, 64, 20, 28, 22, 30, 0, 28, 45, 50, 58, 242],
                      [181, 70, 73, 45, 40, 50, 22, 28, 0, 20, 36, 32, 214],
                      [201, 73, 70, 57, 45, 64, 22, 45, 20, 0, 50, 32, 198],
                      [168, 102, 108, 70, 73, 71, 58, 50, 36, 50, 0, 30, 215],
                      [196, 100, 100, 76, 71, 81, 50, 58, 32, 32, 30, 0, 189],
                      [381, 262, 250, 255, 240, 262, 219, 242, 214, 198, 215, 189, 0]]) / 10
    _demand = [0, 10, 30, 10, 10, 10, 20, 20, 20, 10, 10, 10, 20, 30]
    _time_lower_bound = [0, 912, 825, 65, 727, 15, 621, 170, 255, 534, 357, 448, 652, 30]
    _time_upper_bound = [1236, 967, 870, 146, 782, 67, 702, 225, 324, 605, 410, 505, 721, 92]
    _service_time = [0] + [10] * 12
    _Q = 30
    n = 13
    solomon(n, _time, _time, _demand, _time_lower_bound, _time_upper_bound, _service_time, _Q)
