import re
import numpy as np
from scipy.spatial.distance import cdist
import csv


def read_single_soloman(filename):
    with open(filename, 'r') as file_to_read:
        pos = []
        while True:
            lines = file_to_read.readline()  # 整行读取数据
            if not lines:
                break
                pass
            if re.match(r'\s*[0-9]', lines) is not None:
                lines = lines.strip()
                lines = lines.split()
                lines = list(map(int, lines))
                pos.append(lines)  # 添加新读取的数据
            pass
    pass
    info = pos[0]
    mat = pos[1:]
    return info, mat


def get_dis_mat(cor):
    return cdist(cor, cor)


def resolve_soloman(info, mat):
    vehicle_num, capacity = info
    mat = np.array(mat)
    dis_mat = get_dis_mat(mat[:, 1:3])
    demand = mat[:, 3]
    t_win = mat[:, 4:6]
    t_ser = mat[:, 6]
    return vehicle_num, capacity, dis_mat, demand, t_win, t_ser


def get_soloman_cor(filepath):
    _info, _mat = read_single_soloman(filepath)
    _mat = np.array(_mat)
    return _mat[:, 1:3]


def transform_soloman(filepath='.\TempFiles\C101.txt'):
    _info, _mat = read_single_soloman(filepath)
    _vehicle_num, _capacity, _dis_mat, _demand, _t_win, _t_ser = resolve_soloman(_info, _mat)

    _station_num = _dis_mat.shape[0] - 1

    # _capacity = _capacity
    _capacity = 200

    H = _t_win[0, 1] * 2.5

    _dis_mat = np.around(_dis_mat, 1) * 10

    _nodes = list(range(1, _station_num + 1))

    _demand = _demand[1:].tolist()

    _nodes_info = list(zip(_nodes, _demand))

    with open('.\TestSets\RC101_50.csv', 'w', encoding='utf-8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([_station_num, _capacity, H * 10])
        writer.writerows(_dis_mat.tolist())
        writer.writerows(_nodes_info)

    return


if __name__ == '__main__':
    transform_soloman()
