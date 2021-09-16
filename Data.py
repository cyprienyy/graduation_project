# -*- coding: utf-8 -*-
import re
import numpy as np


def resolve_self_created_case(filename):
    with open(filename, 'r') as file_to_read:
        pos = []
        while True:
            lines = file_to_read.readline()  # 整行读取数据
            if not lines:
                break
                pass
            if re.match(r'\s*[0-9]', lines) is not None:
                lines = lines.strip()
                lines = lines.split(',')
                lines = list(map(float, lines))
                pos.append(lines)  # 添加新读取的数据
            pass
    pass
    return pos


supply_num = 0
demand_num = 0

supplys = []
demands = []

supply_count = dict()
demand_count = dict()

pos = resolve_self_created_case('TestSets/bp_test.csv')
station_num, vehicle_capacity, H = map(int, pos[0])

vehicle_capacity = 200

distance_graph = np.array(pos[1:station_num + 2])
time_graph = distance_graph
for info in pos[station_num + 2:]:
    loc, num = map(int, info)
    if num < 0:
        supply_num += 1
        supplys.append(loc)
        supply_count[loc] = -num
    elif num > 0:
        demand_num += 1
        demands.append(loc)
        demand_count[loc] = num

del loc, num, pos, info
