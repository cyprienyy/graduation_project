# -*- coding: utf-8 -*-
import re
import numpy as np
from Data import *


def helper_1(_supply, _demand):
    _s = supply_count[_supply[0]]
    _new_demand_loc = []
    _new_demand = []
    _new_demand_supply = []
    _s_index = 0
    _d_index = 0
    for _d_index in _demand:
        _d = demand_count[_d_index]
        while _d > 0:
            if _s == 0:
                _s_index += 1
                _s = supply_count[_supply[_s_index]]
            _minus = min(_d, _s, vehicle_capacity)
            _new_demand_loc.append(_d_index)
            _new_demand.append(_minus)
            _new_demand_supply.append(_supply[_s_index])
            _d -= _minus
            _s -= _minus
    return _new_demand, _new_demand_loc, _new_demand_supply


def estimate_shortest_route(st, mids, ed):
    if len(mids) == 1:
        return distance_graph[st][mids[0]] + distance_graph[mids[0]][ed], [st, mids[0], ed]
    _r = [st, ed]
    _dis = distance_graph[st][ed]
    _count = len(mids)
    _inserted = [False] * _count
    i = 0
    while i < _count:
        _point_to_insert = None
        _record_j = None
        _dis_add = float('inf')
        _insert_point = None
        for j, _loc in enumerate(mids):
            if _inserted[j] is True:
                continue
            else:
                for k, _k in enumerate(_r[:-1]):
                    _temp_dis = -distance_graph[_k][_r[k + 1]] + distance_graph[_k][_loc] + distance_graph[_loc][
                        _r[k + 1]]
                    if _temp_dis < _dis_add:
                        _dis_add = _temp_dis
                        _point_to_insert = _loc
                        _insert_point = k
                        _record_j = j
        _r = _r[:_insert_point + 1] + [_point_to_insert] + _r[_insert_point + 1:]
        _dis += _dis_add
        i += 1
        _inserted[_record_j] = True
    return _dis, _r


def calculate_route_length(supply_order, demand_order):
    _supply = [supplys[i] for i in supply_order]
    _demand = [demands[i] for i in demand_order]
    _new_demand, _new_demand_loc, _new_demand_supply = helper_1(_supply, _demand)
    return helper_2(_new_demand, _new_demand_loc, _new_demand_supply)


def helper_2(_new_d, _new_d_l, _new_d_s):
    flag = True
    dis = 0
    route = []
    while flag:
        flag, _new_d, _new_d_l, _new_d_s, _dis_add, _r = helper_3(_new_d, _new_d_l, _new_d_s)
        dis += _dis_add
        route.append(_r)
    return dis, route


def helper_3(_new_d, _new_d_l, _new_d_s):
    _new_demand = [0] + _new_d + [0]
    _new_demand_loc = [0] + _new_d_l + [0]
    _new_demand_supply = [0] + _new_d_s + [0]

    _back_dis = [float('inf')] * len(_new_demand)
    _min_dis = [float('inf')] * len(_new_demand)
    _back_dis[0] = 0
    _min_dis[0] = distance_graph[0][_new_demand_supply[1]]

    _back_r = [[] for _ in _new_demand]
    _min_r = [[] for _ in _new_demand]
    _back_r[0] = [0]
    _min_r[0] = [0]

    for j in range(0, len(_min_dis) - 1):
        _base_dis = _min_dis[j]
        if _back_dis[j] > H:
            return True, _new_demand[j:-1], _new_demand_loc[j:-1], _new_demand_supply[j:-1], _back_dis[j - 1], \
                   _back_r[j - 1]
        _temp_supply = []
        _temp_return = []
        _temp_visit = []
        _temp_d = 0
        count = 0
        for i in range(j + 1, len(_min_dis) - 1):
            _temp_d += _new_demand[i]
            if _temp_d > vehicle_capacity:
                break
            if count == 0:
                _dis_1 = 0
                _dis_2 = 0
                _temp_supply.append(_new_demand_supply[i])
                _temp_return.append(_new_demand_supply[i])
                _temp_visit.append(_new_demand_loc[i])
                _dis_1 += distance_graph[_temp_supply[-1]][_temp_visit[0]]

                count += 1

                _dis_3 = distance_graph[_temp_visit[-1]][_temp_return[0]] + distance_graph[_temp_return[0]][0]
                if _base_dis + _dis_1 + _dis_2 + _dis_3 > H:
                    break
                if _back_dis[i] > _base_dis + _dis_1 + _dis_2 + _dis_3:
                    _back_dis[i] = _base_dis + _dis_1 + _dis_2 + _dis_3
                    _back_r[i] = _min_r[j] + _temp_supply + _temp_visit + _temp_return + [0]

                _next_loc = _new_demand_supply[i + 1]
                _dis_4 = distance_graph[_temp_visit[-1]][_temp_return[0]] + distance_graph[_temp_return[0]][_next_loc]
                if _min_dis[i] > _base_dis + _dis_1 + _dis_2 + _dis_4:
                    _min_dis[i] = _base_dis + _dis_1 + _dis_2 + _dis_4
                    _min_r[i] = _min_r[j] + _temp_supply + _temp_visit + _temp_return
            else:

                _dis_2 += distance_graph[_temp_visit[-1]][_new_demand_loc[i]]
                if _temp_supply[-1] != _new_demand_supply[i]:
                    _dis_1 -= distance_graph[_temp_supply[-1]][_temp_visit[0]]
                    _dis_1 += distance_graph[_temp_supply[-1]][_new_demand_supply[i]]
                    _temp_supply.append(_new_demand_supply[i])
                    _temp_return.append(_new_demand_supply[i])
                    _temp_visit.append(_new_demand_loc[i])

                    _dis_1 += distance_graph[_temp_supply[-1]][_temp_visit[0]]
                    _dis_3, _r_1 = estimate_shortest_route(_temp_visit[-1], _temp_return, 0)
                    if _base_dis + _dis_1 + _dis_2 + _dis_3 > H:
                        break
                    if _back_dis[i] > _base_dis + _dis_1 + _dis_2 + _dis_3:
                        _back_dis[i] = _base_dis + _dis_1 + _dis_2 + _dis_3
                        _back_r[i] = _min_r[j] + _temp_supply + _temp_visit + _r_1[1:]

                    _next_loc = _new_demand_supply[i + 1]
                    _dis_4, _r_2 = estimate_shortest_route(_temp_visit[-1], _temp_return, _next_loc)
                    if _min_dis[i] > _base_dis + _dis_1 + _dis_2 + _dis_4:
                        _min_dis[i] = _base_dis + _dis_1 + _dis_2 + _dis_4
                        _min_r[i] = _min_r[j] + _temp_supply + _temp_visit + _r_2[1:-1]
                else:
                    _temp_visit.append(_new_demand_loc[i])

                    _dis_3, _r_1 = estimate_shortest_route(_temp_visit[-1], _temp_return, 0)
                    if _base_dis + _dis_1 + _dis_2 + _dis_3 > H:
                        break
                    if _back_dis[i] > _base_dis + _dis_1 + _dis_2 + _dis_3:
                        _back_dis[i] = _base_dis + _dis_1 + _dis_2 + _dis_3
                        _back_r[i] = _min_r[j] + _temp_supply + _temp_visit + _r_1[1:]

                    _next_loc = _new_demand_supply[i + 1]
                    _dis_4, _r_2 = estimate_shortest_route(_temp_visit[-1], _temp_return, _next_loc)
                    if _min_dis[i] > _base_dis + _dis_1 + _dis_2 + _dis_4:
                        _min_dis[i] = _base_dis + _dis_1 + _dis_2 + _dis_4
                        _min_r[i] = _min_r[j] + _temp_supply + _temp_visit + _r_2[1:-1]
    return False, [0], [0], [0], _back_dis[-2], _back_r[-2]


def print_journey(X):
    supply_order = X[:supply_num]
    demand_order = X[supply_num:]
    print(calculate_route_length(supply_order, demand_order))


def cal_dis(_routes):
    dis = 0
    for _r in _routes:
        for i, j in zip(_r, _r[1:]):
            dis += distance_graph[i][j]
    print(dis)


_X = [2.0, 1.0, 0.0, 20.0, 3.0, 18.0, 10.0, 5.0, 13.0, 11.0, 12.0, 1.0, 21.0, 19.0, 9.0, 16.0, 6.0, 7.0, 14.0, 4.0, 15.0, 0.0, 2.0, 8.0, 17.0]
print_journey([int(x) for x in _X])
# cal_dis([[0, 2, 2, 16, 25, 2, 24, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 5, 3, 7, 21, 20, 22, 25, 23, 10, 11,
#           8, 6, 4, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0], [0, 0]])
