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


def improve_route_length(supply_order, demand_order):
    _supply = [supplys[i] for i in supply_order]
    _demand = [demands[i] for i in demand_order]
    _new_demand, _new_demand_loc, _new_demand_supply = helper_1(_supply, _demand)
    _, _route = helper_2(_new_demand, _new_demand_loc, _new_demand_supply)
    _segments = [[0]]
    for i in _route:
        for j in i:
            _segments.append(delete_duplicate(j))
    _segments += [[0]]

    # print(_segments)
    _pre = 0
    for i in range(1, len(_segments) - 1):
        _next = head(_segments[i + 1])
        m = 0
        j = len(_segments[i]) - 1
        while _segments[i][m] in supplys:
            m += 1
            j -= 1
        if _segments[i][m] == _pre:
            m += 1
        if _segments[i][j] == _next:
            j -= 1
        search_neighborhood(_segments[i], m, j)
        _pre = _segments[i][j + 1]
    return _segments


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
    # _back_r[0] = [0]
    # _min_r[0] = [0]

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
                    _back_r[i] = _min_r[j] + [_temp_supply + _temp_visit + _temp_return]

                _next_loc = _new_demand_supply[i + 1]
                _dis_4 = distance_graph[_temp_visit[-1]][_temp_return[0]] + distance_graph[_temp_return[0]][_next_loc]
                if _min_dis[i] > _base_dis + _dis_1 + _dis_2 + _dis_4:
                    _min_dis[i] = _base_dis + _dis_1 + _dis_2 + _dis_4
                    _min_r[i] = _min_r[j] + [_temp_supply + _temp_visit + _temp_return]
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
                        _back_r[i] = _min_r[j] + [_temp_supply + _temp_visit + _r_1[1:-1]]

                    _next_loc = _new_demand_supply[i + 1]
                    _dis_4, _r_2 = estimate_shortest_route(_temp_visit[-1], _temp_return, _next_loc)
                    if _min_dis[i] > _base_dis + _dis_1 + _dis_2 + _dis_4:
                        _min_dis[i] = _base_dis + _dis_1 + _dis_2 + _dis_4
                        _min_r[i] = _min_r[j] + [_temp_supply + _temp_visit + _r_2[1:-1]]
                else:
                    _temp_visit.append(_new_demand_loc[i])

                    _dis_3, _r_1 = estimate_shortest_route(_temp_visit[-1], _temp_return, 0)
                    if _base_dis + _dis_1 + _dis_2 + _dis_3 > H:
                        break
                    if _back_dis[i] > _base_dis + _dis_1 + _dis_2 + _dis_3:
                        _back_dis[i] = _base_dis + _dis_1 + _dis_2 + _dis_3
                        _back_r[i] = _min_r[j] + [_temp_supply + _temp_visit + _r_1[1:-1]]

                    _next_loc = _new_demand_supply[i + 1]
                    _dis_4, _r_2 = estimate_shortest_route(_temp_visit[-1], _temp_return, _next_loc)
                    if _min_dis[i] > _base_dis + _dis_1 + _dis_2 + _dis_4:
                        _min_dis[i] = _base_dis + _dis_1 + _dis_2 + _dis_4
                        _min_r[i] = _min_r[j] + [_temp_supply + _temp_visit + _r_2[1:-1]]
    return False, [0], [0], [0], _back_dis[-2], _back_r[-2]


def search_neighborhood(_segment, m, n):
    def try_improve():
        for i in range(m, n + 1):
            for j in range(i + 1, n + 1):
                a = distance_graph[_segment[i - 1]][_segment[i]] + distance_graph[_segment[i]][_segment[i + 1]] + \
                    distance_graph[_segment[j - 1]][_segment[j]] + distance_graph[_segment[j]][_segment[j + 1]]
                _segment[i], _segment[j] = _segment[j], _segment[i]
                b = distance_graph[_segment[i - 1]][_segment[i]] + distance_graph[_segment[i]][_segment[i + 1]] + \
                    distance_graph[_segment[j - 1]][_segment[j]] + distance_graph[_segment[j]][_segment[j + 1]]
                _segment[i], _segment[j] = _segment[j], _segment[i]
                # print(a, b)
                if b < a:
                    return i, j
        return -1, -1

    flag = True
    while flag:
        flag = False
        _i, _j = try_improve()
        if _i > 0:
            _segment[_i], _segment[_j] = _segment[_j], _segment[_i]
            flag = True
            # print(_segment)
    return


def head(_segment):
    for i in _segment:
        if i not in supplys:
            return i


def delete_duplicate(_segment):
    ans = [_segment[0]]
    for i in _segment[1:]:
        if ans:
            if ans[-1] != i:
                ans.append(i)
    return ans


def tail():
    return


def improve_journey(supply_order, demand_order):
    ans = improve_route_length(supply_order, demand_order)
    _supply = []
    _demand = []
    _gene_1 = []
    _gene_2 = []
    for _rt in ans[1:-1]:
        for _i in _rt:
            if _i in supplys and _i not in _supply:
                _supply.append(_i)
            if _i in demands and _i not in _demand:
                _demand.append(_i)
    for _i in _supply:
        _gene_1.append(supplys.index(_i))
    for _i in _demand:
        _gene_2.append(demands.index(_i))
    return _gene_1, _gene_2


def cal_dis(_routes):
    dis = 0
    for _r in _routes:
        for i, j in zip(_r, _r[1:]):
            dis += distance_graph[i][j]
    print(dis)


if '__name__' == '__main__':
    _X_1 = [2, 1, 0, 5, 4, 3, ]
    _X_2 = [16, 15, 14, 12, 10, 9, 11, 13, 1, 3, 7, 6, 5, 0, 4, 2, 42, 43, 41, 37, 33, 30, 28, 26, 25, 27,
            29, 31, 32, 18, 21, 23, 19, 17, 20, 22, 24, 8, 40, 38, 36, 34, 35, 39]
    print(improve_journey(_X_1, _X_2))
