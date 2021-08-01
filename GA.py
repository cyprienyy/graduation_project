# -*- coding: utf-8 -*-
import random
import copy
import time
from collections import Counter
import re
import numpy as np
import geatpy as ea
from GA_Templet import soea_psy_EGA_templet_1, soea_psy_EGA_templet_2
import csv
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
        return distance_graph[st][mids[0]] + distance_graph[mids[0]][ed]
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
    return _dis


def calculate_route_length(supply_order, demand_order):
    _supply = [supplys[i] for i in supply_order]
    _demand = [demands[i] for i in demand_order]
    _new_demand, _new_demand_loc, _new_demand_supply = helper_1(_supply, _demand)
    return helper_2(_new_demand, _new_demand_loc, _new_demand_supply)


def helper_2(_new_d, _new_d_l, _new_d_s):
    flag = True
    dis = 0
    while flag:
        flag, _new_d, _new_d_l, _new_d_s, _dis_add = helper_3(_new_d, _new_d_l, _new_d_s)
        dis += _dis_add
    return dis


def helper_3(_new_d, _new_d_l, _new_d_s):
    _new_demand = [0] + _new_d + [0]
    _new_demand_loc = [0] + _new_d_l + [0]
    _new_demand_supply = [0] + _new_d_s + [0]

    _back_dis = [float('inf')] * len(_new_demand)
    _min_dis = [float('inf')] * len(_new_demand)
    _back_dis[0] = 0
    _min_dis[0] = distance_graph[0][_new_demand_supply[1]]
    for j in range(0, len(_min_dis) - 1):
        _base_dis = _min_dis[j]
        if _back_dis[j] > H:
            return True, _new_demand[j:-1], _new_demand_loc[j:-1], _new_demand_supply[j:-1], _back_dis[j - 1]
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
                # 计算back_dis
                _dis_3 = distance_graph[_temp_visit[-1]][_temp_return[0]] + distance_graph[_temp_return[0]][0]
                if _base_dis + _dis_1 + _dis_2 + _dis_3 > H:
                    break
                _back_dis[i] = min(_back_dis[i], _base_dis + _dis_1 + _dis_2 + _dis_3)
                # 计算min_dis
                _next_loc = _new_demand_supply[i + 1]
                _dis_4 = distance_graph[_temp_visit[-1]][_temp_return[0]] + distance_graph[_temp_return[0]][_next_loc]
                _min_dis[i] = min(_min_dis[i], _base_dis + _dis_1 + _dis_2 + _dis_4)
            else:

                _dis_2 += distance_graph[_temp_visit[-1]][_new_demand_loc[i]]
                if _temp_supply[-1] != _new_demand_supply[i]:
                    _dis_1 -= distance_graph[_temp_supply[-1]][_temp_visit[0]]
                    _dis_1 += distance_graph[_temp_supply[-1]][_new_demand_supply[i]]
                    _temp_supply.append(_new_demand_supply[i])
                    _temp_return.append(_new_demand_supply[i])
                    _temp_visit.append(_new_demand_loc[i])
                    # 待更新
                    _dis_1 += distance_graph[_temp_supply[-1]][_temp_visit[0]]
                    _dis_3 = estimate_shortest_route(_temp_visit[-1], _temp_return, 0)
                    if _base_dis + _dis_1 + _dis_2 + _dis_3 > H:
                        break
                    _back_dis[i] = min(_back_dis[i], _base_dis + _dis_1 + _dis_2 + _dis_3)
                    # 计算min_dis
                    _next_loc = _new_demand_supply[i + 1]
                    _dis_4 = estimate_shortest_route(_temp_visit[-1], _temp_return, _next_loc)
                    _min_dis[i] = min(_min_dis[i], _base_dis + _dis_1 + _dis_2 + _dis_4)
                else:
                    _temp_visit.append(_new_demand_loc[i])
                    # 待更新
                    _dis_3 = estimate_shortest_route(_temp_visit[-1], _temp_return, 0)
                    if _base_dis + _dis_1 + _dis_2 + _dis_3 > H:
                        break
                    _back_dis[i] = min(_back_dis[i], _base_dis + _dis_1 + _dis_2 + _dis_3)
                    # 计算min_dis
                    _next_loc = _new_demand_supply[i + 1]
                    _dis_4 = estimate_shortest_route(_temp_visit[-1], _temp_return, _next_loc)
                    _min_dis[i] = min(_min_dis[i], _base_dis + _dis_1 + _dis_2 + _dis_4)
    return False, [0], [0], [0], _back_dis[-2]


class MyProblem(ea.Problem):  # 继承Problem父类
    def __init__(self):
        name = 'MyProblem'  # 初始化name（函数名称，可以随意设置）
        M = 1  # 初始化M（目标维数）
        maxormins = [1]  # 初始化maxormins（目标最小最大化标记列表，1：最小化该目标；-1：最大化该目标）
        Dim = supply_num + demand_num  # 初始化Dim（决策变量维数）
        varTypes = [1] * Dim  # 初始化varTypes（决策变量的类型，元素为0表示对应的变量是连续的；1表示是离散的）
        lb = [0] * Dim  # 决策变量下界
        ub = [supply_num - 1] * supply_num + [demand_num - 1] * demand_num  # 决策变量上界
        lbin = [1] * Dim  # 决策变量下边界（0表示不包含该变量的下边界，1表示包含）
        ubin = [1] * Dim  # 决策变量上边界（0表示不包含该变量的上边界，1表示包含）
        # 调用父类构造方法完成实例化
        ea.Problem.__init__(self, name, M, maxormins, Dim, varTypes, lb, ub, lbin, ubin)

    def aimFunc(self, pop):
        X = pop.Phen.astype(int)
        ObjV = []
        for i in range(X.shape[0]):
            supply_order = X[i][:supply_num].tolist()
            demand_order = X[i][supply_num:].tolist()
            res = calculate_route_length(supply_order, demand_order)
            ObjV.append(res)
        pop.ObjV = np.array([ObjV]).T


def print_journey(X):
    supply_order = X[:supply_num]
    demand_order = X[supply_num:]
    print(calculate_route_length(supply_order, demand_order))


if __name__ == '__main__':
    start = time.perf_counter()
    """================================实例化问题对象==========================="""
    problem = MyProblem()
    """==================================种群设置=============================="""
    NIND = 100  # 种群规模
    # 创建区域描述器，这里需要创建两个，都使用排列编码
    Encodings = ['P', 'P']
    Field1 = ea.crtfld(Encodings[0], problem.varTypes[:supply_num], problem.ranges[:, :supply_num],
                       problem.borders[:, :supply_num])
    Field2 = ea.crtfld(Encodings[1], problem.varTypes[supply_num:], problem.ranges[:, supply_num:],
                       problem.borders[:, supply_num:])
    Fields = [Field1, Field2]
    population = ea.PsyPopulation(Encodings, Fields, NIND)
    """================================算法参数设置============================="""
    myAlgorithm = soea_psy_EGA_templet_1(problem, population)
    # myAlgorithm = soea_psy_EGA_templet_2(problem, population)
    myAlgorithm.MAXGEN = 200 # 最大进化代数
    myAlgorithm.logTras = 1  # 设置每隔多少代记录日志，若设置成0则表示不记录日志
    myAlgorithm.verbose = True  # 设置是否打印输出日志信息
    myAlgorithm.drawing = 0  # 设置绘图方式（0：不绘图；1：绘制结果图；2：绘制目标空间过程动画；3：绘制决策空间过程动画）
    """===========================根据先验知识创建先知种群========================"""
    # prophetChrom = [np.array([[2, 0,1.0]]),
                    # np.array([[21.0, 20, 16, 17, 18, 19, 9, 13, 0, 2, 5, 4, 7, 6, 3, 1, 8, 10, 12, 11, 15, 14]])]
    # prophetPop = ea.PsyPopulation(Encodings, Fields, 1, prophetChrom)  # 实例化种群对象（设置个体数为1）
    # myAlgorithm.call_aimFunc(prophetPop)  # 计算先知种群的目标函数值及约束（假如有约束）
    """===========================调用算法模板进行种群进化========================"""
    # [BestIndi, population], _record = myAlgorithm.run(prophetPop)
    [BestIndi, population], _record = myAlgorithm.run()
    end = time.perf_counter()
    print('CPU运行时间', end - start)
    BestIndi.save()
    """==================================输出结果=============================="""
    print('评价次数：%s' % myAlgorithm.evalsNum)
    print('时间已过 %s 秒' % myAlgorithm.passTime)
    with open('ga.csv', 'w', newline='')as f:
        f_csv = csv.writer(f)
        f_csv.writerows(_record)
    if BestIndi.sizes != 0:
        print('最优的目标函数值为：%s' % (BestIndi.ObjV[0][0]))
        print('最优的控制变量值为：')
        print(list(BestIndi.Phen[0]))

    else:
        print('没找到可行解。')
