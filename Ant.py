# -*- coding: utf-8 -*-
import random
import copy
import time
from collections import Counter
import re
import numpy as np
import csv
from Data import *

# 参数
'''
ALPHA:信息启发因子，值越大，则蚂蚁选择之前走过的路径可能性就越大
      ，值越小，则蚁群搜索范围就会减少，容易陷入局部最优
BETA:Beta值越大，蚁群越就容易选择局部较短路径，这时算法收敛速度会
     加快，但是随机性不高，容易得到局部的相对最优
'''
(ALPHA, BETA, RHO, Q) = (1.0, 2.0, 0.5, 100.0)

# 蚁群
ant_num = 100

total_points = int(supply_num + demand_num + 1)

pheromone_graph = [[10 for _ in range(total_points)] for _ in range(total_points)]


# ----------- 蚂蚁 -----------
class Ant(object):

    # 初始化
    def __init__(self, ID):

        self.ID = ID  # ID

        self.available_battery = 0
        self.used_battery = 0
        self.used_vehicles = 0

        self.path = []
        self.total_distance = 0.0  # 当前路径的总距离

        self.t = 0
        self.current_city = 0  # 当前停留的城市

        self.open_supply = []
        self.open_demand = []

        self.supply_counter = Counter()
        self.demand_counter = Counter()

        self.__clean_data()  # 随机初始化出生点

    # 初始数据
    def __clean_data(self):

        self.path = []  # 当前蚂蚁的路径
        self.t = 0.0  # 当前路径的总距离

        self.open_supply = supplys.copy()
        self.open_demand = demands.copy()

        self.supply_counter = Counter(supply_count)
        self.demand_counter = Counter(demand_count)

        self.path.append([0])
        self.current_city = 0  # 当前停留的城市

    # def __estimate_time(self, _current_city, _destination_, return_supplys):
    #
    # def __choice_next_point(self):
    #     _possible_points = []
    #     _select_prob = []
    #     if self.available_battery == 0 and self.used_battery == 0 and self.t>=0.9*H:
    #         prob = pow(pheromone_graph[self.current_city][0], ALPHA) * pow(
    #             (1.0 / distance_graph[self.current_city][0]), BETA)
    #         _select_prob.append(prob)
    #
    #     if self.available_battery < vehicle_capacity :
    #         for _s in self.open_supply:
    #             if self.t + estimate_time()

    def __choice_next_supply(self):

        if self.current_city in self.open_supply:
            return self.current_city

        _possible_supply = []
        _select_supply_prob = []
        for _s in self.open_supply:
            if self.t + time_graph[self.current_city][_s] + time_graph[_s][0] <= H:
                _possible_supply.append(_s)
                prob = pow(pheromone_graph[self.current_city][_s], ALPHA) * pow(
                    (1.0 / distance_graph[self.current_city][_s]), BETA)
                _select_supply_prob.append(prob)

        total_prob = sum(_select_supply_prob)

        if total_prob > 0.0:
            # 产生一个随机概率,0.0-total_prob
            temp_prob = random.uniform(0.0, total_prob)
            for i, supply in enumerate(_possible_supply):
                # 轮次相减
                temp_prob -= _select_supply_prob[i]
                if temp_prob < 0.0:
                    next_city = supply
                    return next_city

        return -1

    def __choice_next_demand(self, return_supply):

        _possible_demand = []
        _select_demand_prob = []

        for _d in self.open_demand:
            if self.t + time_graph[self.current_city][_d] + time_graph[_d][return_supply] + \
                    time_graph[return_supply][0] <= H:
                _possible_demand.append(_d)
                prob = pow(pheromone_graph[self.current_city][_d], ALPHA) * pow(
                    (1.0 / distance_graph[self.current_city][_d]), BETA)
                _select_demand_prob.append(prob)

        total_prob = sum(_select_demand_prob)

        if total_prob > 0.0:
            # 产生一个随机概率,0.0-total_prob
            temp_prob = random.uniform(0.0, total_prob)
            for i, demand in enumerate(_possible_demand):
                # 轮次相减
                temp_prob -= _select_demand_prob[i]
                if temp_prob < 0.0:
                    next_city = demand
                    return next_city

        return -1

    # 计算路径总距离
    def __cal_total_distance(self):

        temp_distance = 0.0

        for _r in self.path:
            for start, end in zip(_r, _r[1:]):
                temp_distance += distance_graph[start][end]
        self.total_distance = temp_distance

    def __start_another_route(self):
        self.path[-1].append(0)
        self.path.append([0])
        self.t = 0
        self.used_vehicles += 1
        self.current_city = 0

    # 移动操作
    def __move_out_of_supply(self, next_supply):

        self.path[-1].append(next_supply)
        battery_to_load = min(self.supply_counter[next_supply], vehicle_capacity)
        self.supply_counter[next_supply] -= battery_to_load
        self.available_battery += battery_to_load

        self.t += time_graph[self.current_city][next_supply]
        self.current_city = next_supply

    def __move_back_to_supply(self, to_supply):
        self.path[-1].append(to_supply)
        self.supply_counter[to_supply] += self.available_battery
        self.available_battery = 0
        self.used_battery = 0
        if self.supply_counter[to_supply] == 0:
            self.open_supply.remove(to_supply)
        self.t += time_graph[self.current_city][to_supply]
        self.current_city = to_supply

    def __pass_demand(self, demand):
        self.path[-1].append(demand)
        battery_to_change = min(self.available_battery, self.demand_counter[demand])
        self.available_battery -= battery_to_change
        self.used_battery += battery_to_change
        self.demand_counter[demand] -= battery_to_change
        if self.demand_counter[demand] == 0:
            self.open_demand.remove(demand)
        self.t += time_graph[self.current_city][demand]
        self.current_city = demand

    # 搜索路径
    def search_path(self):

        # 初始化数据
        self.__clean_data()
        terminated = True
        # 搜素路径，遍历完所有城市为止
        while self.open_demand:
            next_supply = self.__choice_next_supply()
            if next_supply == -1:
                self.__start_another_route()
                continue
            self.__move_out_of_supply(next_supply)
            flag = False
            while self.available_battery > 0:
                next_demand = self.__choice_next_demand(next_supply)
                if next_demand == -1:
                    flag = True
                    break
                self.__pass_demand(next_demand)
            self.__move_back_to_supply(next_supply)
            if flag is True:
                self.__start_another_route()
        self.path[-1].append(0)
        self.t += time_graph[self.current_city][0]
        self.current_city = 0
        # 计算路径总长度
        self.__cal_total_distance()


class VRP(object):

    def __init__(self):
        self.clean_phermone()

        self.ants = [Ant(ID) for ID in range(ant_num)]  # 初始蚁群
        self.best_ant = Ant(-1)  # 初始最优解
        self.best_ant.total_distance = 1 << 31  # 初始最大距离
        self.iter = 1  # 初始化迭代次数

    def clean_phermone(self):
        # 初始城市之间的距离和信息素
        for i in range(total_points):
            for j in range(total_points):
                pheromone_graph[i][j] = 10

    # 开始搜索
    def search_path(self, max_iter=100):
        record = []
        non_imp_iter = 0
        while self.iter <= max_iter:
            # 遍历每一只蚂蚁
            non_imp_iter += 1
            for ant in self.ants:
                # 搜索一条路径
                ant.search_path()
                # 与当前最优蚂蚁比较
                if ant.total_distance < self.best_ant.total_distance:
                    # 更新最优解
                    self.best_ant = copy.deepcopy(ant)
                    non_imp_iter = 0
            if non_imp_iter >= 100:
                self.clean_phermone()
            # 更新信息素
            self.__update_pheromone_graph()
            # print(u"迭代次数：", self.iter, u"最佳路径总距离：", self.best_ant.total_distance)
            record.append([self.iter, self.best_ant.total_distance])
            self.iter += 1
        path = self.best_ant.path
        return path, record

    # 更新信息素
    def __update_pheromone_graph(self):

        # 获取每只蚂蚁在其路径上留下的信息素
        temp_pheromone = [[0.0 for _ in range(total_points)] for _ in range(total_points)]
        for ant in self.ants:
            for _r in ant.path:
                for start, end in zip(_r, _r[1:]):
                    # 在路径上的每两个相邻城市间留下信息素，与路径总距离反比
                    temp_pheromone[start][end] += Q / ant.total_distance
                    temp_pheromone[end][start] += Q / ant.total_distance

        # 更新所有城市之间的信息素，旧信息素衰减加上新迭代信息素
        for i in range(total_points):
            for j in range(total_points):
                pheromone_graph[i][j] = pheromone_graph[i][j] * RHO + temp_pheromone[i][j]


# ----------- 程序的入口处 -----------
def ant_vrp():
    start = time.perf_counter()
    vrp = VRP()
    _path, _record = vrp.search_path(200)
    end = time.perf_counter()
    print('CPU运行时间', end - start)
    print('最优解', _record[-1][-1])


if __name__ == '__main__':
    # start = time.perf_counter()
    # vrp = VRP()
    # _path, _record = vrp.search_path(1000)
    # _supply = []
    # _demand = []
    # _gene = []
    # for _rt in _path:
    #     for _i in _rt:
    #         if _i in supplys and _i not in _supply:
    #             _supply.append(_i)
    #         if _i in demands and _i not in _demand:
    #             _demand.append(_i)
    # for _i in _supply:
    #     _gene.append(supplys.index(_i))
    # for _i in _demand:
    #     _gene.append(demands.index(_i))
    # print(_gene)
    # end = time.perf_counter()
    # print('CPU运行时间', end - start)
    # with open('ant.csv', 'w', newline='') as f:
    #     f_csv = csv.writer(f)
    #     f_csv.writerows(_record)
    # for i in range(5):
    #     print('迭代次数', i)
    #     ant_vrp()
    ant_vrp()
