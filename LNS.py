from collections import Counter
import numpy as np
from Data import *


# dis_matrix = np.zeros((1, 1))


class Node:
    def __init__(self, _loc=0, f_battery_q=0, e_battery_q=0, related_nodes=None, prev=None, after=None):
        self.loc = _loc
        self.f_battery_q = f_battery_q
        self.e_battery_q = e_battery_q
        self.related_nodes = related_nodes
        self.prev = prev
        self.after = after

    def get_type(self):
        if self.f_battery_q > 0:
            if self.e_battery_q < 0:
                return 4  # 同时取还电池的电池柜点
            elif self.e_battery_q == 0:
                return 5  # 取电池的电池柜点
        elif self.f_battery_q < 0:
            return -4  # 站点需求点
        elif self.f_battery_q == 0:
            if self.e_battery_q < 0:
                return -1  # 归还电池的电池柜点
            elif self.e_battery_q == 0:
                if self.prev is None:
                    return 8  # 路径头节点
                else:
                    return 9  # 路径尾节点


def insert_node1_after_node_2(node1, node2, k=0):
    node_k = node2
    for _ in range(k):
        node_k = node_k.after
    node1.prev = node_k
    node1.after = node_k.after
    node_k.after.prev = node1
    node_k.after = node1
    return


def remove_node_1(node1):
    if node1.prev is None or node1.after is None:
        return
    node1.prev.after = node1.after
    node1.after.prev = node1.prev
    node1.after, node1.prev = None, None
    return


def create_empty_path(loc_1=0, loc_2=0):
    _head = Node(loc_1)
    _tail = Node(loc_2)
    _head.after = _tail
    _tail.prev = _head
    return _head, _tail


def move_node_to_tail(node1, tail_1):
    if node1.after == tail_1:
        return False
    else:
        next_node = node1.after
        remove_node_1(node1)
        insert_node1_after_node_2(node1, next_node)
        return True


def check_load_feasibility_from_node1_to_node2(node_1, node_2, start_f, start_e, vehicle_q):
    if start_f < 0 or start_e < 0 or start_f + start_e > vehicle_q:
        return False, -1, -1
    _start_f = start_f
    _start_e = start_e
    node = node_1
    while node != node_2:
        node = node.after
        _start_f += node.f_battery_q
        _start_e += node.e_battery_q
        if _start_f < 0 or _start_e < 0 or _start_f + _start_e > vehicle_q:
            return False, -1, -1
    return True, _start_f, _start_e


class VRP:
    def __init__(self):

        self.vehicle_num_limit = 2
        self.vehicle_capacity_limit = [200] * self.vehicle_num_limit
        self.vehicle_dis_limit = [3600] * self.vehicle_num_limit

        self.vehicle_dis = [0] * self.vehicle_num_limit
        self.total_dis = sum(self.vehicle_dis)

        self.record_routes = None

        self.used_vehicle_num = 0
        self.single_vehicle_cost = 500
        self.vehicle_cost = 0

        self.route_heads = []
        self.route_tails = []

        self.demand_counter = demand_count.copy()
        self.supply_counter = supply_count.copy()

        return

    def init_vehicle_routes(self):
        for _ in range(self.vehicle_num_limit):
            _head, _tail = create_empty_path()
            self.route_heads.append(_head)
            self.route_tails.append(_tail)
        self.__record_current_routes()
        return

    def __recalculate_dis(self):
        for _r, _head in enumerate(self.route_heads):
            _tmp_dis = 0
            _node = _head
            _next_node = _head.after
            while _node != self.route_tails[_r]:
                _tmp_dis += self.call_dis(_node.loc, _next_node.loc)
                _node = _node.after
                _next_node = _next_node.after
            self.vehicle_dis[_r] = _tmp_dis
        self.total_dis = sum(self.vehicle_dis)

    def __record_used_vehicle_num(self):
        self.used_vehicle_num = 0
        for i, _head in enumerate(self.route_heads):
            if _head.after != self.route_tails[i]:
                self.used_vehicle_num += 1
        self.vehicle_cost = self.single_vehicle_cost * self.used_vehicle_num
        return

    @staticmethod
    def call_dis(loc_1, loc_2):
        return distance_graph[loc_1, loc_2]

    def __record_current_routes(self):
        self.record_routes = []
        for i, _head in enumerate(self.route_heads):
            _node = _head
            self.record_routes.append([_node.loc])
            while _node != self.route_tails[i]:
                _node = _node.after
                self.record_routes[-1].append(_node.loc)

    def create_initial_solution(self):
        flag = True
        while flag:
            flag = self.best_insertion()
        return

    def create_initial_solution_improved(self):
        flag = True
        while flag:
            flag = self.best_insertion_improved()
        return

    def best_insertion(self):

        sol_found = False
        best_ans = None

        supply_node = Node()
        demand_node = Node()
        return_node = Node()

        # 改为设计一个iterator
        for _r, _head in enumerate(self.route_heads):

            for _supply in self.supply_counter.keys():

                if self.supply_counter[_supply] == 0:
                    continue

                for _demand in self.demand_counter.keys():
                    if self.demand_counter[_demand] == 0:
                        continue

                    add_dis = 0

                    supply_node.loc = _supply
                    demand_node.loc = _demand
                    return_node.loc = _supply

                    supply_node.f_battery_q, supply_node.e_battery_q = 1, 0
                    demand_node.f_battery_q, demand_node.e_battery_q = -1, 1
                    return_node.f_battery_q, return_node.e_battery_q = 0, -1

                    supply_loc = -1
                    insert_node1_after_node_2(supply_node, _head)
                    flag_1 = True

                    while flag_1:

                        add_dis = add_dis - self.call_dis(supply_node.prev.loc,
                                                          supply_node.after.loc) + self.call_dis(
                            supply_node.prev.loc, supply_node.loc) + self.call_dis(supply_node.loc,
                                                                                   supply_node.after.loc)

                        supply_loc += 1
                        # 此处开始快进1
                        insert_node1_after_node_2(demand_node, supply_node)
                        demand_loc = -1
                        flag_2 = True

                        flag_1_1, _, _ = check_load_feasibility_from_node1_to_node2(_head, supply_node, 0, 0,
                                                                                    self.vehicle_capacity_limit[_r])
                        # 记录值以减少计算
                        while flag_2 and flag_1_1 and self.vehicle_dis[_r] + add_dis <= self.vehicle_dis_limit[_r]:

                            add_dis = add_dis - self.call_dis(demand_node.prev.loc,
                                                              demand_node.after.loc) + self.call_dis(
                                demand_node.prev.loc, demand_node.loc) + self.call_dis(demand_node.loc,
                                                                                       demand_node.after.loc)

                            demand_loc += 1
                            # 此处开始快进2
                            return_loc = -1
                            insert_node1_after_node_2(return_node, demand_node)
                            flag_3 = True
                            flag_2_1, _, _ = check_load_feasibility_from_node1_to_node2(_head, demand_node, 0, 0,
                                                                                        self.vehicle_capacity_limit[_r])

                            while flag_3 and flag_2_1 and self.vehicle_dis[_r] + add_dis <= self.vehicle_dis_limit[_r]:
                                return_loc += 1
                                add_dis = add_dis - self.call_dis(return_node.prev.loc,
                                                                  return_node.after.loc) + self.call_dis(
                                    return_node.prev.loc, return_node.loc) + self.call_dis(return_node.loc,
                                                                                           return_node.after.loc)
                                # 此处开始快进3
                                if check_load_feasibility_from_node1_to_node2(_head, self.route_tails[_r], 0, 0,
                                                                              self.vehicle_capacity_limit[_r])[0] and \
                                        self.vehicle_dis[_r] + add_dis <= self.vehicle_dis_limit[_r]:
                                    # 仅供测试
                                    # print(add_dis)
                                    # 仅供测试
                                    sol_found = True
                                    best_ans = self.compare_and_construct_best_ans(best_ans, add_dis,
                                                                                   supply_node.f_battery_q, _r,
                                                                                   supply_node,
                                                                                   demand_node, return_node, supply_loc,
                                                                                   demand_loc, return_loc)

                                    for _num in range(2, min(self.supply_counter[_supply],
                                                             self.demand_counter[_demand]) + 1):
                                        supply_node.f_battery_q, supply_node.e_battery_q = _num, 0
                                        demand_node.f_battery_q, demand_node.e_battery_q = -_num, _num
                                        return_node.f_battery_q, return_node.e_battery_q = 0, -_num
                                        if check_load_feasibility_from_node1_to_node2(_head, self.route_tails[_r], 0, 0,
                                                                                      self.vehicle_capacity_limit[_r])[
                                            0]:
                                            best_ans = self.compare_and_construct_best_ans(best_ans, add_dis,
                                                                                           supply_node.f_battery_q, _r,
                                                                                           supply_node,
                                                                                           demand_node, return_node,
                                                                                           supply_loc,
                                                                                           demand_loc, return_loc)

                                supply_node.f_battery_q, supply_node.e_battery_q = 1, 0
                                demand_node.f_battery_q, demand_node.e_battery_q = -1, 1
                                return_node.f_battery_q, return_node.e_battery_q = 0, -1
                                # 快进到此处3
                                # 仅供测试
                                # self.__record_current_routes()
                                # print(self.record_routes)
                                # 仅供测试
                                add_dis = add_dis + self.call_dis(return_node.prev.loc,
                                                                  return_node.after.loc) - self.call_dis(
                                    return_node.prev.loc, return_node.loc) - self.call_dis(return_node.loc,
                                                                                           return_node.after.loc)
                                flag_3 = move_node_to_tail(return_node, self.route_tails[_r])

                            remove_node_1(return_node)
                            # 快进到此处2
                            add_dis = add_dis + self.call_dis(demand_node.prev.loc,
                                                              demand_node.after.loc) - self.call_dis(
                                demand_node.prev.loc, demand_node.loc) - self.call_dis(demand_node.loc,
                                                                                       demand_node.after.loc)
                            flag_2 = move_node_to_tail(demand_node, self.route_tails[_r])

                        remove_node_1(demand_node)
                        # 快进到此处1

                        add_dis = add_dis + self.call_dis(supply_node.prev.loc,
                                                          supply_node.after.loc) - self.call_dis(
                            supply_node.prev.loc, supply_node.loc) - self.call_dis(supply_node.loc,
                                                                                   supply_node.after.loc)
                        flag_1 = move_node_to_tail(supply_node, self.route_tails[_r])

                    remove_node_1(supply_node)
        self.__record_current_routes()
        if sol_found:
            self.__implement_best_ans(best_ans)
        return sol_found

    def best_insertion_improved(self):

        sol_found = False
        best_ans = None

        supply_node = Node()
        demand_node = Node()
        return_node = Node()

        # 改为设计一个iterator
        for _r, _head in enumerate(self.route_heads):

            for _supply in self.supply_counter.keys():

                if self.supply_counter[_supply] == 0:
                    continue

                for _demand in self.demand_counter.keys():
                    if self.demand_counter[_demand] == 0:
                        continue

                    add_dis = 0

                    supply_node.loc = _supply
                    demand_node.loc = _demand
                    return_node.loc = _supply

                    supply_node.f_battery_q, supply_node.e_battery_q = 1, 0
                    demand_node.f_battery_q, demand_node.e_battery_q = -1, 1
                    return_node.f_battery_q, return_node.e_battery_q = 0, -1

                    supply_loc = -1
                    insert_node1_after_node_2(supply_node, _head)
                    flag_1 = True
                    flag_1_1, f_1, e_1 = check_load_feasibility_from_node1_to_node2(_head, supply_node, 0, 0,
                                                                                    self.vehicle_capacity_limit[
                                                                                        _r])
                    while flag_1:

                        add_dis = add_dis - self.call_dis(supply_node.prev.loc,
                                                          supply_node.after.loc) + self.call_dis(
                            supply_node.prev.loc, supply_node.loc) + self.call_dis(supply_node.loc,
                                                                                   supply_node.after.loc)

                        supply_loc += 1
                        # 此处开始快进1
                        if flag_1_1 and self.vehicle_dis[_r] + add_dis <= self.vehicle_dis_limit[_r] \
                                and (
                                supply_node.loc == supply_node.prev.loc and supply_node.get_type() == supply_node.prev.get_type()) is False:
                            insert_node1_after_node_2(demand_node, supply_node)
                            demand_loc = -1
                            flag_2 = True

                            while flag_2 and (
                                    supply_node.prev.loc == supply_node.after.loc and supply_node.prev.get_type() == supply_node.after.get_type()
                                    and supply_node.prev.loc != supply_node.loc and supply_node.prev.get_type() == supply_node.get_type()) is False:

                                flag_2_1, f_2, e_2 = check_load_feasibility_from_node1_to_node2(supply_node,
                                                                                                demand_node,
                                                                                                f_1,
                                                                                                e_1,
                                                                                                self.vehicle_capacity_limit[
                                                                                                    _r])
                                add_dis = add_dis - self.call_dis(demand_node.prev.loc,
                                                                  demand_node.after.loc) + self.call_dis(
                                    demand_node.prev.loc, demand_node.loc) + self.call_dis(demand_node.loc,
                                                                                           demand_node.after.loc)

                                demand_loc += 1
                                # 此处开始快进2
                                if flag_2_1 and self.vehicle_dis[_r] + add_dis <= self.vehicle_dis_limit[_r] and (
                                        demand_node.loc == demand_node.prev.loc and demand_node.get_type() == demand_node.prev.get_type()) is False:

                                    return_loc = -1
                                    insert_node1_after_node_2(return_node, demand_node)
                                    flag_3 = True

                                    while flag_3 and (
                                            demand_node.prev.loc == demand_node.after.loc and demand_node.prev.get_type() == demand_node.after.get_type()
                                            and demand_node.prev.loc != demand_node.loc and demand_node.prev.get_type() == demand_node.get_type()) is False:
                                        flag_3_1, f_3, e_3 = check_load_feasibility_from_node1_to_node2(demand_node,
                                                                                                        return_node,
                                                                                                        f_2,
                                                                                                        e_2,
                                                                                                        self.vehicle_capacity_limit[
                                                                                                            _r])
                                        return_loc += 1
                                        add_dis = add_dis - self.call_dis(return_node.prev.loc,
                                                                          return_node.after.loc) + self.call_dis(
                                            return_node.prev.loc, return_node.loc) + self.call_dis(return_node.loc,
                                                                                                   return_node.after.loc)
                                        # 此处开始快进3
                                        if ((
                                                    return_node.prev.loc == return_node.after.loc and return_node.prev.get_type() == return_node.after.get_type() and return_node.prev.loc != return_node.loc and return_node.prev.get_type() == return_node.get_type()) is False) and (
                                                (
                                                        return_node.loc == return_node.prev.loc and return_node.get_type() == return_node.prev.get_type()) is False):
                                            if flag_3_1 and self.vehicle_dis[_r] + add_dis <= self.vehicle_dis_limit[
                                                _r] and check_load_feasibility_from_node1_to_node2(return_node,
                                                                                                   self.route_tails[_r],
                                                                                                   f_3,
                                                                                                   e_3,
                                                                                                   self.vehicle_capacity_limit[
                                                                                                       _r])[0]:
                                                # 仅供测试
                                                # print(add_dis)
                                                # 仅供测试
                                                sol_found = True
                                                if \
                                                        check_load_feasibility_from_node1_to_node2(_head,
                                                                                                   self.route_tails[_r],
                                                                                                   0, 0,
                                                                                                   self.vehicle_capacity_limit[
                                                                                                       _r])[0] is False:
                                                    raise Exception("Infeasible Best Ans!")
                                                best_ans = self.compare_and_construct_best_ans(best_ans, add_dis,
                                                                                               supply_node.f_battery_q,
                                                                                               _r,
                                                                                               supply_node,
                                                                                               demand_node, return_node,
                                                                                               supply_loc,
                                                                                               demand_loc, return_loc)
                                                # 改为2分法,此处开始尝试增大更换电池数
                                                low = 2
                                                high = min(self.supply_counter[_supply], self.demand_counter[_demand])
                                                while low <= high:
                                                    _num = int((low + high) / 2)
                                                    supply_node.f_battery_q, supply_node.e_battery_q = _num, 0
                                                    demand_node.f_battery_q, demand_node.e_battery_q = -_num, _num
                                                    return_node.f_battery_q, return_node.e_battery_q = 0, -_num
                                                    flag_4 = False
                                                    if check_load_feasibility_from_node1_to_node2(supply_node.prev,
                                                                                                  self.route_tails[_r],
                                                                                                  f_1 - 1,
                                                                                                  e_1,
                                                                                                  self.vehicle_capacity_limit[
                                                                                                      _r])[0]:
                                                        flag_4 = True
                                                        if check_load_feasibility_from_node1_to_node2(_head,
                                                                                                      self.route_tails[
                                                                                                          _r], 0, 0,
                                                                                                      self.vehicle_capacity_limit[
                                                                                                          _r])[
                                                            0] is False:
                                                            raise Exception("Infeasible Best Ans!")
                                                        best_ans = self.compare_and_construct_best_ans(best_ans,
                                                                                                       add_dis,
                                                                                                       supply_node.f_battery_q,
                                                                                                       _r,
                                                                                                       supply_node,
                                                                                                       demand_node,
                                                                                                       return_node,
                                                                                                       supply_loc,
                                                                                                       demand_loc,
                                                                                                       return_loc)
                                                    if flag_4:
                                                        low = _num + 1
                                                    else:
                                                        high = _num - 1
                                            supply_node.f_battery_q, supply_node.e_battery_q = 1, 0
                                            demand_node.f_battery_q, demand_node.e_battery_q = -1, 1
                                            return_node.f_battery_q, return_node.e_battery_q = 0, -1
                                        # 快进到此处3
                                        # 仅供测试
                                        # self.__record_current_routes()
                                        # print(self.record_routes)
                                        # 仅供测试
                                        add_dis = add_dis + self.call_dis(return_node.prev.loc,
                                                                          return_node.after.loc) - self.call_dis(
                                            return_node.prev.loc, return_node.loc) - self.call_dis(return_node.loc,
                                                                                                   return_node.after.loc)
                                        flag_3 = move_node_to_tail(return_node, self.route_tails[_r])

                                    remove_node_1(return_node)
                                # 快进到此处2
                                add_dis = add_dis + self.call_dis(demand_node.prev.loc,
                                                                  demand_node.after.loc) - self.call_dis(
                                    demand_node.prev.loc, demand_node.loc) - self.call_dis(demand_node.loc,
                                                                                           demand_node.after.loc)
                                flag_2 = move_node_to_tail(demand_node, self.route_tails[_r])

                            remove_node_1(demand_node)
                        # 快进到此处1

                        add_dis = add_dis + self.call_dis(supply_node.prev.loc,
                                                          supply_node.after.loc) - self.call_dis(
                            supply_node.prev.loc, supply_node.loc) - self.call_dis(supply_node.loc,
                                                                                   supply_node.after.loc)
                        flag_1 = move_node_to_tail(supply_node, self.route_tails[_r])
                        flag_1_1, f_1, e_1 = check_load_feasibility_from_node1_to_node2(_head, supply_node, 0, 0,
                                                                                        self.vehicle_capacity_limit[_r])

                    remove_node_1(supply_node)

                    if add_dis != 0 or demand_node.f_battery_q != -1 or demand_node.e_battery_q != 1 or demand_node.loc != _demand:
                        print('error')

                    supply_node.loc = _supply
                    supply_node.f_battery_q, supply_node.e_battery_q = 1, -1
                    supply_loc = -1
                    insert_node1_after_node_2(supply_node, _head)
                    flag_1 = True

                    flag_1_1, f_1, e_1 = check_load_feasibility_from_node1_to_node2(_head, supply_node, 0, 0,
                                                                                    self.vehicle_capacity_limit[
                                                                                        _r])
                    while flag_1:

                        add_dis = add_dis - self.call_dis(supply_node.prev.loc,
                                                          supply_node.after.loc) + self.call_dis(
                            supply_node.prev.loc, supply_node.loc) + self.call_dis(supply_node.loc,
                                                                                   supply_node.after.loc)

                        supply_loc += 1
                        # 此处开始快进1
                        if flag_1_1 and self.vehicle_dis[_r] + add_dis <= self.vehicle_dis_limit[_r] \
                                and (
                                supply_node.loc == supply_node.prev.loc and supply_node.get_type() == supply_node.prev.get_type()) is False:
                            insert_node1_after_node_2(demand_node, supply_node)
                            demand_loc = -1
                            flag_2 = True

                            while flag_2 and (
                                    supply_node.prev.loc == supply_node.after.loc and supply_node.prev.get_type() == supply_node.after.get_type()
                                    and supply_node.prev.loc != supply_node.loc and supply_node.prev.get_type() == supply_node.get_type()) is False:

                                flag_2_1, f_2, e_2 = check_load_feasibility_from_node1_to_node2(supply_node,
                                                                                                demand_node,
                                                                                                f_1,
                                                                                                e_1,
                                                                                                self.vehicle_capacity_limit[
                                                                                                    _r])
                                add_dis = add_dis - self.call_dis(demand_node.prev.loc,
                                                                  demand_node.after.loc) + self.call_dis(
                                    demand_node.prev.loc, demand_node.loc) + self.call_dis(demand_node.loc,
                                                                                           demand_node.after.loc)

                                demand_loc += 1
                                # 此处开始快进2
                                if flag_2_1 and self.vehicle_dis[_r] + add_dis <= self.vehicle_dis_limit[_r] and (
                                        demand_node.loc == demand_node.prev.loc and demand_node.get_type() == demand_node.prev.get_type()) is False and (
                                        demand_node.prev.loc == demand_node.after.loc and demand_node.prev.get_type() == demand_node.after.get_type()
                                        and demand_node.prev.loc != demand_node.loc and demand_node.prev.get_type() == demand_node.get_type()) is False and \
                                        check_load_feasibility_from_node1_to_node2(demand_node,
                                                                                   self.route_tails[_r],
                                                                                   f_2,
                                                                                   e_2,
                                                                                   self.vehicle_capacity_limit[
                                                                                       _r])[0]:

                                    # 仅供测试
                                    # print(add_dis)
                                    # 仅供测试
                                    sol_found = True
                                    if check_load_feasibility_from_node1_to_node2(_head, self.route_tails[_r], 0, 0,
                                                                                  self.vehicle_capacity_limit[_r])[
                                        0] is False:
                                        raise Exception("Infeasible Best Ans!")
                                    best_ans = self.compare_and_construct_best_ans(best_ans, add_dis,
                                                                                   supply_node.f_battery_q,
                                                                                   _r,
                                                                                   supply_node,
                                                                                   demand_node, None,
                                                                                   supply_loc,
                                                                                   demand_loc, None)
                                    # 改为2分法,此处开始尝试增大更换电池数
                                    low = 2
                                    high = min(self.supply_counter[_supply], self.demand_counter[_demand])
                                    while low <= high:
                                        _num = int((low + high) / 2)
                                        supply_node.f_battery_q, supply_node.e_battery_q = _num, -_num
                                        demand_node.f_battery_q, demand_node.e_battery_q = -_num, _num
                                        flag_4 = False
                                        if check_load_feasibility_from_node1_to_node2(supply_node.prev,
                                                                                      self.route_tails[_r],
                                                                                      f_1 - 1,
                                                                                      e_1 + 1,
                                                                                      self.vehicle_capacity_limit[
                                                                                          _r])[0]:
                                            flag_4 = True
                                            if check_load_feasibility_from_node1_to_node2(_head, self.route_tails[_r],
                                                                                          0, 0,
                                                                                          self.vehicle_capacity_limit[
                                                                                              _r])[0] is False:
                                                raise Exception("Infeasible Best Ans!")
                                            best_ans = self.compare_and_construct_best_ans(best_ans,
                                                                                           add_dis,
                                                                                           supply_node.f_battery_q,
                                                                                           _r,
                                                                                           supply_node,
                                                                                           demand_node,
                                                                                           None,
                                                                                           supply_loc,
                                                                                           demand_loc,
                                                                                           None)
                                        if flag_4:
                                            low = _num + 1
                                        else:
                                            high = _num - 1
                                supply_node.f_battery_q, supply_node.e_battery_q = 1, -1
                                demand_node.f_battery_q, demand_node.e_battery_q = -1, 1
                                # 快进到此处2
                                add_dis = add_dis + self.call_dis(demand_node.prev.loc,
                                                                  demand_node.after.loc) - self.call_dis(
                                    demand_node.prev.loc, demand_node.loc) - self.call_dis(demand_node.loc,
                                                                                           demand_node.after.loc)
                                flag_2 = move_node_to_tail(demand_node, self.route_tails[_r])

                            remove_node_1(demand_node)
                        # 快进到此处1

                        add_dis = add_dis + self.call_dis(supply_node.prev.loc,
                                                          supply_node.after.loc) - self.call_dis(
                            supply_node.prev.loc, supply_node.loc) - self.call_dis(supply_node.loc,
                                                                                   supply_node.after.loc)
                        flag_1 = move_node_to_tail(supply_node, self.route_tails[_r])
                        flag_1_1, f_1, e_1 = check_load_feasibility_from_node1_to_node2(_head, supply_node, 0, 0,
                                                                                        self.vehicle_capacity_limit[_r])

                    remove_node_1(supply_node)

        # self.__record_current_routes()
        # print(self.record_routes)
        if sol_found:
            self.__implement_best_ans_2(best_ans)
        return sol_found

    def __implement_best_ans(self, best_ans):
        self.vehicle_dis[best_ans[2]] += best_ans[0]
        self.total_dis += best_ans[0]
        _head = self.route_heads[best_ans[2]]
        supply_node = Node(best_ans[3][0][0], best_ans[3][0][1], best_ans[3][0][2])
        demand_node = Node(best_ans[4][0][0], best_ans[4][0][1], best_ans[4][0][2])
        return_node = Node(best_ans[5][0][0], best_ans[5][0][1], best_ans[5][0][2])
        supply_node.related_nodes = [demand_node, return_node]
        demand_node.related_nodes = [supply_node, return_node]
        return_node.related_nodes = [supply_node, demand_node]
        self.supply_counter[supply_node.loc] -= best_ans[1]
        self.demand_counter[demand_node.loc] -= best_ans[1]
        insert_node1_after_node_2(supply_node, _head, best_ans[3][1])
        insert_node1_after_node_2(demand_node, supply_node, best_ans[4][1])
        insert_node1_after_node_2(return_node, demand_node, best_ans[5][1])
        self.__record_current_routes()
        self.__record_used_vehicle_num()
        # print(self.record_routes)
        # self.print_route_load()
        return

    def __implement_best_ans_2(self, best_ans):
        if len(best_ans) == 6:
            self.__implement_best_ans(best_ans)
        elif len(best_ans) == 5:
            self.vehicle_dis[best_ans[2]] += best_ans[0]
            self.total_dis += best_ans[0]
            _head = self.route_heads[best_ans[2]]
            middle_node = Node(best_ans[3][0][0], best_ans[3][0][1], best_ans[3][0][2])
            demand_node = Node(best_ans[4][0][0], best_ans[4][0][1], best_ans[4][0][2])
            middle_node.related_nodes = [demand_node]
            demand_node.related_nodes = [middle_node]
            self.supply_counter[middle_node.loc] -= best_ans[1]
            self.demand_counter[demand_node.loc] -= best_ans[1]
            insert_node1_after_node_2(middle_node, _head, best_ans[3][1])
            insert_node1_after_node_2(demand_node, middle_node, best_ans[4][1])
            self.__record_current_routes()
            self.__record_used_vehicle_num()
            # print(self.record_routes)
            # self.print_route_load()
        return

    @classmethod
    def construct_best_ans(cls, add_dis, _num, r, supply_node, demand_node, return_node, supply_loc, demand_loc,
                           return_loc):
        best_ans = list()

        best_ans.append(add_dis)
        best_ans.append(_num)

        best_ans.append(r)
        best_ans.append([(supply_node.loc, supply_node.f_battery_q, supply_node.e_battery_q), supply_loc])
        best_ans.append([(demand_node.loc, demand_node.f_battery_q, demand_node.e_battery_q), demand_loc])
        if return_node is not None:
            best_ans.append([(return_node.loc, return_node.f_battery_q, return_node.e_battery_q), return_loc])
        return best_ans

    @classmethod
    def compare_and_construct_best_ans(cls, pre_best_ans, add_dis, _num, r, supply_node, demand_node, return_node,
                                       supply_loc, demand_loc, return_loc):
        if pre_best_ans is None:
            return cls.construct_best_ans(add_dis, _num, r, supply_node, demand_node, return_node,
                                          supply_loc, demand_loc, return_loc)
        if add_dis > pre_best_ans[0]:
            return pre_best_ans
        elif add_dis < pre_best_ans[0]:
            return cls.construct_best_ans(add_dis, _num, r, supply_node, demand_node, return_node,
                                          supply_loc, demand_loc, return_loc)
        else:
            if _num > pre_best_ans[1]:
                return cls.construct_best_ans(add_dis, _num, r, supply_node, demand_node, return_node,
                                              supply_loc, demand_loc, return_loc)
            else:
                return pre_best_ans

    def print_route_load(self):
        for _r, _head in enumerate(self.route_heads):
            print('start=>')
            _f = 0
            _e = 0
            _node = _head
            while _node != self.route_tails[_r]:
                _f += _node.f_battery_q
                _e += _node.e_battery_q
                print('%d(%d,%d)[%d,%d]=>' % (_node.loc, _node.f_battery_q, _node.e_battery_q, _f, _e))
                _node = _node.after
            print('end')

    def remove_supplys_and_related_demands(self, _supplys):
        _nodes2remove = set()
        for _r, _head in enumerate(self.route_heads):
            _node = _head
            while _node != self.route_tails[_r]:
                if _node.loc in _supplys and _node.get_type() in {4, 5}:
                    _nodes2remove.add(_node)
                    _nodes2remove.update(set(_node.related_nodes))
                _node = _node.after
        for _node in _nodes2remove:
            _node.related_nodes = None
            if _node.get_type() in {4, 5}:
                self.supply_counter[_node.loc] += _node.f_battery_q
            if _node.get_type() == -4:
                self.demand_counter[_node.loc] += _node.e_battery_q
            remove_node_1(_node)
        # 需要修改total_dis, used_vehicle_num, vehicle_dis, vehicle_cost
        self.__record_current_routes()
        self.__record_used_vehicle_num()
        self.__recalculate_dis()
        return

    def remove_middles_and_related_nodes(self):
        _nodes2remove = set()
        for _r, _head in enumerate(self.route_heads):
            flag, _, _ = check_load_feasibility_from_node1_to_node2(_head, self.route_tails[_r], 0, 0,
                                                                    self.vehicle_capacity_limit[_r])
            if flag is False:
                _node = _head
                while _node != self.route_tails[_r]:
                    if _node.get_type() == 4:
                        _nodes2remove.add(_node)
                        _nodes2remove.update(set(_node.related_nodes))
                    _node = _node.after
        for _node in _nodes2remove:
            _node.related_nodes = None
            if _node.get_type() in {4, 5}:
                self.supply_counter[_node.loc] += _node.f_battery_q
            if _node.get_type() == -4:
                self.demand_counter[_node.loc] += _node.e_battery_q
            remove_node_1(_node)
        # 需要修改total_dis, used_vehicle_num, vehicle_dis, vehicle_cost
        self.__record_current_routes()
        self.__record_used_vehicle_num()
        self.__recalculate_dis()
        return


if __name__ == '__main__':
    _vrp_instance = VRP()
    _vrp_instance.init_vehicle_routes()
    _vrp_instance.create_initial_solution_improved()
    # _vrp_instance.print_route_load()
    print(_vrp_instance.total_dis)
    print(_vrp_instance.record_routes)
    _vrp_instance.remove_supplys_and_related_demands({2})
    _vrp_instance.remove_middles_and_related_nodes()
    print(_vrp_instance.total_dis)
    print(_vrp_instance.record_routes)
    _vrp_instance.create_initial_solution_improved()
    print(_vrp_instance.total_dis)
    print(_vrp_instance.record_routes)
    # _vrp_instance.print_route_load()
    print('finished')
