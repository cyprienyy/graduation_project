from collections import Counter
import numpy as np

dis_matrix = np.zeros((1, 1))


class Node:
    def __init__(self, loc=0, f_battery_q=0, e_battery_q=0, related_nodes=None, prev=None, after=None):
        self.loc = loc
        self.f_battery_q = f_battery_q
        self.e_battery_q = e_battery_q
        self.related_nodes = related_nodes
        self.prev = prev
        self.after = after


def insert_node1_after_node_2(node1, node2, k=0):
    node_k = node2
    for _ in range(k):
        node_k = node2.after
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


def check_load_feasibility_from_node1_to_node2(node_1, node_2, start_f, start_e, vehicle_capacity):
    if start_f < 0 or start_e < 0 or start_f + start_e > vehicle_capacity:
        return False
    _start_f = start_f
    _start_e = start_e
    node = node_1
    while node != node_2:
        node = node.after
        start_f += node.f_battery_q
        start_e += node.e_battery_q
        if start_f < 0 or start_e < 0 or start_f + start_e > vehicle_capacity:
            return False
    return True


class VRP:
    def __int__(self):

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

        self.demand_counter = Counter()
        self.supply_counter = Counter()

        return

    def init_vehicle_routes(self):
        for _ in range(self.vehicle_num_limit):
            _head, _tail = create_empty_path()
            self.route_heads.append(_head)
            self.route_tails.append(_tail)
        return

    def __record_used_vehicle_num(self):
        self.used_vehicle_num = 0
        for i, _head in enumerate(self.route_heads):
            if _head.after != self.route_tails[i]:
                self.used_vehicle_num += 1
        self.vehicle_cost = self.single_vehicle_cost * self.used_vehicle_num
        return

    @staticmethod
    def call_dis(loc_1, loc_2):
        return dis_matrix[loc_1, loc_2]

    def __record_current_routes(self):
        self.record_routes = []
        for i, _head in enumerate(self.route_heads):
            _node = _head
            self.record_routes.append([_node.loc])
            while _node != self.route_tails[i]:
                _node = _node.after
                self.record_routes[-1].append(_node.loc)

    # 可以通过保存每个节点在上一个关键节点后面第几步和节点具体信息来保存最优路径。
    def create_initial_solution(self):

        sol_found = False
        best_ans = None

        supply_node = Node()
        demand_node = Node()
        return_node = Node()

        for i, _head in self.route_heads:

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
                    return_node.f_battery_q, supply_node.e_battery_q = 0, -1

                    supply_loc = -1
                    insert_node1_after_node_2(supply_node, _head)
                    flag_1 = True

                    while flag_1:

                        add_dis = add_dis - self.call_dis(supply_node.prev.loc,
                                                          supply_node.after.loc) + self.call_dis(
                            supply_node.prev.loc, supply_node.loc) + self.call_dis(supply_node.loc,
                                                                                   supply_node.after.loc)

                        supply_loc += 1
                        insert_node1_after_node_2(demand_node, supply_node)
                        demand_loc = -1
                        flag_2 = True

                        flag_1_1 = check_load_feasibility_from_node1_to_node2(_head, supply_node, 0, 0,
                                                                              self.vehicle_capacity_limit[i])
                        while flag_2 and flag_1_1:

                            add_dis = add_dis - self.call_dis(demand_node.prev.loc,
                                                              demand_node.after.loc) + self.call_dis(
                                demand_node.prev.loc, demand_node.loc) + self.call_dis(demand_node.loc,
                                                                                       demand_node.after.loc)

                            demand_loc += 1
                            return_loc = -1
                            insert_node1_after_node_2(return_node, demand_node)
                            flag_3 = True
                            flag_2_1 = check_load_feasibility_from_node1_to_node2(_head, demand_node, 0, 0,
                                                                                  self.vehicle_capacity_limit[i])

                            while flag_3 and flag_2_1:
                                add_dis = add_dis - self.call_dis(return_node.prev.loc,
                                                                  return_node.after.loc) + self.call_dis(
                                    return_node.prev.loc, return_node.loc) + self.call_dis(return_node.loc,
                                                                                           return_node.after.loc)

                                return_loc += 1
                                # 需要做的更多
                                supply_node.f_battery_q, supply_node.e_battery_q = 1, 0
                                demand_node.f_battery_q, demand_node.e_battery_q = -1, 1
                                return_node.f_battery_q, supply_node.e_battery_q = 0, -1
                                flag_3 = move_node_to_tail(return_node, self.route_tails[i])

                                add_dis = add_dis + self.call_dis(return_node.prev.loc,
                                                                  return_node.after.loc) - self.call_dis(
                                    return_node.prev.loc, return_node.loc) - self.call_dis(return_node.loc,
                                                                                           return_node.after.loc)

                            remove_node_1(return_node)

                            flag_2 = move_node_to_tail(demand_node, self.route_tails[i])
                            add_dis = add_dis + self.call_dis(demand_node.prev.loc,
                                                              demand_node.after.loc) - self.call_dis(
                                demand_node.prev.loc, demand_node.loc) - self.call_dis(demand_node.loc,
                                                                                       demand_node.after.loc)

                        remove_node_1(demand_node)

                        flag_1 = move_node_to_tail(supply_node, self.route_tails[i])

                        add_dis = add_dis + self.call_dis(supply_node.prev.loc,
                                                          supply_node.after.loc) - self.call_dis(
                            supply_node.prev.loc, supply_node.loc) - self.call_dis(supply_node.loc,
                                                                                   supply_node.after.loc)

                    remove_node_1(supply_node)
        self.__record_current_routes()
        return

    @classmethod
    def construct_best_ans(cls, add_dis, num, r, supply_node, demand_node, return_node, supply_loc, demand_loc,
                           return_loc):
        best_ans = list()

        best_ans.append(add_dis)
        best_ans.append(num)

        best_ans.append(r)
        best_ans.append([(supply_node.loc, supply_node.f_battery_q, supply_node.e_battery_q), supply_loc])
        best_ans.append([(demand_node.loc, demand_node.f_battery_q, demand_node.e_battery_q), demand_loc])
        best_ans.append([(return_node.loc, return_node.f_battery_q, return_node.e_battery_q), return_loc])
        return best_ans

    @classmethod
    def compare_and_construct_best_ans(cls, pre_best_ans, add_dis, num, r, supply_node, demand_node, return_node,
                                       supply_loc, demand_loc, return_loc):
        if pre_best_ans is None:
            return cls.construct_best_ans(add_dis, num, r, supply_node, demand_node, return_node,
                                          supply_loc, demand_loc, return_loc)
        if add_dis > pre_best_ans[0]:
            return pre_best_ans
        elif add_dis < pre_best_ans[0]:
            return cls.construct_best_ans(add_dis, num, r, supply_node, demand_node, return_node,
                                          supply_loc, demand_loc, return_loc)
        else:
            if num > pre_best_ans[0]:
                return cls.construct_best_ans(add_dis, num, r, supply_node, demand_node, return_node,
                                              supply_loc, demand_loc, return_loc)
            else:
                return pre_best_ans
