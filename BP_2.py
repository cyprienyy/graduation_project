import gurobipy as gp
from gurobipy import GRB
import numpy as np
from Data import supplys, supply_count, demands, demand_count, vehicle_capacity, distance_graph, H
from BP_2_data import nodes_cnt, supply_nodes, demand_nodes, return_nodes, nodes_rel, f_node, e_node, supply_num, \
    demand_num, lis_i_j, return_supply, count
from collections import deque, Counter, defaultdict


class label_set:
    def __init__(self):
        self.deque = deque()

    def add_label(self, label, compare_func):
        flag = True
        _new_deque = deque()
        while self.deque:
            old_label = self.deque.pop()
            res = compare_func(old_label, label)
            if res == -1:
                flag = False
                label.dominated = True
                _new_deque.append(old_label)
            elif res == 0:
                _new_deque.append(old_label)
            elif res == 1:
                old_label.dominated = True
        if flag is True:
            _new_deque.append(label)
        self.deque = _new_deque

    def get_best(self):
        if len(self.deque) == 0:
            raise Exception('没有找到任何列')
        res = float('inf')
        _best = None
        for _label in self.deque:
            if _label.cost < res:
                _best = _label
                res = _label.cost
        return _best

    def print_paths(self):
        for _label in self.deque:
            print(_label.cost, _label.path)

    def get_best_2(self):
        if len(self.deque) == 0:
            raise Exception('没有找到任何列')
        res = float('inf')
        _best = None
        for _label in self.deque:
            if _label.cost < res:
                _best = _label
                res = _label.cost
        array = []
        for j in supplys + demands:
            array.append(_best.count[j])
        return _best.cost, _best.time, array

    def clear(self):
        self.deque = deque()


class MainProblem:
    def __init__(self):

        self.n_r = np.ones((len(supplys) + len(demands), 2))
        self.c_r = np.zeros(self.n_r.shape[1])
        self.n_r[:, 0] = np.array([1, 0, 1, 0])
        self.n_r[:, 1] = np.array([0, 1, 0, 1])
        self.c_r[0] = 446
        self.c_r[1] = 484
        self.paths = [[], []]

        self.MainProbRelax = gp.Model()  # 松弛后的列生成主问题

        # 构造主问题模型
        # 添加变量
        self.X_r = self.MainProbRelax.addVars(self.n_r.shape[1], lb=0.0, ub=1.0, obj=self.c_r, vtype=GRB.CONTINUOUS,
                                              name='X_r')
        # 添加约束
        i = 0
        for j in supplys:
            self.MainProbRelax.addConstr(
                gp.quicksum(self.n_r[i, r] * self.X_r[r] for r in range(self.n_r.shape[1])) <= supply_count[j])
            i = i + 1
        for j in demands:
            self.MainProbRelax.addConstr(
                gp.quicksum(self.n_r[i, r] * self.X_r[r] for r in range(self.n_r.shape[1])) == demand_count[j])
            i = i + 1
        self.MainProbRelax.setAttr(GRB.Attr.ModelSense, GRB.MINIMIZE)

    def get_dual_solution(self):
        _Dualsolution = self.MainProbRelax.getAttr("Pi", self.MainProbRelax.getConstrs())

        Dualsolution = [0]
        i = 0
        for j in supplys:
            Dualsolution = Dualsolution + [_Dualsolution[i] / supply_count[j]] * supply_count[j]
            i = i + 1
        for j in demands:
            Dualsolution = Dualsolution + [_Dualsolution[i] / demand_count[j]] * demand_count[j]
            i = i + 1
        return Dualsolution

    def optimize(self):
        self.MainProbRelax.optimize()

    def add_column(self, columnCoeff, columnVal, columnPath):
        column = gp.Column(columnCoeff, self.MainProbRelax.getConstrs())
        self.MainProbRelax.addVar(obj=columnVal, lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name="CG", column=column)
        self.n_r = np.column_stack((self.n_r, columnCoeff))
        self.c_r = np.append(self.c_r, columnVal)
        self.paths.append(columnPath)
        return


class SubProblem:
    def __init__(self):
        self.sub_problem = gp.Model()

        # 添加变量
        self.x_ij = self.sub_problem.addVars(lis_i_j, vtype=GRB.BINARY, name='x_ij')
        self.y_i = self.sub_problem.addVars(nodes_cnt, lb=0.0, ub=vehicle_capacity, vtype=GRB.CONTINUOUS, name='y_i')
        self.z_i = self.sub_problem.addVars(nodes_cnt, lb=0.0, ub=vehicle_capacity, vtype=GRB.CONTINUOUS, name='z_i')
        self.t_i = self.sub_problem.addVars(nodes_cnt, lb=0.0, vtype=GRB.CONTINUOUS, name='t_i')

        # 添加约束
        self.sub_problem.addConstr(self.x_ij.sum(0, '*') == 1)
        self.sub_problem.addConstr(self.x_ij.sum('*', count) == 1)
        self.sub_problem.addConstrs(
            self.x_ij.sum('*', i) == self.x_ij.sum(i, '*') for i in supply_nodes + demand_nodes + return_nodes)
        self.sub_problem.addConstrs(self.x_ij.sum(i, '*') == self.x_ij.sum(return_supply[i], '*') for i in return_nodes)
        self.sub_problem.addConstr(
            gp.quicksum(self.x_ij[i, j] * distance_graph[nodes_rel[i], nodes_rel[j]] for i, j in lis_i_j) <= H)
        self.sub_problem.addConstrs(
            self.y_i[i] + f_node[j] <= self.y_i[j] + vehicle_capacity * (1 - self.x_ij[i, j]) for i, j in lis_i_j)
        self.sub_problem.addConstrs(
            self.y_i[j] - f_node[j] <= self.y_i[i] + vehicle_capacity * (1 - self.x_ij[i, j]) for i, j in lis_i_j)
        self.sub_problem.addConstrs(
            self.z_i[i] + e_node[j] <= self.z_i[j] + vehicle_capacity * (1 - self.x_ij[i, j]) for i, j in lis_i_j)
        self.sub_problem.addConstrs(
            self.z_i[j] - e_node[j] <= self.z_i[i] + vehicle_capacity * (1 - self.x_ij[i, j]) for i, j in lis_i_j)
        self.sub_problem.addConstrs(self.y_i[i] + self.z_i[i] <= vehicle_capacity for i in range(nodes_cnt))
        self.sub_problem.addConstrs(self.y_i[i] + self.z_i[i] == 0 for i in [0, count])
        self.sub_problem.addConstrs(
            self.t_i[i] + 1 <= self.t_i[j] + (nodes_cnt + 1) * (1 - self.x_ij[i, j]) for i, j in lis_i_j)
        self.sub_problem.addConstrs(self.t_i[i] >= self.t_i[return_supply[i]] + 1 for i in return_nodes)
        self.sub_problem.addConstrs(self.z_i[i] == 0 for i in supply_nodes)
        self.sub_problem.addConstrs(self.y_i[i] == 0 for i in return_nodes)
        self.sub_problem.setAttr(GRB.Attr.ModelSense, GRB.MINIMIZE)
        self.sub_problem.update()

    def adjust_obj_coeff(self, Dualsolution):
        for i, j in lis_i_j:
            if 1 <= j <= supply_num + demand_num:
                self.x_ij[i, j].obj = distance_graph[nodes_rel[i], nodes_rel[j]] - Dualsolution[j - 1]
            else:
                self.x_ij[i, j].obj = distance_graph[nodes_rel[i], nodes_rel[j]]
        self.sub_problem.update()
        return

    def optimize(self):
        self.sub_problem.optimize()
        # for v in self.sub_problem.getVars():
        # print('%s %g' % (v.VarName, v.X))


class label:
    def __init__(self, nodes_cnt, place):
        self.cost = 0
        self.time = 0
        self.f = 0
        self.e = 0
        self.visited = [0] * nodes_cnt
        self.visited[0] = 1
        self.dominated = False
        self.place = place
        self.count = Counter()
        self.path = [0]

    def extend(self, cost_add, time_add, f_add, e_add, des, des_point, des_type):
        _new_label = label(len(self.visited), des)
        _new_label.cost = self.cost + cost_add
        _new_label.time = self.time + time_add
        _new_label.f = self.f + f_add
        _new_label.e = self.e + e_add
        _new_label.visited = self.visited.copy()
        _new_label.visited[des] = 1
        _new_label.count = self.count.copy()
        if des_type == 5:
            _new_label.count[des_point] += 1
        elif des_type == 1:
            _new_label.count[des_point] -= 1
        if _new_label.count[des_point] >= 0 and _new_label.time <= H and _new_label.f >= 0 and _new_label.e >= 0 and (
                _new_label.f + _new_label.e) <= vehicle_capacity and not (
                des_type == 0 and (_new_label.f != 0 or _new_label.e != 0)):
            _new_label.path = self.path + [des]
            return _new_label
        else:
            return None


class OneLevel:
    def __init__(self):
        self.sub_problem = None
        self.UL = deque()
        self.TL = list()
        for _ in range(nodes_cnt):
            a = label_set()
            self.TL.append(a)

    @staticmethod
    def compare_label1_label2(label1, label2):
        # 0即无关系，-1代表label1 dominates，1代表label2 dominates
        if label1.f != label2.f:
            return 0
        for i in supplys:
            if (label1.count[i] > 0 and label2.count[i] == 0) or (label1.count[i] == 0 and label2.count[i] > 0):
                return 0
        label_1_flag = True
        label_2_flag = True
        for i, visited in enumerate(label1.visited):
            if visited > label2.visited[i]:
                label_1_flag = False
            if visited < label2.visited[i]:
                label_2_flag = False
        if label_1_flag is True and label1.cost <= label2.cost and label1.time <= label2.time:
            return -1
        if label_2_flag is True and label2.cost <= label1.cost and label2.time <= label1.time:
            return 1
        return 0

    def label_setting(self):
        # 成本，时间，满电电池数，缺电电池数，拜访情况列表, dominates
        label_0 = label(nodes_cnt, 0)

        self.UL.append(label_0)
        self.TL[0].add_label(label_0, self.compare_label1_label2)

        while self.UL:
            _label = self.UL.pop()
            if _label.dominated is False and _label.place != count:
                # print('开始拓展')
                i = _label.place

                _prev_extend_point = None
                _prev_extend_point_type = None

                for j, to_visit in enumerate(_label.visited):
                    if to_visit == 0 and (nodes_rel[j] != _prev_extend_point or (
                            5 * f_node[j] - e_node[j]) != _prev_extend_point_type):
                        _prev_extend_point = nodes_rel[j]
                        _prev_extend_point_type = 5 * f_node[j] - e_node[j]
                        if i != count and j != 0 and i != j and not (
                                i == 0 and j in demand_nodes + return_nodes) and not (
                                1 <= i <= (supply_num + demand_num) and j == count) and not (
                                (count > i > (supply_num + demand_num) and return_supply[i] == j) or (
                                count > j > (supply_num + demand_num) and return_supply[j] == i)) and not (
                                1 <= j <= supply_num and _label.e != 0) and not (
                                j > supply_num + demand_num and _label.f != 0):
                            _new_label = _label.extend(self.sub_problem.x_ij[i, j].obj,
                                                       distance_graph[nodes_rel[i], nodes_rel[j]],
                                                       f_node[j], e_node[j],
                                                       j, nodes_rel[j], 5 * f_node[j] - e_node[j])
                            if _new_label is not None:
                                self.UL.append(_new_label)
                                self.TL[j].add_label(_new_label, self.compare_label1_label2)

    def return_result(self):
        _label = self.TL[-1].get_best()
        res = Counter()
        _res = []
        for i in _label.path:
            if 5 * f_node[i] - e_node[i] in (5, -6):
                res[nodes_rel[i]] += 1
        for i in supplys + demands:
            _res.append(res[i])
        return _label.cost, _label.time, _res

    def clear(self):
        self.UL = deque()
        for i in range(nodes_cnt):
            self.TL[i].clear()


class label_1():
    def __init__(self, nodes_cnt, place):
        self.cost = 0
        self.time = 0
        self.f = 0
        self.e = 0
        self.visited = [0] * nodes_cnt
        self.visited[0] = 1
        self.dominated = False
        self.place = place
        self.current_type = 0
        self.supply_demand_count = Counter()
        self.count = Counter()
        self.visited_supply = []
        self.visited_demand = []
        self.visited_return = []
        self.path = [0]

    def extend(self, cost_add, time_add, f_add, e_add, des, des_point, des_type):
        _new_label = label_1(len(self.visited), des)

        _new_label.current_type = des_type
        if (self.current_type, _new_label.current_type) in ((-6, 5), (1, -6), (1, 5)):
            return None

        _new_label.visited_supply = self.visited_supply.copy()
        _new_label.visited_demand = self.visited_demand.copy()
        _new_label.visited_return = self.visited_return.copy()
        _array = None
        if des_type == 5:
            _array = _new_label.visited_supply
        elif des_type == 1:
            _array = _new_label.visited_return
        elif des_type == -6:
            _array = _new_label.visited_demand
        if des_point in _array:
            if des_point != _array[-1]:
                return None
        else:
            _array.append(des_point)

        _new_label.cost = self.cost + cost_add
        _new_label.time = self.time + time_add
        _new_label.f = self.f + f_add
        _new_label.e = self.e + e_add
        _new_label.visited = self.visited.copy()
        _new_label.visited[des] = 1
        _new_label.count = self.count.copy()
        _new_label.supply_demand_count = self.supply_demand_count.copy()
        _new_label.path = self.path + [des]

        if des_type == 5:
            _new_label.count[des_point] += 1
            _new_label.supply_demand_count[des_point] += 1
        elif des_type == 1:
            _new_label.count[des_point] -= 1
        elif des_type == -6:
            _new_label.supply_demand_count[des_point] += 1
        if _new_label.count[des_point] >= 0 and _new_label.time <= H and _new_label.f >= 0 and _new_label.e >= 0 and (
                _new_label.f + _new_label.e) <= vehicle_capacity and not (
                des_type == 0 and (_new_label.f != 0 or _new_label.e != 0)):
            return _new_label
        else:
            return None


class TwoLevelColumnGeneration:
    def __int__(self):
        self.sub_problem = None
        self.UL = deque()
        self.TL = list()
        for _ in range(nodes_cnt):
            a = label_set()
            self.TL.append(a)
        return

    @staticmethod
    def compare_label1_label2_1(label1, label2):
        # 判断dominates的条件需要考量
        label_1_flag = True
        label_2_flag = True
        if label1.f != label2.f or label1.visited_supply[0] != label2.visited_supply[0]:
            return 0
        for i in supplys:
            if (label1.count[i] > 0 and label2.count[i] == 0) or (label1.count[i] == 0 and label2.count[i] > 0):
                return 0
        for i in supplys + demands:
            if label1.supply_demand_count[i] > label2.supply_demand_count[i]:
                label_1_flag = False
            if label1.supply_demand_count[i] < label2.supply_demand_count[i]:
                label_2_flag = False
        if label_1_flag is True and label1.cost <= label2.cost and label1.time <= label2.time:
            return -1
        if label_2_flag is True and label2.cost <= label1.cost and label2.time <= label1.time:
            return 1
        return 0

    def label_setting(self):
        # 注意此时的路径长度是算入了depot距离的，所以利用这两个函数进行处理
        def avoid_depot_distance_graph(_s, _t):
            if nodes_rel[_s] == 0 or nodes_rel[_t] == 0:
                return 0
            else:
                return distance_graph[nodes_rel[_s]][nodes_rel[_t]]

        def avoid_depot_x_ij(_s, _t):
            if nodes_rel[_s] == 0 or nodes_rel[_t] == 0:
                return self.sub_problem.x_ij[i, j].obj - distance_graph[nodes_rel[_s]][nodes_rel[_t]]
            else:
                return self.sub_problem.x_ij[i, j].obj

        # 成本，时间，满电电池数，缺电电池数，拜访情况列表, dominates
        label_0 = label_1(nodes_cnt, 0)

        self.UL.append(label_0)
        self.TL[0].add_label(label_0, self.compare_label1_label2_1)

        while self.UL:
            _label = self.UL.pop()
            if _label.dominated is False and _label.place != count:
                # print('开始拓展')
                if _label.f == 0 and _label.e == 0 and _label.place != 0:
                    i = _label.place
                    j = nodes_cnt - 1
                    _new_label = _label.extend(avoid_depot_x_ij(i, j), avoid_depot_distance_graph(i, j),
                                               f_node[j], e_node[j], j, nodes_rel[j], 5 * f_node[j] - e_node[j])
                    if _new_label is not None:
                        self.UL.append(_new_label)
                        self.TL[j].add_label(_new_label, self.compare_label1_label2_1)
                    continue

                i = _label.place

                _prev_extend_point = None
                _prev_extend_point_type = None

                for j, to_visit in enumerate(_label.visited):
                    if to_visit == 0 and (nodes_rel[j] != _prev_extend_point or (
                            5 * f_node[j] - e_node[j]) != _prev_extend_point_type):
                        _prev_extend_point = nodes_rel[j]
                        _prev_extend_point_type = 5 * f_node[j] - e_node[j]
                        if i != count and j != 0 and i != j and not (
                                i == 0 and j in demand_nodes + return_nodes) and not (
                                1 <= i <= (supply_num + demand_num) and j == count) and not ((
                                                                                                     count > i > (
                                                                                                     supply_num + demand_num) and
                                                                                                     return_supply[
                                                                                                         i] == j) or (
                                                                                                     count > j > (
                                                                                                     supply_num + demand_num) and
                                                                                                     return_supply[
                                                                                                         j] == i)):
                            _new_label = _label.extend(avoid_depot_x_ij(i, j), avoid_depot_distance_graph(i, j),
                                                       f_node[j], e_node[j],
                                                       j, nodes_rel[j], 5 * f_node[j] - e_node[j])
                            if _new_label is not None:
                                self.UL.append(_new_label)
                                self.TL[j].add_label(_new_label, self.compare_label1_label2_1)

        return

    def return_result(self):
        return self.TL[-1].deque

    def clear(self):
        self.UL = deque()
        for i in range(nodes_cnt):
            self.TL[i].clear()


class label_graph:
    def __init__(self):
        self.paths = list()
        self.cost = 0
        self.time = 0
        self.place = 0
        self.dominated = False
        self.count = Counter()

    def extend(self, cost_add, time_add, des_point, path, points, f_node, e_node, nodes_rel):
        _new_label = label_graph()
        _new_label.cost = self.cost + cost_add
        _new_label.time = self.time + time_add
        _new_label.place = des_point
        _new_label.paths = self.paths + path
        _new_label.count = self.count.copy()
        for j in points:
            if 5 * f_node[j] - e_node[j] == 5:
                _new_label.count[nodes_rel[j]] += 1
                if _new_label.count[nodes_rel[j]] > supply_count[nodes_rel[j]]:
                    return None
            elif 5 * f_node[j] - e_node[j] == -6:
                _new_label.count[nodes_rel[j]] += 1
                if _new_label.count[nodes_rel[j]] > demand_count[nodes_rel[j]]:
                    return None
        if _new_label.time <= H:
            return _new_label
        else:
            return


class TwoLevelGraph:
    def __int__(self):
        self.sub_problem = None
        self.UL = deque()
        self.TL = dict()
        for j in supplys + [0, nodes_cnt - 1]:
            a = label_set()
            self.TL[j] = a
        return

    @staticmethod
    def compare_label1_label2_2(label1, label2):
        # 判断dominates的条件需要考量
        label_1_flag = True
        label_2_flag = True
        for i in supplys + demands:
            if label1.count[i] > label2.count[i]:
                label_1_flag = False
            if label1.count[i] < label2.count[i]:
                label_2_flag = False
        if label_1_flag is True and label1.cost <= label2.cost and label1.time <= label2.time:
            return -1
        if label_2_flag is True and label2.cost <= label1.cost and label2.time <= label1.time:
            return 1
        return 0

    def label_setting_graph(self, sub_routes_obtained):
        label_0 = label_graph()

        self.UL.append(label_0)
        self.TL[0].add_label(label_0, self.compare_label1_label2_2)

        sub_routes_set = defaultdict(list)
        for j in [0] + supplys:
            for k in supplys:
                if j != k:
                    sub_routes_set[j].append((distance_graph[j][k], distance_graph[j][k], k, [j, k], set()))
        for k in supplys:
            sub_routes_set[k].append((distance_graph[k][0], distance_graph[k][0], nodes_cnt - 1, [k, 0], set()))

        for _label in sub_routes_obtained:
            if len(_label.path) > 2 and _label.dominated is False:
                sub_routes_set[nodes_rel[_label.path[1]]].append(
                    (_label.cost, _label.time, nodes_rel[_label.path[-2]], _label.path[1:-1],
                     set(_label.path[1:-1])))

        while self.UL:
            _label = self.UL.pop()
            if _label.dominated is False and _label.place != count:
                # print('开始拓展')
                i = _label.place
                for _sub_route in sub_routes_set[i]:
                    _new_label = _label.extend(_sub_route[0], _sub_route[1], _sub_route[2], _sub_route[3],
                                               _sub_route[4], f_node, e_node, nodes_rel)
                    if _new_label is not None:
                        self.UL.append(_new_label)
                        self.TL[_new_label.place].add_label(_new_label, self.compare_label1_label2_2)
        return

    def return_result(self):
        return self.TL[count].get_best_2()

    def clear(self):
        self.UL = deque()
        for j in supplys + [0, nodes_cnt - 1]:
            self.TL[j].clear()


if __name__ == "__main__":
    bp = MainProblem()
    bp.optimize()
    _dual_sol = bp.get_dual_solution()
    sp = SubProblem()
    sp.adjust_obj_coeff(_dual_sol)
    # sp.optimize()
    # print(_dual_sol)
    # one_level = OneLevel()
    # one_level.sub_problem = sp
    # one_level.label_setting()
    # # one_level.TL[-1].print_paths()
    # column_cost, column_val, column_coeff = one_level.return_result()
    # bp.add_column(column_coeff, column_val, [])
    # bp.optimize()
    # bp.MainProbRelax.write('main.sol')

    print('finished')
