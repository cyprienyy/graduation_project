import gurobipy as gp
from gurobipy import GRB
import numpy as np
from Data import supplys, supply_count, demands, demand_count, vehicle_capacity, distance_graph, H
from collections import deque, Counter, defaultdict


def compare_label1_label2(label1, label2):
    # 0即无关系，-1代表label1 dominates，1代表label2 dominates
    # 判断dominates的条件需要考量
    label_1_flag = True
    label_2_flag = True
    for i, visited in enumerate(label1.visited):
        if visited > label2.visited[i]:
            label_2_flag = False
        if visited < label2.visited[i]:
            label_1_flag = False
    if label_1_flag is True and label1.cost <= label2.cost and label1.time <= label2.time:
        return -1
    if label_2_flag is True and label2.cost <= label1.cost and label2.time <= label1.time:
        return 1
    return 0


def compare_label1_label2_1(label1, label2):
    # 判断dominates的条件需要考量
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
        _label = None
        for _label in self.deque:
            if _label.cost < res:
                _best = _label
                res = _label.cost
        return _label.cost, _label.time, np.array(_label.visited[1:-1])

    def print_paths(self):
        for _label in self.deque:
            print(_label.path)

    def get_best_2(self):
        if len(self.deque) == 0:
            raise Exception('没有找到任何列')
        res = float('inf')
        _label = None
        for _label in self.deque:
            if _label.cost < res:
                _best = _label
                res = _label.cost
        array = []
        for j in supplys + demands:
            array.append(_label.count[j])
        return _label.cost, _label.time, array


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
            return _new_label
        else:
            return None


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
        _array = list()
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
        _new_label.path = self.path + [des]

        if des_type == 5:
            _new_label.count[des_point] += 1
        elif des_type == 1:
            _new_label.count[des_point] -= 1
        if _new_label.count[des_point] >= 0 and _new_label.time <= H and _new_label.f >= 0 and _new_label.e >= 0 and (
                _new_label.f + _new_label.e) <= vehicle_capacity and not (
                des_type == 0 and (_new_label.f != 0 or _new_label.e != 0)):
            return _new_label
        else:
            return None


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


def BP():
    nodes_cnt = 2 * sum(supply_count.values()) + 2 + sum(demand_count.values())
    supply_nodes = []
    demand_nodes = []
    return_nodes = []
    count = 1
    nodes_rel = dict()
    f_node = dict()
    e_node = dict()
    return_supply = dict()
    for _s in supplys:
        supply_nodes += list(range(count, count + supply_count[_s]))
        for i in range(count, count + supply_count[_s]):
            nodes_rel[i] = _s
            f_node[i] = 1
            e_node[i] = 0
        count += supply_count[_s]
    for _d in demands:
        demand_nodes += list(range(count, count + demand_count[_d]))
        for i in range(count, count + demand_count[_d]):
            nodes_rel[i] = _d
            f_node[i] = -1
            e_node[i] = 1
        count += demand_count[_d]
    tmp_count = count
    for _s in supplys:
        return_nodes += list(range(count, count + supply_count[_s]))
        for i in range(count, count + supply_count[_s]):
            nodes_rel[i] = _s
            f_node[i] = 0
            e_node[i] = -1
            return_supply[i] = i - tmp_count + 1
        count += supply_count[_s]

    nodes_rel[count] = 0
    nodes_rel[0] = 0
    f_node[count] = 0
    f_node[0] = 0
    e_node[0] = 0
    e_node[count] = 0
    supply_num = sum(supply_count.values())
    demand_num = sum(demand_count.values())

    '''
    n_r = np.ones((tmp_count, 2))
    c_r = np.zeros(n_r.shape[1])
    n_r[:, 0] = np.array([0, 1, 0, 0, 1, 0])
    n_r[:, 1] = np.array([0, 0, 0, 1, 0, 1])
    c_r[0] = 446
    c_r[1] = 484

    try:
        MainProbRelax = gp.Model()  # 松弛后的列生成主问题

        # 构造主问题模型
        # 添加变量
        X_r = MainProbRelax.addVars(n_r.shape[1], lb=0.0, ub=1.0, obj=c_r, vtype=GRB.CONTINUOUS, name='X_r')
        # 添加约束
        supply_constr = MainProbRelax.addConstrs(
            gp.quicksum(n_r[i, r] * X_r[r] for r in range(n_r.shape[1])) <= 1 for
            i in supply_nodes)
        demand_constr = MainProbRelax.addConstrs(
            gp.quicksum(n_r[i, r] * X_r[r] for r in range(n_r.shape[1])) == 1 for
            i in demand_nodes)
    '''
    n_r = np.ones((len(supplys) + len(demands), 2))
    c_r = np.zeros(n_r.shape[1])
    n_r[:, 0] = np.array([1, 0, 1, 0])
    n_r[:, 1] = np.array([0, 1, 0, 1])
    c_r[0] = 446
    c_r[1] = 484

    try:
        MainProbRelax = gp.Model()  # 松弛后的列生成主问题

        # 构造主问题模型
        # 添加变量
        X_r = MainProbRelax.addVars(n_r.shape[1], lb=0.0, ub=1.0, obj=c_r, vtype=GRB.CONTINUOUS, name='X_r')
        # 添加约束
        i = 0
        for j in supplys:
            MainProbRelax.addConstr(gp.quicksum(n_r[i, r] * X_r[r] for r in range(n_r.shape[1])) <= supply_count[j])
            i = i + 1
        for j in demands:
            MainProbRelax.addConstr(gp.quicksum(n_r[i, r] * X_r[r] for r in range(n_r.shape[1])) == demand_count[j])
            i = i + 1
        MainProbRelax.setAttr(GRB.Attr.ModelSense, GRB.MINIMIZE)
        # 求解
        MainProbRelax.optimize()

        # 获得对偶值
        _Dualsolution = MainProbRelax.getAttr("Pi", MainProbRelax.getConstrs())

        Dualsolution = [0]
        i = 0
        for j in supplys:
            Dualsolution = Dualsolution + [_Dualsolution[i] / supply_count[j]] * supply_count[j]
            i = i + 1
        for j in demands:
            Dualsolution = Dualsolution + [_Dualsolution[i] / demand_count[j]] * demand_count[j]
            i = i + 1

        # 构造子问题模型
        sub_problem = gp.Model()

        lis_i_j = [(i, j) for i in range(nodes_cnt) for j in range(nodes_cnt) if
                   i != count and j != 0 and i != j and not (
                           i == 0 and j in demand_nodes + return_nodes) and not (
                           1 <= i <= (supply_num + demand_num) and j == count) and not (
                           count > i > (supply_num + demand_num) and return_supply[i] == j) or (
                           count > j > (supply_num + demand_num) and return_supply[j] == i)]

        # 添加变量
        x_ij = sub_problem.addVars(lis_i_j, vtype=GRB.BINARY, name='x_ij')
        y_i = sub_problem.addVars(nodes_cnt, lb=0.0, ub=vehicle_capacity, vtype=GRB.CONTINUOUS, name='y_i')
        z_i = sub_problem.addVars(nodes_cnt, lb=0.0, ub=vehicle_capacity, vtype=GRB.CONTINUOUS, name='z_i')
        t_i = sub_problem.addVars(nodes_cnt, lb=0.0, vtype=GRB.CONTINUOUS, name='t_i')

        # 添加约束
        sub_problem.addConstr(x_ij.sum(0, '*') == 1)
        sub_problem.addConstr(x_ij.sum('*', count) == 1)
        sub_problem.addConstrs(
            x_ij.sum('*', i) == x_ij.sum(i, '*') for i in supply_nodes + demand_nodes + return_nodes)
        sub_problem.addConstrs(x_ij.sum(i, '*') == x_ij.sum(return_supply[i], '*') for i in return_nodes)
        sub_problem.addConstr(
            gp.quicksum(x_ij[i, j] * distance_graph[nodes_rel[i], nodes_rel[j]] for i, j in lis_i_j) <= H)
        sub_problem.addConstrs(
            y_i[i] + f_node[j] <= y_i[j] + vehicle_capacity * (1 - x_ij[i, j]) for i, j in lis_i_j)
        sub_problem.addConstrs(
            y_i[j] - f_node[j] <= y_i[i] + vehicle_capacity * (1 - x_ij[i, j]) for i, j in lis_i_j)
        sub_problem.addConstrs(
            z_i[i] + e_node[j] <= z_i[j] + vehicle_capacity * (1 - x_ij[i, j]) for i, j in lis_i_j)
        sub_problem.addConstrs(
            z_i[j] - e_node[j] <= z_i[i] + vehicle_capacity * (1 - x_ij[i, j]) for i, j in lis_i_j)
        sub_problem.addConstrs(y_i[i] + z_i[i] <= vehicle_capacity for i in range(nodes_cnt))
        sub_problem.addConstrs(y_i[i] + z_i[i] == 0 for i in [0, count])
        sub_problem.addConstrs(t_i[i] + 1 <= t_i[j] + (nodes_cnt + 1) * (1 - x_ij[i, j]) for i, j in lis_i_j)
        sub_problem.addConstrs(t_i[i] >= t_i[return_supply[i]] + 1 for i in return_nodes)

        def adjust_obj_coeff():
            for i, j in lis_i_j:
                if 1 <= j <= supply_num + demand_num:
                    x_ij[i, j].obj = distance_graph[nodes_rel[i], nodes_rel[j]] - Dualsolution[j - 1]
                else:
                    x_ij[i, j].obj = distance_graph[nodes_rel[i], nodes_rel[j]]
            return

        adjust_obj_coeff()
        sub_problem.update()

        # sub_problem.optimize()
        # for v in sub_problem.getVars():
        #     if v.X != 0.0:
        #         print('%s %g' % (v.VarName, v.X))

        def label_setting(label_set=label_set, label=label):
            # 成本，时间，满电电池数，缺电电池数，拜访情况列表, dominates
            label_0 = label(nodes_cnt, 0)
            UL = deque()
            TL = list()
            for _ in range(nodes_cnt):
                a = label_set()
                TL.append(a)

            UL.append(label_0)
            TL[0].add_label(label_0, compare_label1_label2)

            while UL:
                _label = UL.pop()
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
                                    count > i > (supply_num + demand_num) and return_supply[i] == j) or (
                                    count > j > (supply_num + demand_num) and return_supply[j] == i):
                                _new_label = _label.extend(x_ij[i, j].obj, distance_graph[nodes_rel[i], nodes_rel[j]],
                                                           f_node[j], e_node[j],
                                                           j, nodes_rel[j], 5 * f_node[j] - e_node[j])
                                if _new_label is not None:
                                    UL.append(_new_label)
                                    TL[j].add_label(_new_label, compare_label1_label2)

            return TL[-1].get_best()

        def label_setting_1(label_set=label_set, label=label_1):
            # 注意此时的路径长度是算入了depot距离的，所以利用这两个函数进行处理

            def avoid_depot_distance_graph(_s, _t):
                if nodes_rel[_s] == 0 or nodes_rel[_t] == 0:
                    return 0
                else:
                    return distance_graph[nodes_rel[_s]][nodes_rel[_t]]

            def avoid_depot_x_ij(_s, _t):
                if nodes_rel[_s] == 0 or nodes_rel[_t] == 0:
                    return x_ij[i, j].obj - distance_graph[nodes_rel[_s]][nodes_rel[_t]]
                else:
                    return x_ij[i, j].obj

            # 成本，时间，满电电池数，缺电电池数，拜访情况列表, dominates
            label_0 = label(nodes_cnt, 0)
            UL = deque()
            TL = list()
            for _ in range(nodes_cnt):
                a = label_set()
                TL.append(a)

            UL.append(label_0)
            TL[0].add_label(label_0, compare_label1_label2_1)

            while UL:
                _label = UL.pop()
                if _label.dominated is False and _label.place != count:
                    # print('开始拓展')

                    if _label.f == 0 and _label.e == 0 and _label.place != 0:
                        i = _label.place
                        j = nodes_cnt - 1
                        _new_label = _label.extend(avoid_depot_x_ij(i, j), avoid_depot_distance_graph(i, j),
                                                   f_node[j], e_node[j], j, nodes_rel[j], 5 * f_node[j] - e_node[j])
                        if _new_label is not None:
                            UL.append(_new_label)
                            TL[j].add_label(_new_label, compare_label1_label2_1)
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
                                    1 <= i <= (supply_num + demand_num) and j == count) and not (
                                    count > i > (supply_num + demand_num) and return_supply[i] == j) or (
                                    count > j > (supply_num + demand_num) and return_supply[j] == i):
                                _new_label = _label.extend(avoid_depot_x_ij(i, j), avoid_depot_distance_graph(i, j),
                                                           f_node[j], e_node[j],
                                                           j, nodes_rel[j], 5 * f_node[j] - e_node[j])
                                if _new_label is not None:
                                    UL.append(_new_label)
                                    TL[j].add_label(_new_label, compare_label1_label2_1)

            return TL[-1].deque

        def label_setting_graph(sub_routes_obtained):
            label_0 = label_graph()
            UL = deque()
            TL = dict()
            for j in supplys + [0, nodes_cnt - 1]:
                a = label_set()
                TL[j] = a
            UL.append(label_0)
            TL[0].add_label(label_0, compare_label1_label2_2)

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

            while UL:
                _label = UL.pop()
                if _label.dominated is False and _label.place != count:
                    # print('开始拓展')
                    i = _label.place
                    for _sub_route in sub_routes_set[i]:
                        _new_label = _label.extend(_sub_route[0], _sub_route[1], _sub_route[2], _sub_route[3],
                                                   _sub_route[4], f_node, e_node, nodes_rel)
                        if _new_label is not None:
                            UL.append(_new_label)
                            TL[_new_label.place].add_label(_new_label, compare_label1_label2_2)

            return TL[count].get_best_2()

        def return_val_and_coeff():
            return 0, 0

        _tmp = label_setting_1()
        column_cost, column_val, columnCoeff = label_setting_graph(_tmp)

        '''
        column_cost, column_val, columnCoeff = label_setting()
        columnCoeff = columnCoeff[:demand_num + supply_num]

        # 判断Reduced Cost是否小于零
        # while sub_problem.objVal < 0:
        while column_cost < 0:
            # 获取变量取值
            # column_val, columnCoeff = return_val_and_coeff()
            column = gp.Column(columnCoeff, MainProbRelax.getConstrs())
            # 添加变量
            MainProbRelax.addVar(obj=column_val, lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name="CG", column=column)
            # 求解
            MainProbRelax.optimize()
            # 修改子问题目标函数系数
            _Dualsolution = MainProbRelax.getAttr("Pi", MainProbRelax.getConstrs())
            # 需要改变Dualsolution
            adjust_obj_coeff()
            sub_problem.update()
            column_cost, column_val, columnCoeff = label_setting()
            columnCoeff = columnCoeff[:demand_num + supply_num]
            # sub_problem.optimize()

        # 将CG后的模型转为整数，并求解,需要更改
        for v in MainProbRelax.getVars():
            v.setAttr("VType", GRB.INTEGER)
        MainProbRelax.optimize()
        for v in MainProbRelax.getVars():
            if v.X != 0.0:
                print('%s %g' % (v.VarName, v.X))
        '''
    except gp.GurobiError as e:
        print('Error code ' + str(e.errno) + ": " + str(e))

    except AttributeError:
        print('Encountered an attribute error')
    return


if __name__ == "__main__":
    BP()
