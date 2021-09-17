import gurobipy as gp
from gurobipy import GRB
import numpy as np
from Data import supplys, supply_count, demands, demand_count, vehicle_capacity, distance_graph, H
from collections import deque, Counter


def compare_label1_label2(label1, label2):
    # 0即无关系，-1代表label1 dominates，1代表label2 dominates
    return 0


class label_set():
    def __init__(self):
        self._deque = deque()

    def add_label(self, label):
        flag = True
        _new_deque = deque()
        while self._deque:
            old_label = self._deque.pop()
            res = compare_label1_label2(old_label, label)
            if res == -1:
                flag = False
                label[5] = True
                _new_deque.append(old_label)
            elif res == 0:
                _new_deque.append(old_label)
            elif res == 1:
                old_label[5] = True
        if flag is True:
            _new_deque.append(label)
        self._deque = _new_deque

    def get_best(self):
        res = float('inf')
        for _label in self._deque:
            if _label.cost < res:
                _best = _label
                res = _label.cost
        return _label.cost, _label.time, np.array(_label.visited[1:-1])


class label():
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
        MainProbRelax.setAttr(GRB.Attr.ModelSense, GRB.MINIMIZE)
        # 求解
        MainProbRelax.optimize()

        # 获得对偶值
        Dualsolution = MainProbRelax.getAttr("Pi", MainProbRelax.getConstrs())

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
            TL[0].add_label(label_0)

            while UL:
                _label = UL.pop()
                if _label.dominated is False and _label.place != count:
                    # print('开始拓展')
                    i = _label.place
                    for j, to_visit in enumerate(_label.visited):
                        if to_visit == 0:
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
                                    TL[j].add_label(_new_label)

            return TL[-1].get_best()

        def return_val_and_coeff():
            return 0, 0

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
            Dualsolution = MainProbRelax.getAttr("Pi", MainProbRelax.getConstrs())
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

    except gp.GurobiError as e:
        print('Error code ' + str(e.errno) + ": " + str(e))

    except AttributeError:
        print('Encountered an attribute error')
    return


if __name__ == "__main__":
    BP()
