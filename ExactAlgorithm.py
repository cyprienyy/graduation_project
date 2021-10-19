import gurobipy as gp
from gurobipy import GRB
import numpy as np
from Data import supplys, supply_count, demands, demand_count, vehicle_capacity, distance_graph, H
from collections import deque, Counter, defaultdict

##################准备基础变量#######################
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

############定义其他变量################
VEHICLE_NUM = 2

sub_problem = gp.Model()

# 可以进一步去除一些（i，j）对
lis_i_j = [(i, j) for i in range(nodes_cnt) for j in range(nodes_cnt) if
           i != count and j != 0 and i != j and not (
                   i == 0 and supply_num < j < count) and not (
                   1 <= i <= (supply_num + demand_num) and j == count) and not (
                   count > i > (supply_num + demand_num) and return_supply[i] == j) or (
                   count > j > (supply_num + demand_num) and return_supply[j] == i)]

# 添加变量
x_ijk = sub_problem.addVars([(i, j, k) for i, j in lis_i_j for k in range(VEHICLE_NUM)], vtype=GRB.BINARY, name='x_ijk')
y_i = sub_problem.addVars(nodes_cnt, lb=0.0, ub=vehicle_capacity, vtype=GRB.CONTINUOUS, name='y_i')
z_i = sub_problem.addVars(nodes_cnt, lb=0.0, ub=vehicle_capacity, vtype=GRB.CONTINUOUS, name='z_i')
t_i = sub_problem.addVars(nodes_cnt, lb=0.0, vtype=GRB.CONTINUOUS, name='t_i')

# 添加约束
# sub_problem.addConstr(x_ij.sum(0, '*') == 1)
# sub_problem.addConstr(x_ij.sum('*', count) == 1)
sub_problem.addConstrs(
    x_ijk.sum('*', i, k) == x_ijk.sum(i, '*', k) for i in supply_nodes + demand_nodes + return_nodes for k in
    range(VEHICLE_NUM))
# sub_problem.addConstrs(x_ijk.sum('*', i, '*') == 1 for i in supply_nodes + demand_nodes + return_nodes)
sub_problem.addConstrs(x_ijk.sum('*', i, '*') == 1 for i in demand_nodes)
sub_problem.addConstrs(x_ijk.sum('*', i, '*') <= 1 for i in supply_nodes + return_nodes)
sub_problem.addConstrs(
    x_ijk.sum(i, '*', k) == x_ijk.sum(return_supply[i], '*', k) for i in return_nodes for k in
    range(VEHICLE_NUM))
sub_problem.addConstrs(
    gp.quicksum(x_ijk[i, j, k] * distance_graph[nodes_rel[i], nodes_rel[j]] for i, j in lis_i_j) <= H for k in
    range(VEHICLE_NUM))
sub_problem.addConstrs(
    y_i[i] + f_node[j] <= y_i[j] + vehicle_capacity * (1 - x_ijk.sum(i, j, '*')) for i, j in lis_i_j)
sub_problem.addConstrs(
    y_i[j] - f_node[j] <= y_i[i] + vehicle_capacity * (1 - x_ijk.sum(i, j, '*')) for i, j in lis_i_j)
sub_problem.addConstrs(
    z_i[i] + e_node[j] <= z_i[j] + vehicle_capacity * (1 - x_ijk.sum(i, j, '*')) for i, j in lis_i_j)
sub_problem.addConstrs(
    z_i[j] - e_node[j] <= z_i[i] + vehicle_capacity * (1 - x_ijk.sum(i, j, '*')) for i, j in lis_i_j)
sub_problem.addConstrs(y_i[i] + z_i[i] <= vehicle_capacity for i in range(nodes_cnt))
sub_problem.addConstrs(y_i[i] + z_i[i] == 0 for i in [0, count])
sub_problem.addConstrs(t_i[i] + 1 <= t_i[j] + (nodes_cnt + 1) * (1 - x_ijk.sum(i, j, '*')) for i, j in lis_i_j)
sub_problem.addConstrs(t_i[i] >= t_i[return_supply[i]] + 1 for i in return_nodes)
sub_problem.addConstrs(z_i[i] == 0 for i in supply_nodes)
sub_problem.addConstrs(y_i[i] == 0 for i in return_nodes)

for i, j in lis_i_j:
    for k in range(VEHICLE_NUM):
        x_ijk[i, j, k].obj = distance_graph[nodes_rel[i], nodes_rel[j]]

sub_problem.setAttr(GRB.Attr.ModelSense, GRB.MINIMIZE)
sub_problem.optimize()
sub_problem.write('ExactAlgorithm.sol')
