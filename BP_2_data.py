from Data import supplys, supply_count, demands, demand_count

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
lis_i_j = [(i, j) for i in range(nodes_cnt) for j in range(nodes_cnt) if
           i != count and j != 0 and i != j and not (
                   i == 0 and j in demand_nodes + return_nodes) and not (
                   1 <= i <= (supply_num + demand_num) and j == count) and not (
                   count > i > (supply_num + demand_num) and return_supply[i] == j) or (
                   count > j > (supply_num + demand_num) and return_supply[j] == i)]
