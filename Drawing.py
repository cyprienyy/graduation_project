import numpy as np
from matplotlib import pyplot as plt
from ReadFiles import get_soloman_cor
import re


def draw_route(axis, depot, supply, demand, routes):
    plt.rcParams['font.sans-serif'] = ['SimSun']
    plt.scatter(axis[depot][0], axis[depot][1], c='black', marker=',', label='车场')

    supply_x = [axis[_s][0] for _s in supply]
    supply_y = [axis[_s][1] for _s in supply]
    plt.scatter(supply_x, supply_y, c='black', marker='^', label='电池柜')

    demand_x = [axis[_d][0] for _d in demand]
    demand_y = [axis[_d][1] for _d in demand]
    plt.scatter(demand_x, demand_y, c='black', marker='o', label='需求')

    c = ['black', 'green', 'blue']
    ls = ['-', '--', ':']
    for i, _r in enumerate(routes):
        _st = [axis[i][0] for i in _r]
        _ed = [axis[i][1] for i in _r]
        plt.plot(_st, _ed, c=c[i], ls=ls[i])
    plt.legend(loc='best', fontsize=17)
    plt.xticks(fontsize=17)
    plt.yticks(fontsize=17)
    plt.show()


def draw_curve(filepath='TempFiles/data.csv'):
    with open(filepath, 'r') as file_to_read:
        ga = []
        ant = []
        ant_ga = []
        imp_ga = []
        while True:
            lines = file_to_read.readline()  # 整行读取数据
            if not lines:
                break
                pass
            if re.match(r'\s*[0-9]', lines) is not None:
                lines = lines.strip()
                lines = lines.split(',')
                lines = list(map(float, lines))
                ga.append(lines[0])
                ant.append(lines[1])
                ant_ga.append(lines[2])
                imp_ga.append(lines[3])
            pass
    pass
    # plt.rcParams['font.sans-serif'] = ['SimHei']

    font1 = {'family': 'Times New Roman',
             'weight': 'normal',
             'size': 23,
             }

    font2 = {'family': 'SimSun',
             'weight': 'normal',
             'size': 20,
             }
    plt.plot(range(100), imp_ga[:100], label='改进的遗传算法', ls='-.')
    plt.plot(range(999), ga, label='遗传算法', ls='-')
    plt.plot(range(999), ant, label='蚁群算法', ls='--')
    # plt.plot(range(999), ant_ga, label='蚁群算法提供初始解的遗传算法',ls=':')

    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.legend(prop=font2)
    plt.xlabel('迭代次数', font2)
    plt.ylabel('路径长度', font2)
    plt.title('各算法收敛图', font2)
    plt.savefig('figure1.png')
    plt.show()
    return


if __name__ == '__main__':
    # draw_curve()

    _cor = get_soloman_cor(r'TempFiles\RC101_50.txt')

    x = [
        [0, 3, 5, 3, 3, 1, 7, 23, 25, 24, 20, 21, 22, 8, 8, 11, 9, 6, 4, 3, 1, 1, 2, 10, 13, 17, 17, 18, 15, 19, 16, 14,
         12,
         2, 1, 0]]

    y = [[0, 1, 4, 6, 9, 11, 10, 8, 7, 5, 1, 2, 12, 14, 16, 15, 19, 18, 17, 2, 3, 22, 21, 20, 24, 25, 23, 17, 13, 3, 0]]

    Z = [[0, 1, 4, 6, 8, 9, 11, 10, 7, 5, 1, 2, 13, 17, 18, 19, 16, 14, 12, 2, 3, 23, 22, 21, 20, 24, 25, 15, 3, 0]]

    imp_ga_r = [
        [0, 1, 4, 1, 1, 2, 6, 8, 10, 23, 25, 24, 22, 20, 21, 7, 5, 3, 1, 2, 2, 9, 12, 14, 16, 16, 15, 19, 18, 17, 13,
         11, 9, 2, 0]]

    ant_ga_r = [
        [0, 3, 25, 24, 22, 20, 21, 23, 5, 3, 3, 1, 4, 6, 8, 9, 11, 10, 7, 3, 1, 1, 2, 13, 12, 12, 14, 16, 15, 19, 18,
         17, 2, 1, 0]]
    # draw_route(_cor, 0, list(range(1, 4)), list(range(4, 26)), [])
    draw_route(_cor, 0, list(range(1,45)), list(range(45, 51)), [])
