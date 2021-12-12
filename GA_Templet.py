# -*- coding: utf-8 -*-
import numpy as np
import geatpy as ea  # 导入geatpy库
from sys import path as paths
from os import path
from collections import defaultdict
import random
from Neighborhood_Search import improve_journey

paths.append(path.split(path.split(path.realpath(__file__))[0])[0])


class soea_psy_EGA_templet_1(ea.SoeaAlgorithm):
    """
soea_psy_EGA_templet.py - Polysomy Elitist Reservation GA templet(精英保留的多染色体遗传算法模板)
模板说明:
    该模板是内置算法模板soea_EGA_templet的多染色体版本，
    因此里面的种群对象为支持混合编码的多染色体种群类PsyPopulation类的对象。

算法描述:
    本模板实现的是基于杰出保留的单目标遗传算法。算法流程如下：
    1) 根据编码规则初始化N个个体的种群。
    2) 若满足停止条件则停止，否则继续执行。
    3) 对当前种群进行统计分析，比如记录其最优个体、平均适应度等等。
    4) 独立地从当前种群中选取N-1个母体。
    5) 独立地对这N-1个母体进行交叉操作。
    6) 独立地对这N-1个交叉后的个体进行变异。
    7) 计算当代种群的最优个体，并把它插入到这N-1个交叉后的个体的第一位，得到新一代种群。
    8) 回到第2步。

"""

    def __init__(self, problem, population):
        ea.SoeaAlgorithm.__init__(self, problem, population)  # 先调用父类构造方法
        if population.ChromNum == 1:
            raise RuntimeError('传入的种群对象必须是多染色体的种群类型。')
        self.name = 'psy-EGA'
        self.selFunc = 'tour'  # 锦标赛选择算子
        # 由于有多个染色体，因此需要用多个重组和变异算子
        self.recOpers = []
        self.mutOpers = []
        self.recOpers.append(ea.Xovpmx(XOVR=0.5))
        self.recOpers.append(ea.Xovox(XOVR=0.8))
        self.mutOpers.append(ea.Mutswap(Pm=0.5))
        self.mutOpers.append(ea.Mutswap(Pm=0.8))

    def run(self, prophetPop=None):  # prophetPop为先知种群（即包含先验知识的种群）
        # ==========================初始化配置===========================
        population = self.population
        NIND = population.sizes
        self.initialization()  # 初始化算法模板的一些动态参数
        # ===========================准备进化============================
        population.initChrom(NIND)  # 初始化种群染色体矩阵
        self.call_aimFunc(population)  # 计算种群的目标函数值
        # 插入先验知识（注意：这里不会对先知种群prophetPop的合法性进行检查，故应确保prophetPop是一个种群类且拥有合法的Chrom、ObjV、Phen等属性）
        if prophetPop is not None:
            population = (prophetPop + population)[:NIND]  # 插入先知种群
        population.FitnV = ea.scaling(population.ObjV, population.CV, self.problem.maxormins)  # 计算适应度
        iter = 0
        _record = []
        # ===========================开始进化============================
        while self.terminated(population) == False:
            bestIndi = population[np.argmax(population.FitnV, 0)]  # 得到当代的最优个体
            # 选择
            if self.trace['f_avg'][-1] - self.trace['f_best'][-1] > 0.1:
                offspring = population[ea.selecting(self.selFunc, population.FitnV, NIND - 1)]
                # 进行进化操作，分别对各种编码的染色体进行重组和变异
                for i in range(population.ChromNum):
                    offspring.Chroms[i] = self.recOpers[i].do(offspring.Chroms[i])  # 重组

                for i in range(offspring.sizes):
                    offspring.Chroms[0][i], offspring.Chroms[1][i] = improve_journey(list(offspring.Chroms[0][i]), list(
                        offspring.Chroms[1][i]))
            else:
                # print(self.trace['f_avg'][0], self.trace['f_best'][0])
                offspring.initChrom(NIND - 1)
            self.call_aimFunc(offspring)  # 计算目标函数值
            population = bestIndi + offspring  # 更新种群
            population.FitnV = ea.scaling(population.ObjV, population.CV, self.problem.maxormins)  # 计算适应度
            iter += 1
            _record.append([iter, bestIndi.ObjV[0][0]])
        return self.finishing(population), _record  # 调用finishing完成后续工作并返回结果


class ERX:
    def __init__(self, XOVR=0.5):
        self.xovr = XOVR

    def do(self, array):
        _res = []
        n = array.shape[0]
        k = n // 2
        num = 0
        index = 0
        while num < n:
            parent_1 = array[(index) % n].tolist()
            parent_2 = array[(index + k) % n].tolist()
            a = random.random()
            if a < self.xovr:
                _res.append(self.edge_recombination_crossover(parent_1, parent_2))
                _res.append(self.edge_recombination_crossover(parent_2, parent_1))
            else:
                _res.append(parent_2)
                _res.append(parent_1)
            index += 1
            num += 2
        return np.array(_res[:n])

    @staticmethod
    def edge_recombination_crossover(parent_1, parent_2):
        _unvisited = set(parent_1)
        _visited = set()
        _edge_dict = defaultdict(set)
        for i, j in zip(parent_1, [parent_1[-1]] + parent_1[:-1]):
            _edge_dict[i].add(j)
            _edge_dict[j].add(i)
        for i, j in zip(parent_2, [parent_2[-1]] + parent_2[:-1]):
            _edge_dict[i].add(j)
            _edge_dict[j].add(i)
        _child = []
        _candidates = [parent_1[0]]
        while _unvisited:
            if _candidates:
                _current_city = random.choice(_candidates)
            else:
                _current_city = random.choice(list(_unvisited))
            _child.append(_current_city)
            _visited.add(_current_city)
            _unvisited.remove(_current_city)
            for i in _unvisited:
                _edge_dict[i].discard(_current_city)
            _candidates = []
            k = float('inf')
            for i in _edge_dict[_current_city]:
                m = len(_edge_dict[i])
                if m < k:
                    _candidates = [i]
                    k = m
                elif m == k:
                    _candidates.append(i)
        if len(_child) != len(parent_1):
            raise Exception('ERX交叉时产生了问题')
        return _child


class soea_psy_EGA_templet_2(ea.SoeaAlgorithm):
    """
soea_psy_EGA_templet.py - Polysomy Elitist Reservation GA templet(精英保留的多染色体遗传算法模板)
模板说明:
    该模板是内置算法模板soea_EGA_templet的多染色体版本，
    因此里面的种群对象为支持混合编码的多染色体种群类PsyPopulation类的对象。

算法描述:
    本模板实现的是基于杰出保留的单目标遗传算法。算法流程如下：
    1) 根据编码规则初始化N个个体的种群。
    2) 若满足停止条件则停止，否则继续执行。
    3) 对当前种群进行统计分析，比如记录其最优个体、平均适应度等等。
    4) 独立地从当前种群中选取N-1个母体。
    5) 独立地对这N-1个母体进行交叉操作。
    6) 独立地对这N-1个交叉后的个体进行变异。
    7) 计算当代种群的最优个体，并把它插入到这N-1个交叉后的个体的第一位，得到新一代种群。
    8) 回到第2步。

"""

    def __init__(self, problem, population):
        ea.SoeaAlgorithm.__init__(self, problem, population)  # 先调用父类构造方法
        if population.ChromNum == 1:
            raise RuntimeError('传入的种群对象必须是多染色体的种群类型。')
        self.name = 'psy-EGA'
        self.selFunc = 'tour'  # 锦标赛选择算子
        # 由于有多个染色体，因此需要用多个重组和变异算子
        self.recOpers = []
        self.mutOpers = []
        self.recOpers.append(ea.Xovpmx(XOVR=0.8))
        self.recOpers.append(ea.Xovox(XOVR=0.8))
        self.mutOpers.append(ea.Mutswap(Pm=0.6))
        self.mutOpers.append(ea.Mutswap(Pm=0.6))

    def run(self, prophetPop=None):  # prophetPop为先知种群（即包含先验知识的种群）
        # ==========================初始化配置===========================
        population = self.population
        NIND = population.sizes
        self.initialization()  # 初始化算法模板的一些动态参数
        # ===========================准备进化============================
        population.initChrom(NIND)  # 初始化种群染色体矩阵
        self.call_aimFunc(population)  # 计算种群的目标函数值
        # 插入先验知识（注意：这里不会对先知种群prophetPop的合法性进行检查，故应确保prophetPop是一个种群类且拥有合法的Chrom、ObjV、Phen等属性）
        if prophetPop is not None:
            population = (prophetPop + population)[:NIND]  # 插入先知种群
        population.FitnV = ea.scaling(population.ObjV, population.CV, self.problem.maxormins)  # 计算适应度
        iter = 0
        _record = []
        # ===========================开始进化============================
        while self.terminated(population) == False:
            bestIndi = population[np.argmax(population.FitnV, 0)]  # 得到当代的最优个体
            # 选择
            offspring = population[ea.selecting(self.selFunc, population.FitnV, NIND - 1)]
            # 进行进化操作，分别对各种编码的染色体进行重组和变异
            for i in range(population.ChromNum):
                offspring.Chroms[i] = self.recOpers[i].do(offspring.Chroms[i])  # 重组
                offspring.Chroms[i] = self.mutOpers[i].do(offspring.Encodings[i], offspring.Chroms[i],
                                                          offspring.Fields[i])  # 变异
            self.call_aimFunc(offspring)  # 计算目标函数值
            population = bestIndi + offspring  # 更新种群
            population.FitnV = ea.scaling(population.ObjV, population.CV, self.problem.maxormins)  # 计算适应度
            iter += 1
            _record.append([iter, bestIndi.ObjV[0][0]])
        return self.finishing(population), _record  # 调用finishing完成后续工作并返回结果


if __name__ == '__main__':
    erx = ERX()
    print(erx.do(
        np.array([[1.0, 2.0, 3.0, 4.0, 5.0, 6.0], [2.0, 4.0, 3.0, 1.0, 5.0, 6.0], [1.0, 2.0, 4.0, 5.0, 6.0, 3.0]])))
