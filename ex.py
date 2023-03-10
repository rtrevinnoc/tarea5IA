from itertools import combinations, product
from random import randint, random, sample
import time

import matplotlib.pyplot as plt

import wgraph as wgf

# Lista de adyacencia de grafo de prueba

# grafo = {'A': [('B', 1), ('C', 2), ('D', 3)],
#          'B': [('A', 1), ('C', 4)],
#          'C': [('A', 2), ('B', 4), ('D', 5)],
#          'D': [('A', 3), ('C', 5)]}

# grafo = {'A': [('B', 4), ('C', 4)],
#          'B': [('A', 4), ('C', 2)],
#          'C': [('D', 3), ('E', 1), ('F', 6)],
#          'D': [('C', 3), ('F', 2)],
#          'E': [('C', 1), ('F', 3)],
#          'F': [('C', 6), ('D', 2), ('E', 3)]}

# graph = {
#     'A': [('B', 2), ('C', 1)],
#     'B': [('A', 2), ('C', 3), ('D', 1)],
#     'C': [('A', 1), ('B', 3), ('D', 2), ('E', 4)],
#     'D': [('B', 1), ('C', 2), ('E', 3)],
#     'E': [('C', 4), ('D', 3)]
# }
#
# graph2 = {
#     'A': [('B', 2), ('C', 3), ('D', 1)],
#     'B': [('A', 2), ('D', 4), ('E', 3)],
#     'C': [('A', 3), ('D', 2)],
#     'D': [('A', 1), ('B', 4), ('C', 2), ('E', 5)],
#     'E': [('B', 3), ('D', 5)]
# }

# graph = {0: [(1, 8), (2, 7), (3, 3), (4, 10)],
#          1: [(0, 10), (3, 8), (4, 1)],
#          2: [(0, 9), (3, 10), (4, 4)],
#          3: [(0, 7), (1, 6), (2, 2), (4, 7)],
#          4: [(0, 8), (1, 7), (2, 9), (3, 9)]}
#
# graph = {'A': [('B', 8), ('C', 7), ('D', 3), ('E', 10)],
#          'B': [('A', 10), ('D', 8), ('E', 1)],
#          'C': [('A', 9), ('D', 10), ('E', 4)],
#          'D': [('A', 7), ('B', 6), ('C', 2), ('E', 7)],
#          'E': [('A', 8), ('B', 7), ('C', 9), ('D', 9)]}

# Convertir a grafo ponderado
# g = wgf.WeightedGraph(grafo)
# gf = wgf.WeightedGraph(graph)
# gf2 = wgf.WeightedGraph(graph2)

# print(g.graph[0])
# print(g.astar('B', 'F'))
# print(gf.astar('A', 'D'))
# print(gf2.astar('A', 'E'))
# print(gf.astar('A', 'E'))
# gf.drawGraph()

# Ejecutar el algoritmo de djikstra desde el vertice A
# print("# Algoritmo de Djikstra desde el vertice A")
# g.djikstra("D")

# # Dibujar el grafo
# g.drawGraph()
#
# Funcion para crear listas de adyacencia aleatorias con cierto numero (n) de nodos con cierto umbral para los nodos a crear
def random_graph(n, p, directed=False):
    nodes = range(n)
    adj_mat = [[] for i in nodes]
    possible_edges = product(nodes, repeat=2) if directed else combinations(nodes, 2)
    for u, v in possible_edges:
        weight = randint(1,10)
        if (len(adj_mat[u]) * len(adj_mat[v]) < 1 or random() < p) and u != v:
            adj_mat[u].append( (v, weight) )
            if not directed:
                adj_mat[v].append( (u, weight) )
    return {vertex: adj_list for vertex, adj_list in enumerate(adj_mat)}

# Clase para construir nuestros experimentos
class Experiment:
    def __init__(self, prob, trials, djikstra=False):
        self.prob = prob
        self.trials = trials
        self.directed = djikstra

        self.currentStep = 1
        self.increment = 1

        self.timesPerStep = []
        self.avgTimesPerStep = []

        self.timesPerStepDirected = []
        self.avgTimesPerStepDirected = []
    
    def runStep(self):
        print("\n# STEP " + str(self.currentStep))
        times = []
        for x in range(self.trials):
            graphMat = random_graph(self.currentStep * self.increment, self.prob)
            graph = wgf.WeightedGraph(graphMat)
            start = time.time()

            # startNode = 0
            # endNode = randint(1, graph.order() - 1)
            startNode, endNode = sample(range(0, graph.order()), 2)

            print(f'Shortest path from node {startNode} to {endNode} is:', graph.astar(startNode, endNode))
            # print(graph.graph)

            end = time.time()

            # graph.drawGraph()

            currentTime = (end - start) * 1000
            times.append(currentTime)
            print("\n# TRIAL " + str(x) + " took " + str(currentTime) + " s")
        avgTime = sum(times) / len(times)
        self.timesPerStep.append(times)
        self.avgTimesPerStep.append(avgTime)

        print("\n\n# STEP TIMES:", times, "AVG: " + str(avgTime) + " s")

    def runStepDjikstra(self):
        print("\n# STEP " + str(self.currentStep))
        times = []
        for x in range(self.trials):
            graphMat = random_graph(self.currentStep * self.increment, self.prob, directed=True)
            graph = wgf.WeightedGraph(graphMat)
            start = time.time()

            # startNode = 0
            # endNode = randint(1, graph.order() - 1)
            graph.djikstra(0)

            # print(graph.graph)

            end = time.time()

            # graph.drawGraph()

            currentTime = (end - start) * 1000
            times.append(currentTime)
            print("\n# TRIAL " + str(x) + " took " + str(currentTime) + " s")
        avgTime = sum(times) / len(times)
        self.timesPerStepDirected.append(times)
        self.avgTimesPerStepDirected.append(avgTime)

        print("\n\n# STEP TIMES:", times, "AVG: " + str(avgTime) + " s")

    def run(self, steps, increment=1):
        self.increment = increment
        self.steps = steps
        for x in range(steps):
            self.runStep()
            if(self.directed): self.runStepDjikstra()
            self.currentStep += 1

    def plot(self):
        rows = 1
        if(self.directed): rows = 2
        fig, ax = plt.subplots(rows, self.trials, sharey=True, sharex=True)

        for step in range(self.trials):
            if not self.directed:
                ax[step].bar([str(trial) for trial in range(self.increment, (self.steps + 1) * self.increment, self.increment)], self.timesPerStep[step])
            else:
                ax[0, step].bar([str(trial) for trial in range(self.increment, (self.steps + 1) * self.increment, self.increment)], self.timesPerStep[step])
                ax[0, step].set_title(str((step + 1) * self.increment) + " nodos")
                ax[1, step].bar([str(trial) for trial in range(self.increment, (self.steps + 1) * self.increment, self.increment)], self.timesPerStepDirected[step])
                ax[0, step].set_title(str((step + 1) * self.increment) + " nodos")

        fig.text(0.5, 0.04, 'Numero de Nodos', ha='center', va='center')
        fig.text(0.06, 0.5, 'Tiempo de Ejecucion (s)', ha='center', va='center', rotation='vertical')
        fig.suptitle('Tiempos de Ejecucion de algoritmo de A* con distancia de Manhattan')
        plt.show()

        plt.plot([str(trial) for trial in range(self.increment, (self.steps + 1) * self.increment, self.increment)], self.avgTimesPerStep, label="A*")
        if (self.directed): plt.plot([str(trial) for trial in range(self.increment, (self.steps + 1) * self.increment, self.increment)], self.avgTimesPerStepDirected, label="Djikstra")

        plt.xlabel("Numero de Nodos")
        plt.ylabel("Tiempo de Ejecucion Promedio (s)")
        plt.legend()
        plt.show()


# # Construir un experimento con cinco pruebas, es decir de probar con grafos de 1, 2, 3, 4 y 5 nodos, con y sin direccionamiento
# ex = Experiment(0.5, 5, directed=True)
# # Correr 5 pruebas por cada tipo de grafo y numero de nodos
# ex.run(5)
# # Construir y mostrar las graficas con los resultados del experimento
# ex.plot()

# Ahora a correr el experimento de 5 en 5 hasta 25
ex2 = Experiment(0.1, 5, djikstra=True)
# Correr 5 pruebas por cada tipo de grafo y numero de nodos
ex2.run(5, increment=20)
# Construir y mostrar las graficas con los resultados del experimento
ex2.plot()
