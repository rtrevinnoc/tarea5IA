import networkx as nx
import matplotlib.pyplot as plt
# grafo no dirigido
# admite lazos 

class Graph():
    # Constructor, por defecto crea un diccionario vacío
    # El grafo se presenta como un diccionario de la forma
    # {nodo: [arcos]}
    # arcos es una lista de los nodos adyacentes 
    def __init__(self, graph={}):
        self.graph = graph

        self.nxGraph = nx.Graph()
        self.nxGraph.add_nodes_from(self.nodes())
        self.nxGraph.add_edges_from([(edge[0], edge[1][0], {'weight': edge[1][1]}) for edge in self.edges()])

    # Devuelve una representación formal del contenido del grafo
    def __repr__(self):
        nodes = ''
        for node, edges in self.graph.items():
            nodes += f'{node}: {edges}\n'
        return nodes

    # Iterador para recorrer todos los nodos del grafo
    def __iter__(self):
        self.iter_obj = iter(self.graph)
        return self.iter_obj

    # Devuelve los nodos del grafo como una lista
    def nodes(self):
        return list(self.graph.keys())

    # Devuelve los arcos del grafo como una lista de tuplas
    # (nodo_origen, nodo_destino)
    def edges(self, node=None):
        if node:
            if self.existNode(node):
                return [(node, e) for e in self.graph[node]]
            else:
                return []
        else:
            return [(n, e) for n in self.graph.keys() for e in self.graph[n]]

    # Devuelve una lista de los nodos aislados
    def isolatedNodes(self):
        return [node for node in self.graph if not self.graph[node]]

    # Devuelve el número de nodos del grafo
    def order(self):
        return len(self.graph)

    # Devuelve el número de arcos del grafo
    def size(self):
        arcs = []
        for node, edges in self.graph.items():
            for edge in edges:
                sorted_edge = sorted([node, edge])
                if sorted_edge not in arcs:
                    arcs.append(sorted_edge)
        return len(arcs)
                   
    # Devuelve el número de arcos que conectan con el nodo
    # Los lazos se cuentan doblemente
    def degree(self, node):
        if self.graph == {}:
            return -1       # grafo vacío
        degree = len(self.graph[node])
        if node in self.graph[node]:
            degree += 1     # existe un lazo
        return degree

    # Devuelve el grado máximo
    def Delta(self):
        if self.graph == {}:
            return -1       # grafo vacío
        degrees = [self.degree(node) for node in self.graph]
        degrees.sort(reverse=True)
        return degrees[0]

    # Inserta un nodo en el grafo
    def addNode(self, node):
        # Si el nodo no está en el grafo,
        # se añade al diccionario con una lista vacía 
        if node not in self.graph:
            self.graph[node] = []   # nodo aislado

    # Elimina un nodo del grafo
    # Primero elimina todos los arcos del nodo
    def removeNode(self, node):
        if node in self.graph:
            edges = list(self.graph[node])
            for edge in edges:
                self.removeEdge((node, edge))
            self.graph.pop(node)

    # Inserta una arco entre los nodos indicados
    # El arco es una lista con los nodos que une
    def addEdge(self, edge):
        n1, n2 = tuple(edge)
        # Se crean arcos del nodo1 al nodo2 y viceversa
        for n, e in [(n1, n2), (n2, n1)]:
            if n in self.graph:
                if e not in self.graph[n]:
                    self.graph[n].append(e)
                    if n == e:
                        break           # es un lazo
            else:
                self.graph[n] = [e]     # no existe el nodo, se inserta

    # Elimina un arco entre nodos
    # El arco es una lista con los nodos que une
    def removeEdge(self, edge):
        n1, n2 = tuple(edge)
        for n, e in [(n1, n2), (n2, n1)]:
            if n in self.graph:
                if e in self.graph[n]:
                    self.graph[n].remove(e)

    # Recorrido en profundidad (Depth First Search)
    # comienza en un nodo del grafo y
    # comprueba a todo lo largo de cada arco antes de retroceder
    def DFS(self, node):
        visited = [node]                    # lista de visitados
        stack = [node]                      # pila a tratar
        while stack:                        # mientras haya nodos en la pila
            current = stack.pop()           # sacar nodo de la pila
            if current not in visited:      # si no ha sido visitado
                visited.append(current)     # añadir a visitados
            for e in self.graph[current]:   # para cada nodo adyacente
                if e not in visited:        # si no ha sido visitado
                    stack.append(e)         # añadir a la pila
        return visited                      # devolver el recorrido

    # Recorrido en anchura (Breadth-First Search)
    # comienza en un nodo del grafo y
    # comprueba los nodos adyacentes en el nivel actual
    # antes de pasar al siguiente nivel del grafo
    def BFS(self, node):
        visited = [node]                    # lista de visitados
        queue = [node]                      # cola a tratar
        while queue:                        # mientras haya nodos en la cola
            current = queue.pop(0)          # sacar nodo de la cola
            if current not in visited:      # si no ha sido visitado
                visited.append(current)     # añadir a visitados
            for e in self.graph[current]:   # para cada nodo adyacente
                if e not in visited:        # si no ha sido visitado
                    queue.append(e)         # añadir a la cola
        return visited                      # devolver el recorrido      

    # Devuelve todos los caminos entre dos nodos
    def findPaths(self, start, end, path = []):
        if start not in self.graph or end not in self.graph:
            return []
        path = path + [start]
        if start == end:
            return [path]
        paths = []
        for node in self.graph[start]:
            if node not in path:
                newpaths = self.findPaths(node, end, path)
                for subpath in newpaths:
                    paths.append(subpath)
        return paths

    # Devuelve el camino más corto entre dos nodos
    # camino más corto == menos nodos
    def shortestPath(self, start, end):
        path = sorted(self.findPaths(start, end), key = len)
        return path[0] if path else []

    # Consulta si el grafo está vacío
    def isEmpty(self):
        return self.graph == {}

    # Consulta si el nodo existe en el grafo
    def existNode(self, node):
        return node in self.graph.keys()

    # Consulta si el arco existe en el grafo
    def existEdge(self, edge):
        n1, n2 = tuple(edge)
        return (n1, n2) in self.edges(n1) or (n2, n1) in self.edges(n2)

    def drawGraph(self):
        positions = nx.spring_layout(self.nxGraph)
        colors = list(nx.coloring.greedy_color(self.nxGraph).values())
        nx.draw_networkx_nodes(self.nxGraph, positions, node_color=colors)
        nx.draw_networkx_labels(self.nxGraph, positions)
        nx.draw_networkx_edges(self.nxGraph, positions, edgelist=self.nxGraph.edges)
        nx.draw_networkx_edge_labels(self.nxGraph, positions, edge_labels=nx.get_edge_attributes(self.nxGraph, 'weight'), font_weight='bold')

        plt.show()
