import matplotlib.pyplot as plt
import numpy as np

class PolygonGraph:
    def __init__(self, entry_node):
        self.entry_node = entry_node
        self.neighbors = {entry_node: set()}
        self.nodes = set([entry_node])

    def addEdge(self, node0, node1):
        node_pair = [node0, node1]
        for i in [0, 1]:
            this_node, that_node = node_pair[i], node_pair[1-i]
            if this_node not in self.neighbors:
                self.neighbors[this_node] = set()
            self.neighbors[this_node].add(that_node)
            self.nodes.add(this_node)

    def removeNode(self, node):
        its_neighbors = self.neighbors[node]
        self.neighbors.pop(node)
        for node in its_neighbors:
            self.neighbors[node].update(its_neighbors)
            self.neighbors[node].discard(node)

        if node == self.entry_node:
            self.entry_node = its_neighbors.pop()

def angle(p0, p1, p2):
    v1, v2 = p0 - p1, p2 - p1
    theta = np.arccos(v1.dot(v2) / np.linalg.norm(v1) / np.linalg.norm(v2))
    if np.cross(v1, v2) > 0:
        return 2 * np.pi - theta
    else:
        return theta

def dfsEdge(graph, root=None, visited=None):
    """
    recursive version
    """
    seq = []

    root = graph.entry_node if root is None else root
    visited = set() if visited is None else visited
    for node in graph.neighbors[root]:
        edge = tuple(sorted([root, node]))
        if edge in visited:
            continue
        visited.add(edge)
        seq.append(edge)
        seq += dfsEdge(graph, node, visited)

    return seq

class MeshPolygon:
    def __init__(self, vertices):
        self.vertex_buffer = vertices
        self.index_graph = PolygonGraph(0)
        self.index_angle = []
        self.index_buffer = []

        for i in range(len(vertices) - 1):
            self.index_graph.addEdge(i, i+1)
        self.index_graph.addEdge(len(vertices) - 1, 0)

        for i in range(len(self.vertex_buffer)):
            neighbors_idx = list(self.index_graph.neighbors[i])
            print("{},{}".format(neighbors_idx, i))
            p0, p1, p2 = self.vertex_buffer[neighbors_idx[0]], self.vertex_buffer[i], self.vertex_buffer[neighbors_idx[1]]
            self.index_angle.append((i, angle(p0, p1, p2)))

        self.index_angle.sort()

    def edgeForPlot(self):
        edges = dfsEdge(self.index_graph)

        point_pairs = [
            ((self.vertex_buffer[e[0]][0], self.vertex_buffer[e[1]][0]),
            (self.vertex_buffer[e[0]][1], self.vertex_buffer[e[1]][1]) )
            for e in edges]

        return point_pairs

    def cutCorner(self):
        if len(self.index_graph.nodes) == 3:
            return False

        return True

if __name__ == "__main__":

    vertex_buffer = np.array([(0,0), (1,0), (1,0.5), (2,3), (1,2), (-1, 0.5)])
    mesh = MeshPolygon(vertex_buffer)
    point_pairs = mesh.edgeForPlot()
    for pair in point_pairs:
        plt.plot(pair[0], pair[1])

    plt.show()
