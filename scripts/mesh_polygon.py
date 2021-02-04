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
        for neighbor_idx in its_neighbors:
            self.neighbors[neighbor_idx].update(its_neighbors)
            self.neighbors[neighbor_idx].discard(node)
            self.neighbors[neighbor_idx].discard(neighbor_idx)

        if node == self.entry_node:
            self.entry_node = its_neighbors.pop()

        self.nodes.discard(node)

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
        self.index_angle = {}
        self.index_buffer = []

        for i in range(len(vertices) - 1):
            self.index_graph.addEdge(i, i+1)
        self.index_graph.addEdge(len(vertices) - 1, 0)

        for i in range(len(self.vertex_buffer)):
            self.updateAngle(i)

        # self.index_angle.sort()

    def updateAngle(self, vertex_idx):
        neighbors_idx = list(self.index_graph.neighbors[vertex_idx])
        print("{},{}".format(neighbors_idx, vertex_idx))
        p0, p1, p2 = self.vertex_buffer[neighbors_idx[0]], self.vertex_buffer[vertex_idx], self.vertex_buffer[neighbors_idx[1]]
        # self.index_angle.append((i, angle(p0, p1, p2)))
        self.index_angle[vertex_idx] = angle(p0, p1, p2)


    def edgeForPlot(self):
        edges = dfsEdge(self.index_graph)

        point_pairs = [
            ((self.vertex_buffer[e[0]][0], self.vertex_buffer[e[1]][0]),
            (self.vertex_buffer[e[0]][1], self.vertex_buffer[e[1]][1]) )
            for e in edges]

        return point_pairs

    def triangleForPlot(self):
        # [self.vertex_buffer[e[i]][0] for e in self.index_buffer for i in range(-1, 3)],
        #     [self.vertex_buffer[e[i]][1] for e in self.index_buffer for i in range(-1, 3)]
        point_pairs = []
        for triangle in self.index_buffer:
            triangle_pair = []
            for xy in [0,1]:
                triangle_pair.append([self.vertex_buffer[triangle[idx]][xy] for idx in range(-1, 3)])
            point_pairs.append(triangle_pair)
        return point_pairs

    def cutCorner(self):
        if len(self.index_graph.nodes) == 0:
            return False

        if len(self.index_graph.nodes) == 3:
            self.index_buffer.append(tuple(self.index_graph.nodes))
            self.index_graph.nodes.clear()
            self.index_angle.clear()
            return True

        min_vertex_idx = self.index_graph.entry_node
        min_angle = self.index_angle[min_vertex_idx]

        for idx in self.index_angle:
            if self.index_angle[idx] < min_angle:
                min_angle = self.index_angle[idx]
                min_vertex_idx = idx

        p0, p2 = list(self.index_graph.neighbors[min_vertex_idx])[:2]

        self.index_buffer.append((p0, min_vertex_idx, p2))
        self.index_angle.pop(min_vertex_idx)
        self.index_graph.removeNode(min_vertex_idx)
        self.updateAngle(p0)
        self.updateAngle(p2)

        print("remove: {}".format(min_vertex_idx))
        return True

if __name__ == "__main__":
    debug = True
    vertex_buffer = np.array([(0,0), (1,0), (1,0.5), (2,3), (1,2), (-1, 0.5)])
    # vertex_buffer = np.array([[np.cos(t), np.sin(t)] for t in np.linspace(0, 2 * np.pi, 100)])

    mesh = MeshPolygon(vertex_buffer)

    point_pairs = mesh.edgeForPlot()
    for pair in point_pairs:
        plt.plot(pair[0], pair[1])

    plt.axis('equal')
    plt.show()

    while mesh.cutCorner():
        if debug:
            point_pairs = mesh.triangleForPlot()

            for pair in point_pairs:
                plt.plot(pair[0], pair[1])

            point_pairs = mesh.edgeForPlot()
            for pair in point_pairs:
                plt.plot(pair[0], pair[1])

            plt.axis('equal')
            plt.show()

    point_pairs = mesh.triangleForPlot()

    for pair in point_pairs:
        plt.plot(pair[0], pair[1])

    plt.axis('equal')
    plt.show()



