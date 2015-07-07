from geo import *
from math import sqrt, cos, sin, atan2, pi
from operator import add
from collections import defaultdict
from djikstra import *

class World():
    def __init__(self, robot, obstacles, goal, window=None):
        self.robot = robot
        self.obs = obstacles
        self.goal = goal
        if window:
            self.window = window
        else:
            self.window = DrawingWindow(600, 600, 0, 600, 0, 600, 'test')
        cspace_obs = [o.cspace_object(self.robot) for o in self.obs]
        self.cspace_polys = []
        for o in cspace_obs:
            self.cspace_polys.extend(o.polygons)

    def visible(self, start, end):
        e = Edge(start, end)
        return not any([p.collides(e) for p in self.cspace_polys])

    def bitangent(self, A, B):
        e = Edge(A, B)
        for p in self.cspace_polys:
            for edge in p.edges:
                if e.bitangent_intersect(edge, count_edges=True):
                    return False
        return True

    def build_visibility_graph(self):
        graph = defaultdict(list)
        candidate_vertices = []
        for p in self.cspace_polys:
            for edge in p.edges:
                candidate_vertices.append(edge.start)
                candidate_vertices.append(edge.end)
        for i in range(len(candidate_vertices)):
            for j in range(i+1, len(candidate_vertices)):
                vertex1 = candidate_vertices[i]
                vertex2 = candidate_vertices[j]
                vertex1 = Point(vertex1.x, vertex1.y)
                vertex2 = Point(vertex2.x, vertex2.y)
                if self.visible(vertex1, vertex2):
                    graph[vertex1].append(vertex2)
        start = self.robot.ref_point
        for i in candidate_vertices:
            if self.visible(start, i):
                graph[start].append(i)

        for i in candidate_vertices:
            if self.visible(i, self.goal):
                graph[i].append(self.goal)
        self.graph = graph
    
    def display_graph(self):
        for p in self.cspace_polys:
            self.window.drawPolygon(p.points, color='red')
        for a in self.graph:
            for b in self.graph[a]:
                Edge(a, b).draw(self.window, color='blue')
        self.goal.draw(self.window, color='green')
        self.robot.draw(self.window, color='yellow')
        raw_input('Enter to close')


    def graph_to_weighted_graph(self):
        graph = defaultdict(list)
        for i in self.graph:
            for j in self.graph[i]:
                e = Edge(i, j)
                cost = (e.vector.magnitude())
                graph[i].append((cost, j))
        return graph

    def djikstra(self, color='green'):
        graph = self.graph_to_weighted_graph()
        (cost, edges) = djikstra(graph, self.robot.ref_point, self.goal)
        for e in edges:
            e.draw(self.window, color=color)
