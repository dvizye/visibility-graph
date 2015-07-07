from DrawingWindowStandalone import *
from math import sqrt, cos, sin, atan2, pi
from operator import add

def normalize_angle(angle):
    angle = angle - int(angle / (2*pi)) * 2*pi
    return angle - 2*pi*(angle > pi) + 2*pi*(angle < -1*pi)

class Coord():
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def dot(self, other):
        return self.x*other.x + self.y*other.y
    def  __sub__(self,elf, other):
        return Coord(self.x - other.x, self.y - other.y)
    def __str__(self):
        return "Coord(" + str(self.x) + " , " + str(self.y)  + ")"
    def __add__(self, other):
        return Coord(other.x + self.x, other.y + self.y)
    def __eq__(self, other):
        return (other.x, other.y) == (self.x, self.y)
    def normal(self):
        return Coord(-self.y, self.x)
    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)
    def __hash__(self):
        return str(self).__hash__()

class Vector(Coord):
    def __init__(self, start, end):
        Coord.__init__(self, end.x - start.x, end.y - start.y)
    
    def __str__(self):
        return "[" + str(self.x) + " , " + str(self.y) + "]"

class Point(Coord):
    def __str__(self):
        return "Point(" + str(self.x) + " , " + str(self.y)  + ")"
    def draw(self, window, color):
        window.drawPoint(self.x, self.y, color=color)

class Edge():
    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.vector = Vector(self.start, self.end)
    
    def midpoint(self):
        return Point( (self.start.x + self.end.x) / 2.0, \
                (self.start.y + self.end.y) / 2.0)

    def __str__(self):
        return "(" + str(self.start) + " , " + str(self.end) + ")"

    def invert(self):
        return Edge(self.end, self.start)

    def normal_angle(self, direction="clockwise"):
        """
        Return: Angle of vector normal to edge
        """
        normal = self.vector.normal()
        if direction == "clockwise":
            return normalize_angle(atan2(normal.y, normal.x) + pi)
        else:
            return atan2(normal.y, normal.x)
    
    def ccw(self, point):
        (A, B, C) = (self.start, self.end, point)
        normal = Vector(A, B).normal()
        BC = Vector(B, C)
        if BC.dot(normal) == 0:
            return 0
        return 1 if BC.dot(normal) > 0 else -1

    def draw(self, window, color):
        window.drawLineSeg(self.start.x, self.start.y, self.end.x, self.end.y, color=color)

    def _on_segment(self, point):
        (A, B, C) = (self.start, self.end, point)
        if self.vector.normal().dot(Vector(A, C)) != 0:
            return False
        return min(A.x, B.x) <= C.x <= max(A.x, B.x) and \
                min(A.y, B.y) <= C.y <= max(A.y, B.y)

    def bitangent_intersect(self, other, count_edges=False):
        [A, B, C, D] = [self.start, self.end, other.start, other.end]
        if self.ccw(C) * self.ccw(D) == -1:
            return True
        if count_edges:
            return self.ccw(C) * self.ccw(D) * other.ccw(A) * other.ccw(D) == 0
        return False

    def intersect(self, other, count_endpoints=False):
        assert(isinstance(other, Edge))
        [A, B, C, D] = [self.start, self.end, other.start, other.end]
        if self.ccw(C) * self.ccw(D) == -1 and other.ccw(A) * other.ccw(B) == -1:
            return True
        if count_endpoints:
            if self._on_segment(D) \
                    or self._on_segment(C) \
                    or other._on_segment(A) \
                    or other._on_segment(B):
                        return True
        return False

class Polygon():
    def __init__(self, points):
        self.points = points
        # self.edges = [Edge(points[i], points[(i+1) % len(points)]) for i in range(len(points))]
        #Edges are oriented counterclockwise, if points are spec in clockwise order
        self.edges = [Edge(points[i], points[(i-1)]) for i in range(len(points))]
    
    def translate(self, vector):
        return Polygon([p + vector for p in self.points])
    
    def collides(self, other, count_edges=False):
        if isinstance(other, Edge):
            if self.contains(other.start) \
                    or self.contains(other.midpoint()) \
                    or self.contains(other.end):
                return True
            return any([i.intersect(other) for i in self.edges])

    def contains(self, point, count_edges=False):
        if all([i.ccw(point) == -1 for i in self.edges]):
            return True
        if count_edges and any([i.ccw(point) == 0 for i in self.edges]):
            return True
        return False

    def __str__(self):
        output = ' , '
        # output = output.join([str(p) for p in self.points])
        output = output.join([str(p) for p in self.points[-1::-1]])
        return "Polygon ([" + output + "])"

    def __eq__(self, other):
        return all([p in other.points for p in self.points]) and all([p in self.points for p in other.points]) \
                and isinstance(other, Polygon)

    def cspace_polygon(self, robot):
        """
        Calculates the cspace footprint of self, given the geometry of robot
        """
        ref_point = robot.points[0]
        self_angles = [(e.normal_angle(), 'obs', e) for e in self.edges]
        self_angles.sort()
        robot_angles = [(e.normal_angle("counterclockwise"), 'robot', e.invert()) for e in robot.edges]
        robot_angles.sort()
        angles = (self_angles + robot_angles)
        angles.sort()
        # Get start point
        _, _, start_edge = self_angles[0]
        start_index = angles.index(self_angles[0])
        wrapped_angles = angles[start_index::] + angles[0:start_index]
        first_robot_edge = None
        last_edge = start_edge
        cspace_polygon_points = [start_edge.end]
        for _, type, edge in wrapped_angles[1::]:
            if first_robot_edge == None and type == 'robot':
                first_robot_edge = edge
            cspace_polygon_points.append(cspace_polygon_points[-1] + edge.vector)
        cspace_polygon = Polygon(cspace_polygon_points)
        translate_vector = first_robot_edge.invert().vector + Vector(first_robot_edge.start, ref_point)
        cspace_polygon = cspace_polygon.translate(translate_vector)
        return cspace_polygon

    def draw(self, window, color='blue'):
        window.drawPolygon(self.points, color=color)

class Object():
    def __init__(self, ref_point, polygons):
        self.polygons = [p.translate(Vector(Point(0,0), ref_point)) \
                for p in polygons]
        self.ref_point = ref_point

    def cspace_object(self, robot):
        c_polys = []
        for p in robot.polygons:
            for q in self.polygons:
                c_poly = q.cspace_polygon(p)
                c_poly = c_poly.translate(Vector(p.points[0], robot.ref_point))
                c_polys.append(c_poly)
        return Object(Point(0,0), c_polys)                
    
    def draw(self, window, color='blue'):
        for p in self.polygons:
            window.drawPolygon(p.points, color=color)

    def __eq__(self, other):
        return isinstance(other, Object) and \
                all([p in other.polygons for p in self.polygons]) and \
                all([q in self.polygons for q in other.polygons])

    def __str__(self):
        output = ' , '
        output = output.join([str(p) for p in self.polygons])
        return 'Object(' + str(self.ref_point) + ', [' + output + '])'
