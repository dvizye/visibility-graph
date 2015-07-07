import unittest
from geo import *

class GeometryTest(unittest.TestCase):

    def setUp(self):
        pass

    def test_collides(self):
        poly = Polygon([Point(1,0), Point(2, 0), Point(1,1)])
        self.assertTrue(poly.contains(Point(1.5, 0.4)))
        self.assertFalse(poly.contains(Point(3, 0.5)))
        self.assertTrue(poly.contains(Point(1.5, 0.5), count_edges=True))
        self.assertFalse(poly.contains(Point(1.5, 0.5), count_edges=False))


    def test_polygon_line_intersect(self):
        e = Edge(Point(0,0), Point(2,1))
        poly = Polygon([Point(1,0), Point(2, 0), Point(1,1)])
        self.assertTrue(poly.collides(e))

    def test_collinear_line_intersect(self):
        a = Edge(Point(0,0), Point(0, 3))
        b = Edge(Point(0,1), Point(0, -1))
        self.assertTrue(a.intersect(b, count_endpoints=True))
        self.assertFalse(a.intersect(b, count_endpoints=False))
        
    def test_line_intersect(self):
        a = Edge(Point(1,1), Point(6, 6))
        b = Edge(Point(3,3), Point(5, 1))
        self.assertTrue(a.intersect(b, count_endpoints=True))
        self.assertFalse(a.intersect(b, count_endpoints=False))
        
    def test_ccw(self):
        A = Point(0, 0)
        B = Point(1, 0)
        C = Point(0.5, 0.5)
        line = Edge(A, B)
        self.assertTrue(line.ccw(C) > 0)

    def test_bitangent_intersect(self):
        A = Point(0, 0)
        B = Point(0, 5)
        C = Point(-1, 10)
        D = Point(1, 10)
        E = Point(5, 1)
        AB = Edge(A, B)
        CD = Edge(C, D)
        self.assertTrue(AB.bitangent_intersect(CD))
        self.assertFalse(AB.bitangent_intersect(Edge(D, E)))

    def test_intersect(self):
        A = Point(0, 0)
        B = Point(1, 0)
        C = Point(1, 1)
        D = Point(0, 1)
        ABCD = Polygon([A, B, C, D])
        AC = Edge(A, C)
        self.assertTrue(ABCD.collides(AC))

    def test_contains(self):
        A = Point(0, 0)
        B = Point(1, 0)
        C = Point(1, 1)
        D = Point(0, 1)
        ABCD = Polygon([A, B, C, D])
        AC = Edge(A, C)
        self.assertTrue(ABCD.contains(AC.midpoint()))
        
unittest.main()

