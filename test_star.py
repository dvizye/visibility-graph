from geo import *
# from geometry import *
from DrawingWindowStandalone import *
import unittest
from visibility_graph import *
from djikstra import *

DRAW=True

class TestVisibilityGraph(unittest.TestCase):
    def setUp(self):
        if DRAW == True:
            self.window = DrawingWindow(600, 600, 0, 600, 0, 600, 'test')

    def drawWorld(self, robot, obj, obs):
        if DRAW:
            if robot:
                robot.draw(self.window, color='yellow')
            if obj:
                obj.draw(self.window)
            if obs:
                obs.draw(self.obs, color='red')
            raw_input('Press Enter to close window')

    def test_cspace_object(self):
        robot = Object(Point(20, 40), 
                [Polygon([Point(0, 0), Point(30, 30), Point(10, -20)])])
        obs = Object(Point(20,20), 
                [Polygon([Point(100, 100), Point(100, 200), Point(300, 200), Point(300, 100)])])
        cspace_obj = obs.cspace_object(robot)
        if DRAW == True:
            cspace_obj.draw(self.window)
            robot.draw(self.window,color='yellow')
            obs.draw(self.window, color='red')
            raw_input('Press Enter to close window')
        expected = Object(Point(20,20),
            [Polygon([Point(70 , 70), Point(270 , 70) , Point(300 , 100) , Point(300 , 200) , Point(290 , 220) , Point(90 , 220) , Point(70 , 170)])])
        self.assertEqual(cspace_obj, expected)

    def test_multiple_poly(self):
        robot = Object(Point(20, 40), 
                [Polygon([Point(0, 0), Point(30, 30), Point(10, -20)]), 
                    Polygon([Point(0, 0), Point(10, -20), Point(10, -30), Point(0, -30)])])
        obs = Object(Point(20,20), 
                [Polygon([Point(100, 100), Point(100, 200), Point(300, 200), Point(300, 100)])])
        cspace_obj = obs.cspace_object(robot)
        if DRAW:
            cspace_obj.draw(self.window)
            robot.draw(self.window,color='yellow')
            obs.draw(self.window, color='red')
            raw_input('Press Enter to close window')
        expected = Object(Point(0 , 0), [Polygon ([Point(90 , 90) , Point(290 , 90) , Point(320 , 120) , Point(320 , 220) , Point(310 , 240) , Point(110 , 240) , Point(90 , 190) , Point(90 , 90)]) , Polygon ([Point(120 , 120) , Point(320 , 120) , Point(320 , 220) , Point(320 , 250) , Point(120 , 250) , Point(110 , 250) , Point(110 , 150) , Point(110 , 140) , Point(120 , 120)])])
        self.assertEqual(cspace_obj, expected)

    def test_vis_graph(self):
        robot = Object(Point(20, 40), 
                [Polygon([Point(0, 0), Point(30, 30), Point(10, -20)])])
        obs = Object(Point(20,20), 
                [Polygon([Point(100, 100), Point(100, 200), Point(300, 200), Point(300, 100)])])
        goal = Point(300, 300)
        cspace_obj = obs.cspace_object(robot)
        world = World(robot, [obs], goal, self.window)
        world.build_visibility_graph()

        if DRAW == True:
            world.display_graph()
            raw_input('Press Enter to close window')
        expected = Object(Point(20,20),
            [Polygon([Point(70 , 70), Point(270 , 70) , Point(300 , 100) , Point(300 , 200) , Point(290 , 220) , Point(90 , 220) , Point(70 , 170) , Point(70 , 70)])])
        self.assertEqual(cspace_obj, expected)

    def test_djikstra(self):
        robot = Object(Point(20, 40), 
                [Polygon([Point(0, 0), Point(30, 30), Point(10, -20)])])
        obs = Object(Point(20,20), 
                [Polygon([Point(100, 100), Point(100, 200), Point(300, 200), Point(300, 100)])])
        goal = Point(300, 300)
        cspace_obj = obs.cspace_object(robot)
        world = World(robot, [obs], goal)
        world.build_visibility_graph()
        graph = world.graph_to_weighted_graph()
        (cost, edges) = djikstra(graph, robot.ref_point, goal)
        if DRAW == True:
            world.display_graph()
            for e in edges:
                e.draw(self.window, color='green')
            raw_input('Press Enter to close window')
        expected = Object(Point(20,20),
            [Polygon([Point(70 , 70), Point(270 , 70) , Point(300 , 100) , Point(300 , 200) , Point(290 , 220) , Point(90 , 220) , Point(70 , 170) , Point(70 , 70)])])
        self.assertEqual(cspace_obj, expected)

    def test_multiple_poly_djikstra(self):
        robot = Object(Point(20, 40), 
                [Polygon([Point(0, 0), Point(30, 30), Point(10, -20)]), 
                    Polygon([Point(0, 0), Point(10, -20), Point(10, -30), Point(0, -30)])])
        obs = Object(Point(20,20), 
                [Polygon([Point(100, 100), Point(100, 200), Point(300, 200), Point(300, 100)])])
        cspace_obj = obs.cspace_object(robot)
        goal = Point(300, 300)
        world = World(robot, [obs], goal, self.window)
        world.build_visibility_graph()
        graph = world.graph_to_weighted_graph()
        (cost, edges) = djikstra(graph, robot.ref_point, goal)
        self.assertAlmostEqual(cost, 424.94202022)
        if DRAW:
            world.djikstra()
            world.display_graph()

unittest.main()
