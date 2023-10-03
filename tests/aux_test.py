import unittest
import matplotlib.pyplot as plt
import numpy as np

import matplotlib as mpl

import sys

sys.path.insert(0, '/home/arxi/SSL23/strategy/')

import time
import math
import bridge.processors.const as const
import bridge.processors.robot as robot
import bridge.processors.auxiliary as aux
import bridge.processors.field as field
import bridge.processors.router as router
import bridge.processors.strategy as strategy
import bridge.processors.waypoint as wp
import bridge.processors.drawer as drw


class Testing(unittest.TestCase):
    def test_line_intersect(self):
        p1 = aux.Point(0, 0)
        v1 = aux.Point(3, 1)
        p2 = aux.Point(2, -1)
        v2 = aux.Point(-1, 2)/3

        pi1 = aux.Point(2, 0)
        po1 = aux.get_line_intersection(p1, p1+v1, p2, p2+v2, 'LL')

        fig, ax = plt.subplots()  # Create a figure containing a single axes.
        ax.plot([p1.x, (p1+v1).x], [p1.y, (p1+v1).y])  # Plot some data on the axes.
        ax.plot([p2.x, (p2+v2).x], [p2.y, (p2+v2).y])  # Plot some data on the axes.
        ax.plot(po1.x, po1.y, 'x')
        plt.show()
        # while(True):
        #     pass
        
        # print(po1)
        self.assertEqual(pi1.x, po1.x)
        self.assertEqual(pi1.y, po1.y)


if __name__ == '__main__':
    unittest.main()
    # test_line_intersect()
