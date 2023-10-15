import sys

sys.path.insert(0, '/home/arxi/SSL23/strategy/')

import time
import math
from context import tau

import unittest
import matplotlib.pyplot as plt
import numpy as np

import matplotlib as mpl

class Testing(unittest.TestCase):
    def test_rate_limiter(self):
        
        ts = 0.01
        t = tau.Integrator(ts)

        rl = tau.RateLimiter(ts, 0.1)

        t_end = 10

        t_data = []
        in_data = []
        out_data = []

        while t.getVal() < t_end:
            t_data.append(t.getVal())

            u = -0.5 + 1/(t.getVal()+1)
            out = rl.process(u)

            in_data.append(u)
            out_data.append(out)

            t.process(1)            

        fig, ax = plt.subplots()  # Create a figure containing a single axes.
        ax.plot(t_data, in_data)
        ax.plot(t_data, out_data)
        plt.show()
        self.assertEqual(1, 1)


if __name__ == '__main__':
    unittest.main()
    # test_line_intersect()

