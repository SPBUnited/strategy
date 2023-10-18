"""
Тест модуля динамических звеньев tau.py
"""
import unittest
import matplotlib.pyplot as plt

from context import tau

class Testing(unittest.TestCase):
    """
    Класс-тест
    """
    def test_rate_limiter(self):
        """
        Тест ограничителя скорости роста
        """

        ts = 0.01
        t = tau.Integrator(ts)

        rl = tau.RateLimiter(ts, 0.1)

        t_end = 10

        t_data = []
        in_data = []
        out_data = []

        while t.get_val() < t_end:
            t_data.append(t.get_val())

            u = -0.5 + 1/(t.get_val()+1)
            out = rl.process(u)

            in_data.append(u)
            out_data.append(out)

            t.process(1)

        _, ax = plt.subplots()  # Create a figure containing a single axes.
        ax.plot(t_data, in_data)
        ax.plot(t_data, out_data)
        plt.show()
        self.assertEqual(1, 1)


if __name__ == '__main__':
    unittest.main()
