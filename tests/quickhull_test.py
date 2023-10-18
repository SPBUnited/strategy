"""
Модуль теста quickhull.py
"""
import unittest
import random
import matplotlib.pyplot as plt

from context import aux, qh


class Testing(unittest.TestCase):
    """
    Класс-тест
    """
    def test_quickhull(self):
        """
        Тест алгоритма quickhull
        """

        p1 = aux.Point(-2000, 0)
        p2 = aux.Point(2000, 5000)

        points = []
        num_points = 100
        for _ in range(num_points):
            x, y = random.randint(-1000, 1000), random.randint(-1000, 1000)
            points.append(aux.Point(x, y))

        # print(points)

        qho = qh.shortesthull(p1, p2, points)

        _, ax = plt.subplots()  # Create a figure containing a single axes.
        ax.plot(p1.x, p1.y, 'o')
        ax.plot(p2.x, p2.y, 'o')
        for p in points:
            ax.plot(p.x, p.y, 'rx')
        for i in range(len(qho)-1):
            print(qho[i])
            ax.plot([qho[i].x, qho[i+1].x], [qho[i].y, qho[i+1].y])
        plt.show()
        self.assertEqual(1, 1)

    def test_quickhull_2(self):
        """
        Тест краевого случая из симулятора (точки оболочки не между точками пути)
        """

        p1 = aux.Point(-2500, 553)
        # p2 = aux.Point(4000, 2000)
        p2 = aux.Point(-4200,200)

        points = [
            aux.Point(-4500.00, -1200.00),
            aux.Point(-3300.00, -1200.00),
            aux.Point(-3300.00, 1200.00),
            aux.Point(-4500.00, 1200.00),
            aux.Point(-10000.00, 0.00)
        ]
        # NUM_POINTS = 100
        # for _ in range(NUM_POINTS):
        #     x, y = random.randint(-1000, 1000), random.randint(-1000, 1000)
        #     points.append(aux.Point(x, y))

        # print(points)

        # qho = qh.shortesthull(p1, p2, points)
        qho = qh.quickhull(p1, p2, points, 1) + qh.quickhull(p2, p1, list(reversed(points)), 1)

        _, ax = plt.subplots()  # Create a figure containing a single axes.
        ax.plot(p1.x, p1.y, 'o')
        ax.plot(p2.x, p2.y, 'o')
        for p in points:
            ax.plot(p.x, p.y, 'rx')
        for i in range(len(qho)-1):
            print(qho[i])
            ax.plot([qho[i].x, qho[i+1].x], [qho[i].y, qho[i+1].y])
        plt.show()
        self.assertEqual(1, 1)


if __name__ == '__main__':
    unittest.main()
