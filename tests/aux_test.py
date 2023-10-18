"""
Тест математики из auxiliary.py
"""
import unittest

from context import aux

class Testing(unittest.TestCase):
    """
    Класс-тестер
    """
    def test_line_intersect(self):
        """
        Тест get_line_intersection
        """
        p1 = aux.Point(0, 0)
        v1 = aux.Point(3, 0)
        p2 = aux.Point(2, -1)
        v2 = aux.Point(-1, 2)

        pi1 = aux.Point(1.5, 0)
        po1 = aux.get_line_intersection(p1, p1+v1, p2, p2+v2, 'LL')

        # fig, ax = plt.subplots()  # Create a figure containing a single axes.
        # ax.plot([p1.x, (p1+v1).x], [p1.y, (p1+v1).y])  # Plot some data on the axes.
        # ax.plot([p2.x, (p2+v2).x], [p2.y, (p2+v2).y])  # Plot some data on the axes.
        # ax.plot(po1.x, po1.y, 'x')
        # plt.show()
        # while(True):
        #     pass

        # print(po1)
        self.assertEqual(pi1.x, po1.x)
        self.assertEqual(pi1.y, po1.y)

    def test_poly_line_intersect(self):
        """
        Тест line_poly_intersect
        """
        p1 = aux.Point(-100, 0)
        p2 = aux.Point(100, 0)
        points = [aux.Point(-50, -50), aux.Point(-50, 50), aux.Point(50, 50), aux.Point(50, -50)]

        print(aux.line_poly_intersect(p1, p2, points))

    def test_is_point_inside_poly(self):
        """
        Тест is_point_inside_poly
        """
        p = aux.Point(0, 0)
        points = [aux.Point(-50, -50), aux.Point(-50, 50), aux.Point(50, 50), aux.Point(50, -50)]

        self.assertTrue(aux.is_point_inside_poly(p, points))

        p = aux.Point(-100, 0)
        self.assertFalse(aux.is_point_inside_poly(p, points))

        p = aux.Point(4000.00, -500.00)

        points = [
            aux.Point(4500.00, 1200.00),
            aux.Point(3300.00, 1200.00),
            aux.Point(3300.00, -1200.00),
            aux.Point(4500.00, -1200.00),
            aux.Point(10000.00, -0.00)
        ]

        # fig, ax = plt.subplots()  # Create a figure containing a single axes.
        # for i in range(-1, len(points)-1):
        #     ax.plot([points[i].x, points[i+1].x], [points[i].y, points[i+1].y])
        # # Plot some data on the axes.
        # ax.plot(p.x, p.y, 'x')
        # plt.show()

        self.assertTrue(aux.is_point_inside_poly(p, points))

    def test_is_in_list(self):
        """
        Тест is_in_list
        """

        lst = [1,2,3,4,5,6]
        x = 4
        self.assertTrue(aux.is_in_list(lst, x))

        x = 40
        self.assertFalse(aux.is_in_list(lst, x))


if __name__ == '__main__':
    unittest.main()
