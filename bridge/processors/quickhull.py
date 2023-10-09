"""
Реализация алгоритма quickhull для построения наименьшей выпуклой оболочки
"""

import bridge.processors.auxiliary as aux
import math

def shortesthull(p1: aux.Point, p2: aux.Point, points):
    """
    Получить кратчайший путь от точки p1 до точки p2, огибающий точки points
    """
    hull = []
    mindist = math.inf
    minidx = None
    for i in [-1, 1]:
        hull.append(quickhull(p1, p2, points, i))
        dist = 0
        last_wp_pos = hull[-1][0]
        for wpt in hull[-1]:
            dist += (wpt - last_wp_pos).mag()
            last_wp_pos = wpt
        if dist < mindist:
            mindist = dist
            minidx = i
    if minidx == 1:
        return hull[1]
    else:
        return hull[0]
        


def quickhull(p1: aux.Point, p2: aux.Point, points, polarity = 1):
    """
    Найти половину выпуклой оболочки вокруг points с началом в p1, концом в p2

    При polarity == 1: оболочка сверху
    При polarity == -1: оболочка снизу
    """
    if polarity == 1:
        return [p1] + quickhullupper(p1, p2, points) + [p2]
    elif polarity == -1:
        return list(reversed(([p2] + quickhullupper(p2, p1, points) + [p1])))
    else:
        return None

def quickhullupper(p1: aux.Point, p2: aux.Point, points):
    """
    Найти верхнюю половину выпуклой оболочки вокруг points с началом в p1, концом в p2
    """
    if len(points) == 0:
        return []

    vec = p2-p1
    uppoints = []

    for p in points:
        if aux.vect_mult(vec, p - p1) > 0:
            uppoints.append(p)

    max_dist = 0
    max_p = None
    for p in uppoints:
        if aux.dist2line(p2, p1, p) > max_dist:
            max_dist = aux.dist2line(p2, p1, p)
            max_p = p
        
    if max_p is None:
        return []
    else:
        return quickhullupper(p1, max_p, uppoints) + [max_p] + quickhullupper(max_p, p2, uppoints)
