"""
Модуль маршрутизации роботов
Получает от стратега требуемые координаты для каждого робота и
считает оптимальный маршрут для достижения этой точки
"""

import bridge.processors.const as const
import bridge.processors.waypoint as wp
import bridge.processors.field as field

class Router:
    def __init__(self) -> None:
        self.routes = [[] for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]

    """
    Установить единственную путевую точку для робота с индексом idx
    """
    def setWaypoint(self, idx, target: wp.Waypoint):
        self.routes[idx].clear()
        self.routes[idx].append(target)

    """
    Рассчитать маршруты по актуальным путевым точкам
    """
    def calcRoutes(self, field: field.Field):
        pass

    """
    Получить маршрут для робота с индексом idx
    """
    def getRoute(self, idx):
        return self.routes[idx]