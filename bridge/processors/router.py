"""
Модуль маршрутизации роботов
Получает от стратега требуемые координаты для каждого робота и
считает оптимальный маршрут для достижения этой точки
"""

import bridge.processors.const as const
import bridge.processors.waypoint as wp
import bridge.processors.field as field
import bridge.processors.auxiliary as aux

import math

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
    def reRoute(self, field: field.Field):
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            
            if len(self.routes[i]) == 0:
                continue

            pth_wp = self.calcVectorField(i, field)
            if pth_wp is not None:
                self.routes[i].insert(-1, pth_wp)

    def calcVectorField(self, idx, field: field.Field):
        allies = field.b_team
        enemy = field.y_team
        self_pos = allies[idx].getPos()
        target_point = self.routes[idx][-1]

        sep_dist = 500
        ball_sep_dist = 150

        closest_robot = None
        closest_dist = math.inf
        closest_separation = 0
        angle_to_point = math.atan2(target_point.pos.y - self_pos.y, target_point.pos.x - self_pos.x)

        if target_point.type != wp.WType.IGNOREOBSTACLES:
            # Расчет теней роботов для векторного поля
            for r in field.all_bots:
                if r == self or r.is_used() == 0:
                    continue
                robot_separation = aux.dist(aux.closest_point_on_line(self_pos, target_point.pos, r.getPos()), r.getPos())
                robot_dist = aux.dist(self_pos, r.getPos())
                if robot_dist == 0:
                    continue
                if robot_separation < sep_dist and robot_dist < closest_dist:
                    closest_robot = r
                    closest_dist = robot_dist
                    closest_separation = robot_separation

            ball_separation = aux.dist(aux.closest_point_on_line(self_pos, target_point.pos, field.ball.getPos()), field.ball.getPos())
            ball_dist = aux.dist(self_pos, field.ball.getPos())
            if ball_separation < ball_sep_dist and ball_dist < closest_dist:
                closest_robot = field.ball
                closest_dist = ball_dist
                closest_separation = ball_separation

        passthrough_wp_pos = target_point.pos - self_pos
        # angle_cos = aux.scal_mult((closest_robot.getPos() - self_pos).unity(), (target_point.pos - self_pos).unity())
        if closest_robot is not None and closest_dist != 0:
            side = aux.vect_mult(closest_robot.getPos() - self_pos, target_point.pos - self_pos)
            # offset_angle_val = 200*math.pi/6 * closest_dist**(-2)
            offset_angle_val = -2 * math.atan((sep_dist - closest_separation)/(2*(closest_dist)))
            offset_angle = offset_angle_val if side < 0 else -offset_angle_val
            passthrough_wp_pos = self_pos + aux.rotate(target_point.pos - self_pos, offset_angle)

            passthrough_wp = wp.Waypoint(
                passthrough_wp_pos,
                0,
                wp.WType.PASSTHROUGH
            )
        else:
            passthrough_wp = None

        return passthrough_wp

    """
    Получить маршрут для робота с индексом idx
    """
    def getRoute(self, idx):
        return self.routes[idx]
