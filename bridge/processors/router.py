"""
Модуль маршрутизации роботов
Получает от стратега требуемые координаты для каждого робота и
считает оптимальный маршрут для достижения этой точки
"""

import bridge.processors.const as const
import bridge.processors.waypoint as wp
import bridge.processors.field as field
import bridge.processors.auxiliary as aux
import bridge.processors.route as route

import math

class Router:
    def __init__(self, field: field.Field) -> None:
        self.routes = [route.Route(field.allies[i]) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]

    def update(self, field: field.Field):
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            self.routes[i].update(field.allies[i])
    
    """
    Установить единственную путевую точку для робота с индексом idx
    """
    def setDest(self, idx, target: wp.Waypoint):
        self.routes[idx].setDestWP(target)

    """
    Рассчитать маршруты по актуальным путевым точкам
    """
    def reRoute(self, field: field.Field):
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            
            if not self.routes[i].isUsed():
                continue

            if self.routes[i].getNextType() == wp.WType.S_BALL_KICK:
                if not field.allies[i].is_kick_aligned(self.routes[i].getDestWP()):
                    align_wp = self.calcKickWP(i, field)
                    self.routes[i].insertWP(align_wp)


            
            pth_wp = self.calcVectorField(i, field)
            if pth_wp is not None:
                self.routes[i].insertWP(pth_wp)
            
    # def calcPenDetour(self, idx, field: field.Field):
    #     self_pos = field.allies[idx].getPos()
    #     detwp = None
    #     for goal in [*field.ally_goal, *field.enemy_goal]:
    #         vec1 = aux.vect_mult(self.routes[idx].getNextVec().unity(), (goal.forwup - self_pos).unity())
    #         vec2 = aux.vect_mult(self.routes[idx].getNextVec().unity(), (goal.forwdown - self_pos).unity())
    #         vecc = aux.vect_mult(self.routes[idx].getNextVec().unity(), (goal.center - self_pos).unity())
    #         if(aux.sign(vec1) != aux.sign(vecc)):
    #             if(aux.sign(vec2) != aux.sign(vecc)):
    #                 detwp = 


    def calcVectorField(self, idx, field: field.Field):
        allies = field.allies
        enemy = field.enemies
        self_pos = allies[idx].getPos()
        target_point = self.routes[idx].getNextWP()
        dist = (self_pos - target_point.pos).mag()

        vector_field_threshold = 200
        if dist < vector_field_threshold:
            return None

        if self.routes[idx].getDestWP().type == wp.WType.S_IGNOREOBSTACLES:
            return None

        sep_dist = 500
        ball_sep_dist = 150

        if self.routes[idx][-1].type == wp.WType.KEEP_BALL_DISTANCE:
            ball_sep_dist = 700

        closest_robot = None
        closest_dist = dist
        closest_separation = 0
        angle_to_point = math.atan2(target_point.pos.y - self_pos.y, target_point.pos.x - self_pos.x)

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
                wp.WType.R_PASSTHROUGH
            )
        else:
            passthrough_wp = None

        return passthrough_wp

    def calcKickWP(self, idx, field: field.Field):
        target_point = self.routes[idx].getDestWP()
        target_pos = target_point.pos
        target_angle = target_point.angle

        align_pos = target_pos - aux.rotate(aux.i, target_angle)*const.KICK_ALIGN_DIST
        align_angle = target_angle
        align_type = wp.WType.R_BALL_ALIGN
        align_wp = wp.Waypoint(
            align_pos,
            align_angle,
            align_type
        )
        return align_wp

    """
    Получить маршрут для робота с индексом idx
    """
    def getRoute(self, idx):
        return self.routes[idx]
