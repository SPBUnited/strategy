"""
Модуль маршрутизации роботов
Получает от стратега требуемые координаты для каждого робота и
считает оптимальный маршрут для достижения этой точки
"""

from typing import Optional

import bridge.auxiliary.quickhull as qh
import bridge.router.waypoint as wp
from bridge import const
from bridge.auxiliary import aux, fld
from bridge.auxiliary.entity import Entity
from bridge.router import route


class Router:
    """
    Маршрутизатор
    """

    def __init__(self, field: fld.Field) -> None:
        """
        Конструктор
        """
        self.routes = [route.Route(field.allies[i]) for i in range(const.TEAM_ROBOTS_MAX_COUNT)]
        self.__avoid_ball = False

    def avoid_ball(self, state: bool = True) -> None:
        """
        Stop game state
        """
        self.__avoid_ball = state

    def update(self, field: fld.Field) -> None:
        """
        Обновить маршруты актуальным состоянием поля
        """
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            self.routes[i].update(field.allies[i])

    def get_route(self, idx: int) -> route.Route:
        """
        Получить маршрут для робота с индексом idx
        """
        return self.routes[idx]

    def set_dest(self, idx: int, target: wp.Waypoint, field: fld.Field) -> None:
        """
        Установить единственную путевую точку для робота с индексом idx
        """
        if idx != field.gk_id and target.type != wp.WType.R_IGNORE_GOAl_HULL:
            dest_pos = target.pos
            for goal in [field.ally_goal, field.enemy_goal]:
                if aux.is_point_inside_poly(dest_pos, goal.big_hull):
                    closest_out = aux.nearest_point_on_poly(dest_pos, goal.big_hull)
                    angle0 = target.angle
                    self.routes[idx].set_dest_wp(wp.Waypoint(closest_out, angle0, wp.WType.S_ENDPOINT))
                    return

        if self.__avoid_ball:
            dest_pos = target.pos
            if aux.is_point_inside_circle(dest_pos, field.ball.get_pos(), const.KEEP_BALL_DIST):
                delta = -aux.rotate(aux.RIGHT, target.angle)
                closest_out = aux.nearest_point_on_circle(
                    dest_pos + delta,
                    field.ball.get_pos(),
                    const.KEEP_BALL_DIST + const.ROBOT_R,
                )
                angle0 = target.angle
                self.routes[idx].set_dest_wp(wp.Waypoint(closest_out, angle0, wp.WType.S_ENDPOINT))
                return

        if abs(target.pos.x) > const.GOAL_DX:
            target.pos.x = const.GOAL_DX * aux.sign(target.pos.x)
        if abs(target.pos.y) > 3000:
            target.pos.y = 3000 * aux.sign(target.pos.y)
        self.routes[idx].set_dest_wp(target)

    def reroute(self, field: fld.Field) -> None:
        """
        Рассчитать маршруты по актуальным путевым точкам
        """

        for idx in range(const.TEAM_ROBOTS_MAX_COUNT):

            self_pos = field.allies[idx].get_pos()

            if not self.routes[idx].is_used() or not field.allies[idx].is_used():
                continue

            if self.routes[idx].get_next_type() == wp.WType.S_VELOCITY:
                continue

            if (
                self.routes[idx].get_next_type() == wp.WType.S_BALL_KICK
                or self.routes[idx].get_next_type() == wp.WType.S_BALL_KICK_UP
                or self.routes[idx].get_next_type() == wp.WType.S_BALL_PASS
            ):
                # if not field.allies[idx].is_kick_aligned(self.routes[idx].get_dest_wp()):
                align_wp = self.calc_kick_wp(idx)
                self.routes[idx].insert_wp(align_wp)
            elif self.routes[idx].get_next_type() == wp.WType.S_BALL_GRAB:
                align_wp = self.calc_grab_wp(idx)
                self.routes[idx].insert_wp(align_wp)

            if idx == field.gk_id or self.routes[idx].get_dest_wp().type == wp.WType.R_IGNORE_GOAl_HULL:
                pth_wp = self.calc_passthrough_wp(field, idx)
                if pth_wp is not None:
                    self.routes[idx].insert_wp(pth_wp)
                continue

            for goal in [field.ally_goal, field.enemy_goal]:
                if aux.is_point_inside_poly(self_pos, goal.hull):
                    closest_out = aux.find_nearest_point(self_pos, goal.big_hull, [goal.up, aux.GRAVEYARD_POS, goal.down])
                    angle0 = self.routes[idx].get_dest_wp().angle
                    self.routes[idx].set_dest_wp(
                        wp.Waypoint(
                            goal.center + (closest_out - goal.center) * 1.2,
                            angle0,
                            wp.WType.S_ENDPOINT,
                        )
                    )
                    continue
                pint = aux.segment_poly_intersect(self_pos, self.routes[idx].get_next_wp().pos, goal.hull)
                if pint is not None:
                    angle0 = self.routes[idx].get_dest_wp().angle
                    if aux.is_point_inside_poly(self.routes[idx].get_dest_wp().pos, goal.big_hull):
                        self.routes[idx].set_dest_wp(wp.Waypoint(pint, angle0, wp.WType.S_ENDPOINT))
                        break
                    convex_hull = qh.shortesthull(
                        self_pos,
                        self_pos + self.routes[idx].get_next_vec(),
                        goal.big_hull,
                    )
                    for j in range(len(convex_hull) - 2, 0, -1):
                        self.routes[idx].insert_wp(wp.Waypoint(convex_hull[j], angle0, wp.WType.R_PASSTHROUGH))

            if self.__avoid_ball:
                dest_pos = self.routes[idx].get_dest_wp().pos
                self_pos = field.allies[idx].get_pos()
                angle0 = self.routes[idx].get_dest_wp().angle
                if aux.is_point_inside_circle(self_pos, field.ball.get_pos(), const.KEEP_BALL_DIST):
                    closest_out = aux.nearest_point_on_circle(self_pos, field.ball.get_pos(), const.KEEP_BALL_DIST)
                    self.routes[idx].insert_wp(wp.Waypoint(closest_out, angle0, wp.WType.R_PASSTHROUGH))
                    continue
                points = aux.line_circle_intersect(self_pos, dest_pos, field.ball.get_pos(), const.KEEP_BALL_DIST)
                if points is None:
                    continue
                if len(points) == 2:
                    points = aux.get_tangent_points(field.ball.get_pos(), self_pos, const.KEEP_BALL_DIST)
                    p = aux.Point(0, 0)
                    if points is None:
                        continue
                    if len(points) == 2:
                        p = points[0] if aux.dist(points[0], dest_pos) < aux.dist(points[1], dest_pos) else points[1]
                    else:
                        p = points[0]
                    self.routes[idx].insert_wp(
                        wp.Waypoint(
                            p + (p - field.ball.get_pos()).unity() * const.ROBOT_R,
                            angle0,
                            wp.WType.R_PASSTHROUGH,
                        )
                    )

            pth_wp = self.calc_passthrough_wp(field, idx)
            if pth_wp is not None:
                is_inside = False
                for goal in [field.ally_goal, field.enemy_goal]:
                    if aux.is_point_inside_poly(pth_wp.pos, goal.big_hull):
                        is_inside = True
                        break
                if not is_inside:
                    self.routes[idx].insert_wp(pth_wp)

    def calc_kick_wp(self, idx: int) -> wp.Waypoint:
        """
        Рассчитать точку для выравнивания по мячу
        """
        target_point = self.routes[idx].get_dest_wp()
        target_pos = target_point.pos
        target_angle = target_point.angle

        align_pos = target_pos - aux.rotate(aux.RIGHT, target_angle) * const.KICK_ALIGN_DIST
        align_angle = target_angle
        align_type = wp.WType.R_BALL_ALIGN
        # align_type = wp.WType.S_ENDPOINT
        align_wp = wp.Waypoint(align_pos, align_angle, align_type)
        return align_wp

    def calc_grab_wp(self, idx: int) -> wp.Waypoint:
        """
        Рассчитать точку для захвата мяча
        """
        target_point = self.routes[idx].get_dest_wp()
        target_pos = target_point.pos
        target_angle = target_point.angle

        align_pos = target_pos - aux.rotate(aux.RIGHT, target_angle) * const.GRAB_ALIGN_DIST
        align_angle = target_angle
        align_type = wp.WType.R_BALL_ALIGN
        # align_type = wp.WType.S_ENDPOINT
        align_wp = wp.Waypoint(align_pos, align_angle, align_type)
        return align_wp

    def calc_passthrough_wp(
        self, field: fld.Field, idx: int, obstacles: Optional[list[Entity]] = None
    ) -> Optional[wp.Waypoint]:
        """
        Рассчитать ближайшую промежуточную путевую точку
        согласно первому приближению векторного поля
        """
        robot = field.allies[idx]
        target = self.routes[idx].get_next_wp().pos

        if obstacles is None:
            obstacles_dist: list[tuple[Entity, float]] = [(field.ball, aux.dist(field.ball.get_pos(), robot.get_pos()))]

            for obstacle in field.enemies:  # in [field.enemies, field.allies]
                dist = (obstacle.get_pos() - robot.get_pos()).mag()
                if obstacle.is_used() and dist > obstacle.get_radius() + robot.get_radius():
                    obstacles_dist.append((obstacle.to_entity(), dist))

            sorted_obstacles = sorted(obstacles_dist, key=lambda x: x[1])

            obstacles = []
            for obst in sorted_obstacles:
                obstacles.append(obst[0])
        pth_point, length = self.calc_next_point(field, robot.get_pos(), target, obstacles)
        field.path_image.draw_line(robot.get_pos(), pth_point, color=(0, 0, 0))
        if pth_point == target:
            return None
        angle = self.routes[idx].get_next_angle()
        return wp.Waypoint(pth_point, angle, wp.WType.R_PASSTHROUGH)

    def calc_next_point(
        self,
        field: fld.Field,
        position: aux.Point,
        target: aux.Point,
        obstacles: list[Entity],
    ) -> tuple[aux.Point, float]:
        remaining_obstacles: list[Entity] = obstacles.copy()
        skipped_obstacles: list[Entity] = []
        while len(remaining_obstacles) > 0:
            obstacle = remaining_obstacles.pop(0)
            skipped_obstacles.append(obstacle)

            time_to_reach = aux.dist(obstacle.get_pos(), position) / const.MAX_SPEED
            center = obstacle.get_pos() + obstacle.get_vel() * time_to_reach
            radius = (
                obstacle.get_radius()
                + const.ROBOT_R
                + time_to_reach * obstacle.get_vel().mag() * 0.3  # <-- coefficient of fear [0; 1] (for moving obst)
            )
            # field.path_image.draw_dot(
            #     center,
            #     (127, 127, 127),
            #     radius,
            # )
            if (
                aux.line_circle_intersect(
                    position,
                    target,
                    center,
                    radius,
                )
                is not None
            ):
                tangents = aux.get_tangent_points(center, position, radius)
                if tangents is None or len(tangents) < 2:
                    continue

                tangents[0] = aux.point_on_line(
                    center,
                    tangents[0],
                    radius + const.ROBOT_R * 0.5,
                )
                tangents[1] = aux.point_on_line(
                    center,
                    tangents[1],
                    radius + const.ROBOT_R * 0.5,
                )

                point_before: list[aux.Point] = [aux.Point(0, 0) for _ in range(2)]
                length_before: list[float] = [0 for _ in range(2)]
                point_before[0], length_before[0] = self.calc_next_point(
                    field, position, tangents[0], skipped_obstacles[:-1]
                )
                point_before[1], length_before[1] = self.calc_next_point(
                    field, position, tangents[1], skipped_obstacles[:-1]
                )

                length_after: list[float] = [0 for _ in range(2)]
                _, length_after[0] = self.calc_next_point(field, tangents[0], target, remaining_obstacles)
                _, length_after[1] = self.calc_next_point(field, tangents[1], target, remaining_obstacles)

                length0 = length_before[0] + length_after[0]
                length1 = length_before[1] + length_after[1]

                in_zone0 = aux.is_point_inside_poly(point_before[0], field.ally_goal.big_hull) or aux.is_point_inside_poly(
                    point_before[0], field.enemy_goal.big_hull
                )
                in_zone1 = aux.is_point_inside_poly(point_before[1], field.ally_goal.big_hull) or aux.is_point_inside_poly(
                    point_before[1], field.enemy_goal.big_hull
                )

                if (length0 < length1 or in_zone1) and not in_zone0:
                    pth_point = point_before[0]
                    length = length_before[0] + length_after[0]
                else:
                    pth_point = point_before[1]
                    length = length_before[1] + length_after[1]

                field.path_image.draw_line(position, pth_point, color=(255, 0, 255))

                return (pth_point, length)

        field.path_image.draw_line(position, target, color=(255, 0, 127))
        return target, aux.dist(position, target)
