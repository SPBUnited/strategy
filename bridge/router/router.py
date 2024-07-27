"""
Модуль маршрутизации роботов
Получает от стратега требуемые координаты для каждого робота и
считает оптимальный маршрут для достижения этой точки
"""

import math
from typing import Optional

import bridge.router.waypoint as wp
from bridge import const
from bridge.auxiliary import aux, entity, fld
from bridge.auxiliary import quickhull as qh
from bridge.auxiliary import rbt
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
                    dest_pos + delta, field.ball.get_pos(), const.KEEP_BALL_DIST + const.ROBOT_R
                )
                angle0 = target.angle
                self.routes[idx].set_dest_wp(wp.Waypoint(closest_out, angle0, wp.WType.S_ENDPOINT))
                return

        if abs(target.pos.x) > const.GOAL_DX:
            target.pos.x = const.GOAL_DX * aux.sign(target.pos.x)
        if abs(target.pos.y) > 1500:
            target.pos.y = 1500 * aux.sign(target.pos.y)
        self.routes[idx].set_dest_wp(target)

    def reroute(self, field: fld.Field) -> None:
        """
        Рассчитать маршруты по актуальным путевым точкам
        """
        for idx in range(const.TEAM_ROBOTS_MAX_COUNT):

            self_pos = field.allies[idx].get_pos()

            if not self.routes[idx].is_used():
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
                # pth_wp = None
                if pth_wp is not None:
                    self.routes[idx].insert_wp(pth_wp)
                continue

            for goal in [field.ally_goal, field.enemy_goal]:
                if aux.is_point_inside_poly(self_pos, goal.big_hull):
                    closest_out = aux.find_nearest_point(self_pos, goal.big_hull, [goal.up, aux.GRAVEYARD_POS, goal.down])
                    angle0 = self.routes[idx].get_dest_wp().angle
                    self.routes[idx].set_dest_wp(
                        wp.Waypoint(goal.center + (closest_out - goal.center) * 1.2, angle0, wp.WType.S_ENDPOINT)
                    )
                    continue
                pint = aux.segment_poly_intersect(self_pos, self.routes[idx].get_next_wp().pos, goal.big_hull)
                if pint is not None:
                    angle0 = self.routes[idx].get_dest_wp().angle
                    if aux.is_point_inside_poly(self.routes[idx].get_dest_wp().pos, goal.big_hull):
                        self.routes[idx].set_dest_wp(wp.Waypoint(pint, angle0, wp.WType.S_ENDPOINT))
                        break
                    convex_hull = qh.shortesthull(self_pos, self_pos + self.routes[idx].get_next_vec(), goal.big_hull)
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
                        wp.Waypoint(p + (p - field.ball.get_pos()).unity() * const.ROBOT_R, angle0, wp.WType.R_PASSTHROUGH)
                    )

            pth_wp = self.calc_passthrough_wp(field, idx)
            # print(pth_wp)
            if pth_wp is not None:
                is_inside = False
                for goal in [field.ally_goal, field.enemy_goal]:
                    if aux.is_point_inside_poly(pth_wp.pos, goal.big_hull):
                        is_inside = True
                        break
                if not is_inside:
                    self.routes[idx].insert_wp(pth_wp)

    def calc_passthrough_wp(self, field: fld.Field, idx: int) -> Optional[wp.Waypoint]:
        """
        просчитать путь до конечной точки, учитывая объекты на поле
        """
        p_to_go = self.routes[idx].get_next_wp().pos
        # print(p_to_go.x, p_to_go.y)
        start_p = PointTree(field.allies[idx].get_pos().x, field.allies[idx].get_pos().y)
        end_p = PointTree(p_to_go.x, p_to_go.y)
        all_robots: list[entity.Entity | rbt.Robot] = []
        queue = [start_p]
        n_max = float("inf")
        n_steps = 0
        finish = None
        point_go_now = None
        for ally in field.allies:
            if ally.is_used() and ally.r_id != idx:
                all_robots.append(ally)
        for enemy in field.enemies:
            if enemy.is_used():
                all_robots.append(enemy)
        all_robots.append(field.ball)
        while n_steps < n_max and n_steps < len(queue):
            on_way: list[int] = []
            for i, robot in enumerate(all_robots):
                if (
                    aux.dist2line(queue[n_steps].point(), end_p.point(), robot.get_pos())
                    < robot.get_radius() + all_robots[0].get_radius()
                    and aux.is_on_line(queue[n_steps].point(), end_p.point(), all_robots[i].get_pos())
                    and queue[n_steps].my_robot != i
                ):
                    on_way.append(i)
            if len(on_way) == 0:
                flag = 1
                if end_p.father:
                    old_way_l = 0.0
                    point_mas = end_p
                    # print("start")
                    while point_mas.father is not None:
                        # print(point_mas.x, point_mas.y)
                        old_way_l += aux.dist(point_mas.point(), point_mas.father.point())
                        point_mas = point_mas.father
                    new_way_l = aux.dist(end_p.point(), queue[n_steps].point())
                    point_mas = queue[n_steps]
                    while point_mas.father is not None:
                        new_way_l += aux.dist(point_mas.point(), point_mas.father.point())
                        point_mas = point_mas.father
                    if old_way_l < new_way_l:
                        flag = 0
                if flag:
                    end_p.father = queue[n_steps]
                    # print("start")
                    # print(end_p.x, end_p.y)
                    point_mas = end_p
                    finish = True
                    if point_mas.father is not None:
                        while point_mas.father.father is not None:
                            # print(point_mas.x, point_mas.y)
                            finish = False
                            point_mas = point_mas.father
                        # print(point_mas.x, point_mas.y)
                    point_go_now = point_mas
                    if n_max == float("inf"):
                        n_max = n_steps + 5
            else:
                min_num_on_way = -1
                min_dist_on_way = float("inf")
                for i, on_w in enumerate(on_way):
                    if aux.dist(all_robots[on_w].get_pos(), queue[n_steps].point()) < min_dist_on_way:
                        min_dist_on_way = aux.dist(all_robots[on_w].get_pos(), queue[n_steps].point())
                        min_num_on_way = i
                all_new = all_robots.copy()
                mas2 = [all_new[on_way[min_num_on_way]]]
                i2 = 0
                while i2 < len(mas2):
                    another = 0
                    while another < len(all_new):
                        if (
                            aux.dist(mas2[i2].get_pos(), all_new[another].get_pos())
                            < mas2[i2].get_radius() + all_new[another].get_radius() + all_robots[0].get_radius() * 2
                            and another not in mas2
                        ):
                            mas2.append(all_new[another])
                            all_new.pop(another)
                            another -= 1
                        another += 1
                    i2 += 1
                all_angles: list[float] = []
                all_dists: list[float] = []
                flag_none = False
                gamma = aux.angle_to_point(queue[n_steps].point(), end_p.point())
                for wall in mas2:
                    tangents = aux.get_tangent_points(
                        wall.get_pos(), queue[n_steps].point(), wall.get_radius() + all_robots[0].get_radius()
                    )
                    if not tangents:
                        flag_none = True
                    else:
                        for tang in tangents:
                            tang = aux.point_on_line(
                                wall.get_pos(), tang, (wall.get_radius() + all_robots[0].get_radius()) * 1.1
                            )
                            all_angles.append(aux.get_angle_between_points(end_p.point(), queue[n_steps].point(), tang))
                            all_dists.append(aux.dist(tang, queue[n_steps].point()))
                # print(angs[0], angs[1], gamma)
                if flag_none:
                    point_go_now = end_p
                    break
                else:
                    idx_max = aux.get_minmax_idx(all_angles, "max")
                    idx_min = aux.get_minmax_idx(all_angles, "min")
                    if idx_max is not None and idx_min is not None:
                        queue.append(
                            PointTree(
                                queue[n_steps].x + math.cos(all_angles[idx_min] + gamma) * all_dists[idx_min],
                                queue[n_steps].y + math.sin(all_angles[idx_min] + gamma) * all_dists[idx_min],
                                queue[n_steps],
                                on_way[min_num_on_way],
                            )
                        )
                        queue.append(
                            PointTree(
                                queue[n_steps].x + math.cos(all_angles[idx_max] + gamma) * all_dists[idx_max],
                                queue[n_steps].y + math.sin(all_angles[idx_max] + gamma) * all_dists[idx_max],
                                queue[n_steps],
                                on_way[min_num_on_way],
                            )
                        )
                # print("start")
                # for i in queue:
                #    print(i.x, i.y)
            n_steps += 1
        # if idx == 0:
        #     print(point_go_now.x, point_go_now.y)
        if finish or point_go_now is None:
            return None
        return wp.Waypoint(point_go_now.point(), self.routes[idx].get_next_wp().angle, wp.WType.R_PASSTHROUGH)

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

    def get_route(self, idx: int) -> route.Route:
        """
        Получить маршрут для робота с индексом idx
        """
        return self.routes[idx]


class PointTree:
    """
    класс точки в дереве.
    """

    def __init__(self, x: float, y: float, father: Optional["PointTree"] = None, my_robot: Optional[int] = None) -> None:
        self.x = x
        self.y = y
        self.father = father
        self.my_robot = my_robot

    def point(self) -> aux.Point:
        """
        возвращает значение в формате дефолтной точки.
        """
        return aux.Point(self.x, self.y)
