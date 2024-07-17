"""Верхнеуровневый код стратегии"""

# pylint: disable=redefined-outer-name

# @package Strategy
# Расчет требуемых положений роботов исходя из ситуации на поле

import math

# !v DEBUG ONLY
from enum import Enum
from time import time
from typing import Optional

from bridge.auxiliary import aux, fld, rbt

from bridge import const, drawing
import bridge.strategy.ref_states as refs
from bridge.strategy import kicker
import bridge.router.waypoint as wp

from bridge.processors.referee_state_processor import State as GameStates
from bridge.processors.referee_state_processor import Color as ActiveTeam


class States(Enum):
    """Класс с глобальными состояниями игры"""

    DEBUG = 0
    DEFENSE = 1
    ATTACK = 2


class Role(Enum):
    """Класс с ролями"""

    GOALKEEPER = 0
    ATTACKER = 1

    WALLLINER = 3
    FORWARD = 11

    UNAVAILABLE = 100


class FloatingBorder:
    """Floating border for dangerous borders in conditions"""

    def __init__(self, middle: float, moving: float = 0.1) -> None:
        self.low = middle * (1 - moving)
        self.high = middle * (1 + moving)

        self.middle = middle
        self.border = middle

    def reset(self) -> None:
        """Reset"""
        self.__border = self.middle

    def is_higher(self, val: float) -> bool:
        """Return True if higher, return False if lower"""
        if val > self.border:
            self.border = self.low
            return True
        else:
            self.border = self.high
            return False

    def is_lower(self, val: float) -> bool:
        """Return False if higher, return True if lower"""
        return not self.is_higher(val)


class Strategy:
    """Основной класс с кодом стратегии"""

    def __init__(
        self,
        dbg_game_status: GameStates = GameStates.RUN,
        dbg_state: States = States.ATTACK,
    ) -> None:
        self.refs = refs.RefStates()

        self.game_status = dbg_game_status
        self.active_team: ActiveTeam = ActiveTeam.ALL
        self.we_kick = False
        self.we_active = False
        self.state = dbg_state
        self.timer = time()

        self.forwards: list[rbt.Robot] = []
        self.prev_roles: list[Role] = [
            Role.UNAVAILABLE for _ in range(const.TEAM_ROBOTS_MAX_COUNT)
        ]
        self.ball_start_point: Optional[aux.Point] = None

        self.kick = kicker.KickerAux()

        self.ball_history: list[Optional[aux.Point]] = [None] * round(0.3 / const.Ts)
        self.ball_pass_update_timer = 0
        self.ball_history_idx = 0
        self.old_ball_pos = None

        self.pass_or_kick_decision_border = FloatingBorder(0.2, 0.3)

    def change_game_state(
        self, new_state: GameStates, upd_active_team: ActiveTeam
    ) -> None:
        """Изменение состояния игры и цвета команды"""
        self.game_status = new_state
        self.active_team = upd_active_team

    def choose_roles(self, field: fld.Field) -> list[Role]:
        """
        Определение ролей для роботов на поле
        """

        ATTACK_ROLES = [
            Role.FORWARD,
            Role.FORWARD,
            Role.FORWARD,
            Role.FORWARD,
            Role.FORWARD,
            Role.FORWARD,
        ]

        DEFENSE_ROLES = [
            Role.WALLLINER,
            Role.WALLLINER,
            Role.WALLLINER,
            Role.WALLLINER,
            Role.WALLLINER,
            Role.WALLLINER,
        ]

        atk_min = 1
        def_min = 2
        # atk_min = 0
        # def_min = 0

        free_allies = -atk_min - def_min - 1

        for ally in field.allies:
            if ally.is_used() and ally.r_id != field.gk_id:
                free_allies += 1

        # free_allies = max(0, free_allies)

        ball_pos = aux.minmax(field.ball.get_pos().x, const.GOAL_DX)
        atks = (
            round(
                free_allies
                / (2 * const.GOAL_DX)
                * (-ball_pos * field.polarity + const.GOAL_DX)
            )
            + atk_min
        )
        defs = free_allies - (atks - atk_min) + def_min

        roles = ATTACK_ROLES[:atks] + DEFENSE_ROLES[:defs]
        return [Role.GOALKEEPER, Role.ATTACKER] + sorted(roles, key=lambda x: x.value)

    def choose_robots_for_roles(
        self, field: fld.Field, roles: list[Role], wall_pos: aux.Point
    ) -> list[Role]:
        robot_roles: list[Role] = [
            Role.UNAVAILABLE for _ in range(const.TEAM_ROBOTS_MAX_COUNT)
        ]
        used_ids: list[int] = []

        for robot_id, role in enumerate(self.prev_roles):
            if role in [Role.FORWARD, Role.WALLLINER] and field.is_ball_moves_to_point(
                field.allies[robot_id].get_pos()
            ):
                used_ids.append(robot_id)
                robot_roles[robot_id] = role
                delete_role(roles, role)

        # used_ids.append(13)
        # robot_roles[13] = Role.WALLLINER
        # delete_role(roles, Role.WALLLINER)

        for role in roles:
            robot_id = -1
            if role == Role.GOALKEEPER:
                robot_id = field.gk_id
            elif role == Role.ATTACKER:
                if (
                    not field.robot_with_ball in field.allies
                    or field.robot_with_ball == field.allies[field.gk_id]
                ):
                    robot_id = fld.find_nearest_robot(
                        field.ball.get_pos(), field.allies, used_ids
                    ).r_id
                else:
                    robot_id = field.robot_with_ball.r_id
            elif role == Role.WALLLINER:
                robot_id = fld.find_nearest_robot(wall_pos, field.allies, used_ids).r_id
            elif role == Role.FORWARD:
                robot_id = fld.find_nearest_robot(
                    field.enemy_goal.center, field.allies, used_ids
                ).r_id
            robot_roles[robot_id] = role
            used_ids.append(robot_id)

        self.prev_roles = robot_roles
        return robot_roles

    def process(self, field: fld.Field) -> list[wp.Waypoint]:
        """
        Рассчитать конечные точки для каждого робота
        """
        if self.active_team == ActiveTeam.ALL or field.ally_color == self.active_team:
            self.we_active = True
        else:
            self.we_active = False

        if self.ball_history[self.ball_history_idx] is None:
            self.ball_start_point = self.ball_history[0]
        else:
            self.ball_start_point = self.ball_history[self.ball_history_idx]

        self.ball_history[self.ball_history_idx] = field.ball.get_pos()
        self.ball_history_idx += 1
        self.ball_history_idx %= len(self.ball_history)

        waypoints: list[wp.Waypoint] = []
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoints.append(
                wp.Waypoint(
                    field.allies[i].get_pos(),
                    field.allies[i].get_angle(),
                    wp.WType.S_STOP,
                )
            )

        if self.game_status != GameStates.PENALTY:
            self.refs.is_started = 0

        print("-" * 32)
        print(self.game_status, "\twe_active:", self.we_active)

        if self.game_status == GameStates.RUN:
            self.run(field, waypoints)
        else:
            if self.game_status == GameStates.TIMEOUT:
                self.refs.timeout(field, waypoints)
            elif self.game_status == GameStates.HALT:
                pass
                # self.halt(field, waypoints)
            elif self.game_status == GameStates.PREPARE_PENALTY:
                self.refs.prepare_penalty(field, waypoints)
            elif self.game_status == GameStates.PENALTY:
                if self.refs.we_kick or 1:
                    self.refs.penalty_kick(field, waypoints)
                else:
                    robot_with_ball = fld.find_nearest_robot(
                        field.ball.get_pos(), field.enemies
                    )
                    waypoints[field.gk_id] = self.goalk(field, 0, robot_with_ball)[0]
            elif self.game_status == GameStates.PREPARE_KICKOFF:
                self.prepare_kickoff(field, waypoints)
            elif self.game_status == GameStates.KICKOFF:
                self.kickoff(field, waypoints)
            elif self.game_status == GameStates.FREE_KICK:
                self.run(field, waypoints)
            elif self.game_status == GameStates.STOP:
                self.run(field, waypoints)

        # self.debug(field, waypoints)  # NOTE

        return waypoints

    def run(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """
        Определение глобального состояния игры
        roles - роли роботов, отсортированные по приоритету
        robot_roles - список соответствия id робота и его роли
        """
        # waypoints[12] = kicker.spin_around_ball(1)
        # return

        "Определение набора ролей для роботов"
        roles = self.choose_roles(field)

        "Вычисление конечных точек, которые не зависят от выбранного робота"
        if aux.is_point_inside_poly(field.ball.get_pos(), field.ally_goal.hull):
            wall_enemy = fld.find_nearest_robot(
                field.ally_goal.center, field.enemies
            ).get_pos()
        else:
            wall_enemy = field.ball.get_pos()
        wall_pos = self.calc_wall_pos(field, wall_enemy)

        "Выбор роботов для всех ролей, с учетом вычисленных выше точек"
        robot_roles = self.choose_robots_for_roles(field, roles, wall_pos)

        print("Roles", field.ally_color)
        for idx, role in enumerate(robot_roles):
            if role != Role.UNAVAILABLE:
                print("  ", idx, role)

        "Вычисление конечных точек, которые зависят от положения робота и создание путевых точек"
        self.forwards = find_role(field, robot_roles, Role.FORWARD)
        if self.forwards is not None:
            self.set_forwards_wps(field, waypoints)

        if Role.ATTACKER in robot_roles:
            attacker_id = find_role(field, robot_roles, Role.ATTACKER)[0].r_id
            self.attacker(field, waypoints, attacker_id)

        wallliners = find_role(field, robot_roles, Role.WALLLINER)
        for waller in wallliners:
            # print("waller:", waller.r_id)
            pass
        if Role.WALLLINER in robot_roles:
            self.set_wallliners_wps(field, waypoints, wallliners, wall_enemy)

        if Role.GOALKEEPER in robot_roles:
            robot_with_ball = fld.find_nearest_robot(field.ball.get_pos(), field.allies)
            waypoints[field.gk_id] = self.goalk(field, wallliners, robot_with_ball)

    def debug(
        self, field: fld.Field, waypoints: list[wp.Waypoint]
    ) -> list[wp.Waypoint]:
        """Отладка"""

        angle = aux.angle_to_point(field.allies[0].get_pos(), field.ball.get_pos())
        waypoints[0] = wp.Waypoint(field.ball.get_pos(), angle, wp.WType.S_ENDPOINT)
        # self.goalk(field, waypoints, [field.gk_id], robot_with_ball)

        # waypoints[const.DEBUG_ID].pos = field.ball.get_pos()
        # waypoints[const.DEBUG_ID].angle = (field.ally_goal.center - field.ball.get_pos() + aux.UP * self.square.get()).arg()
        # waypoints[const.DEBUG_ID].type = wp.WType.S_BALL_KICK

        return waypoints

    def attacker(
        self, field: fld.Field, waypoints: list[wp.Waypoint], attacker_id: int
    ) -> None:
        """Логика действий для робота с мячом"""
        kick_point = self.choose_kick_point(field, attacker_id)
        kick_est = self.estimate_pass_point(field, field.ball.get_pos(), kick_point)

        if (
            self.pass_or_kick_decision_border.is_lower(kick_est)
            and len(self.forwards) > 0
        ):
            receiver_id, pass_est = self.choose_receiver(field)
            # print(pass_est, kick_est)
            if pass_est is not None and pass_est > kick_est and receiver_id is not None:
                waypoints[attacker_id] = self.pass_kicker(
                    field, attacker_id, receiver_id
                )
                print("attacker: pass to", receiver_id)
                return

        waypoints[attacker_id] = self.kick.shoot_to_goal(
            field, field.allies[attacker_id], kick_point
        )
        print("attacker: shoot to goal")

    def calc_wall_pos(self, field: fld.Field, ball: aux.Point) -> aux.Point:
        """Рассчитывает точку, по которой будут выбираться роботы для стенки"""
        for i in range(
            len(field.ally_goal.hull) * 1
        ):  # NOTE: исправить пересечение прямой с углом зоны
            point = aux.get_line_intersection(
                field.ally_goal.hull[i - 1],
                field.ally_goal.hull[i],
                field.ally_goal.center,
                ball,
                "SR",
            )
            if point is not None:
                return point
        return field.ally_goal.frw

    def set_wall_targets(
        self, field: fld.Field, num: int, ball: aux.Point
    ) -> list[aux.Point]:
        """Рассчитывает точки для стенки из num роботов"""
        if aux.is_point_inside_poly(ball, field.ally_goal.big_hull):
            ball = fld.find_nearest_robot(
                field.ally_goal.center, field.enemies
            ).get_pos()
        poses = []

        intersections: list[Optional[aux.Point]] = []

        intersections.append(
            aux.segment_poly_intersect(
                ball, field.ally_goal.down, field.ally_goal.big_hull
            )
        )
        intersections.append(
            aux.segment_poly_intersect(
                ball, field.ally_goal.up, field.ally_goal.big_hull
            )
        )

        if intersections[0] is None or intersections[1] is None:
            return [
                field.ally_goal.frw
                + field.ally_goal.eye_up * const.ROBOT_R * (-(num - 1) + 2 * i)
                for i in range(num)
            ]  # ball in goal.hull
        elif aux.dist(ball, intersections[0]) > aux.dist(ball, intersections[1]):
            intersections[1], intersections[0] = intersections[0], intersections[1]

        length = min((ball - intersections[0]).mag(), (ball - intersections[1]).mag())
        intersections[0] = ball + (intersections[0] - ball).unity() * length
        intersections[1] = ball + (intersections[1] - ball).unity() * length
        for point in field.ally_goal.big_hull:
            inter = aux.get_line_intersection(
                ball, point, intersections[0], intersections[1], "RS"
            )
            if inter is None:
                continue
            if aux.is_point_inside_poly(
                point, [ball, intersections[0], intersections[1]]
            ):
                scale = aux.dist(ball, point) / aux.dist(ball, inter)
                intersections[0] = aux.lerp(ball, intersections[0], scale)
                intersections[1] = aux.lerp(ball, intersections[1], scale)

        wall_vec = intersections[1] - intersections[0]

        if wall_vec.mag() > num * 2 * const.ROBOT_R:
            for i in range(num):
                delta = const.ROBOT_R * (2 * i + 1)
                poses.append(intersections[0] + wall_vec.unity() * delta)
                # field.image.draw_dot(
                #     intersections[0] + wall_vec.unity() * delta,
                #     const.ROBOT_R * field.image.scale,
                #     (128, 128, 128),
                # )
        else:
            wall_middle = intersections[0] + wall_vec / 2
            for i in range(num):
                delta = const.ROBOT_R * (-(num - 1) + 2 * i)
                poses.append(wall_middle + wall_vec.unity() * delta)
                # field.image.draw_dot(
                #     wall_middle + wall_vec.unity() * delta,
                #     const.ROBOT_R * field.image.scale,
                #     (200, 200, 200),
                # )

        return poses

    def set_wallliners_wps(
        self,
        field: fld.Field,
        waypoints: list[wp.Waypoint],
        wallliners: list[rbt.Robot],
        enemy_pos: aux.Point,
    ) -> None:
        """Создает путевые точки для роботов в стенке"""
        poses = self.set_wall_targets(field, len(wallliners), enemy_pos)

        projections: list[tuple[float, int]] = []
        for _, waller in enumerate(wallliners):
            point = aux.closest_point_on_line(
                poses[0], poses[len(poses) - 1], waller.get_pos(), "L"
            )
            projections.append((point.y, waller.r_id))
        projections = sorted(projections, key=lambda x: x[0])
        poses = sorted(poses, key=lambda x: x.y)

        wp_type = wp.WType.R_IGNORE_GOAl_HULL
        if aux.is_point_inside_poly(field.ball.get_pos(), field.ally_goal.hull):
            wp_type = wp.WType.S_ENDPOINT

        for i, pos in enumerate(poses):
            idx = projections[i][1]
            angle = (field.ball.get_pos() - field.ally_goal.center).arg()
            waypoints[idx] = wp.Waypoint(pos, angle, wp_type)

    def choose_receiver(
        self, field: fld.Field
    ) -> tuple[Optional[int], Optional[float]]:
        """Выбирает робота для получения паса"""
        receiver_id = None
        receiver_score = None
        for forward in self.forwards:
            pass_score = self.estimate_pass_point(
                field, field.ball.get_pos(), forward.get_pos()
            )
            kick_point = self.choose_kick_point(
                field, forward.r_id, ball_pos=forward.get_pos()
            )
            kick_score = self.estimate_pass_point(field, forward.get_pos(), kick_point)

            score = pass_score * kick_score
            if receiver_id is None or score > receiver_score:
                receiver_id = forward.r_id
                receiver_score = score
        return receiver_id, score

    def set_forwards_wps(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Расставляет роботов по точкам для получения паса"""
        pos_num = len(self.forwards)

        k = -1 if const.SELF_PLAY else 1
        poses = [
            aux.Point(-3500 * field.polarity * k, 1750),
            aux.Point(-3500 * field.polarity * k, -1750),
            aux.Point(-3000 * field.polarity * k, 0),
        ]
        poses = poses[: (pos_num + 1)]
        bad_pos = aux.find_nearest_point(field.ball.get_pos(), poses)

        used_forwards: list[int] = []

        for pos in poses:
            if pos == bad_pos:
                continue
            if len(used_forwards) == pos_num:
                return
            pop = fld.find_nearest_robot(pos, self.forwards, used_forwards)
            used_forwards.append(pop.r_id)
            self.pass_receiver(field, waypoints, pop.r_id, pos)

    def pass_kicker(
        self, field: fld.Field, kicker_id: int, receiver_id: int
    ) -> wp.Waypoint:
        """
        Отдает пас от робота kicker_id роботу receiver_id
        Должна вызываться в конечном автомате постоянно, пока первый робот не даст пас
        """
        receiver = field.allies[receiver_id]
        if not field.is_ball_moves_to_point(receiver.get_pos()):
            # waypoints[kicker_id] = wp.Waypoint(
            #     field.ball.get_pos(),
            #     aux.angle_to_point(field.ball.get_pos(), receiver.get_pos()),
            #     wp.WType.S_BALL_PASS,
            # )
            # print("pass to", receiver_id)
            waypoint = self.kick.pass_to_point(
                field, field.allies[kicker_id], receiver.get_pos()
            )
            # field.image.draw_dot(
            #     field.ball.get_pos()
            #     + aux.rotate(
            #         aux.RIGHT,
            #         aux.angle_to_point(field.ball.get_pos(), receiver.get_pos()),
            #     ),
            #     5,
            #     (255, 0, 255),
            # )
        else:
            waypoint = wp.Waypoint(aux.Point(0, 0), 0, wp.WType.S_STOP)

        return waypoint

    def pass_receiver(
        self,
        field: fld.Field,
        waypoints: list[wp.Waypoint],
        receiver_id: int,
        receive_point: aux.Point,
    ) -> None:
        """
        Отдает пас от робота kicker_id роботу receiver_id
        Должна вызываться в конечном автомате постоянно, пока второй робот не поймает мяч
        TODO: прописать действия отдающего пас робота после удара и принимающего пас робота до удара
        """
        receiver = field.allies[receiver_id]
        if (
            field.is_ball_moves_to_point(receiver.get_pos())
            and self.ball_start_point is not None
            and (self.ball_start_point - field.ball.get_pos()).mag()
            > const.INTERCEPT_SPEED
        ):
            target = aux.closest_point_on_line(
                self.ball_start_point, field.ball.get_pos(), receiver.get_pos(), "R"
            )
            # field.image.draw_dot(target, 5, (255, 255, 0))

            waypoints[receiver_id] = wp.Waypoint(
                target,
                aux.angle_to_point(field.ball.get_pos(), self.ball_start_point),
                wp.WType.S_ENDPOINT,
            )
        else:
            waypoints[receiver_id] = wp.Waypoint(
                receive_point,
                aux.angle_to_point(receive_point, field.ball.get_pos()),
                wp.WType.S_ENDPOINT,
            )
            # field.image.draw_dot(receive_point, 5, (255, 255, 0))

    def estimate_pass_point(
        self, field: fld.Field, frm: Optional[aux.Point], to: Optional[aux.Point]
    ) -> float:
        """
        Оценивает пас из точки "frm" в точку "to, возвращая положительное значение до 0.8
        """
        if frm is None or to is None:
            return 0
        positions: list[tuple[int, aux.Point]] = []

        poses = field.enemies if not const.SELF_PLAY else field.allies
        for robot in poses:
            if robot.is_used() and robot.get_pos() != to:
                positions.append((robot.r_id, robot.get_pos()))
        positions = sorted(positions, key=lambda x: x[1].y)

        tangents: list[tuple[int, list[aux.Point]]] = []
        for p in positions:
            tgs = aux.get_tangent_points(p[1], frm, const.ROBOT_R)
            if tgs is None or len(tgs) < 2:
                continue
            tangents.append((p[0], tgs))

        min_ = 10e3

        shadows_bots = []
        for tangent in tangents:
            ang1 = aux.get_angle_between_points(to, frm, tangent[1][0])
            ang2 = aux.get_angle_between_points(to, frm, tangent[1][1])

            if ang1 * ang2 < 0:
                shadows_bots.append(tangent[0])
            ang1 = abs(ang1)
            ang2 = abs(ang2)
            # if ang1 > 180:
            #     ang1 = 360 - ang1
            # if ang2 > 180:
            #     ang2 = 360 - ang2

            if ang1 < min_:
                min_ = ang1
            if ang2 < min_:
                min_ = ang2
        # if minId == -1:
        #     return 0

        if min_ == 10e3 or len(shadows_bots) != 0:
            return 0
        dist = (frm - to).mag() / 1000
        max_ang = abs(
            aux.wind_down_angle(2 * math.atan2(const.ROBOT_SPEED, -0.25 * dist + 4.5))
        )
        # max_ang = 10
        return min(abs(min_ / max_ang), 1)

    def choose_kick_point(
        self,
        field: fld.Field,
        kicker_id: int,
        goal: Optional[fld.Goal] = None,
        ball_pos: Optional[aux.Point] = None,
        interfering_robots: Optional[list[rbt.Robot]] = None,
    ) -> aux.Point:
        """
        Выбирает оптимальную точку в воротах для удара
        """
        if goal is None:
            goal = field.enemy_goal
        if ball_pos is None:
            ball_pos = field.ball.get_pos()
        if interfering_robots is None:
            interfering_robots = field.enemies
            if const.SELF_PLAY:
                interfering_robots = field.allies

        A, C = self.choose_segment_in_goal(
            field, kicker_id, goal, ball_pos, interfering_robots
        )
        B = ball_pos

        tmp1 = (C - B).mag()
        tmp2 = (A - B).mag()
        CA = A - C
        pnt = A
        if tmp2 != 0:
            pnt = C + CA * 0.5 * (tmp1 / tmp2)
        # field.image.draw_dot(pnt, 10, (255, 0, 0))

        return pnt

    def choose_segment_in_goal(
        self,
        field: fld.Field,
        kicker_id: int,
        goal: fld.Goal,
        ball_pos: aux.Point,
        interfering_robots: list[rbt.Robot],
    ) -> tuple[aux.Point, aux.Point]:
        """
        Выбирает самый большой угловой промежуток на воротах (если смотреть из точки ball_pos)
        """
        positions = []
        for rbt in interfering_robots:
            if rbt != field.allies[kicker_id]:
                if (
                    aux.dist(rbt.get_pos(), goal.center)
                    < aux.dist(goal.center, ball_pos)
                    and rbt.is_used()
                ):
                    positions.append(rbt.get_pos())

        positions = sorted(positions, key=lambda x: x.y * -goal.eye_up.y)

        segments = [goal.up]
        for p in positions:
            tangents = aux.get_tangent_points(p, ball_pos, const.ROBOT_R)
            if tangents is None or len(tangents) < 2:
                continue

            int1 = aux.get_line_intersection(
                ball_pos,
                tangents[0],
                goal.down,
                goal.up,
                "RS",
            )
            int2 = aux.get_line_intersection(
                ball_pos,
                tangents[1],
                goal.down,
                goal.up,
                "RS",
            )

            if int1 is None and int2 is not None:
                segments.append(goal.up)
                segments.append(int2)
            elif int1 is not None and int2 is None:
                segments.append(int1)
                segments.append(goal.down)
            elif int1 is not None and int2 is not None:
                segments.append(int1)
                segments.append(int2)

        segments.append(goal.down)
        max_ = 0.0
        maxId = -1
        for i in range(0, len(segments), 2):
            c = segments[i]
            a = segments[i + 1]
            b = ball_pos
            if (c.y - a.y) * goal.eye_up.y < 0:
                continue  # Shadow intersection
            ang = abs(
                aux.get_angle_between_points(a, b, c)
            )  # Саша, я тут градусы на радианы заменил, надеюсь ничего не сломалось
            if ang > max_:
                max_ = ang
                maxId = i

        if maxId == -1:
            return (goal.down, goal.up)

        return (segments[maxId], segments[maxId + 1])

    def goalk(
        self,
        field: fld.Field,
        wallliners: list[rbt.Robot],
        robot_with_ball: Optional[rbt.Robot],
    ) -> wp.Waypoint:
        """
        Управление вратарём и стенкой
        Возвращает массив с элементами класса Waypoint, где 0й элемент - вратарь, а остальные - стенка
        """
        gk_pos: aux.Point | None = None
        gk_angle: float
        if field.gk_id < 9:
            gk_angle = field.ally_goal.eye_forw.arg()
        else:
            gk_angle = math.pi / 2
        ball = field.ball.get_pos()
        goal_down = field.ally_goal.down
        goal_up = field.ally_goal.up

        if abs(ball.x) > const.GOAL_DX:  # NOTE
            return wp.Waypoint(
                field.ally_goal.center, gk_angle, wp.WType.S_IGNOREOBSTACLES
            )

        if (
            field.is_ball_stop_near_goal()
            or field.robot_with_ball == field.allies[field.gk_id]
        ):
            return wp.Waypoint(
                ball, field.ally_goal.eye_forw.arg(), wp.WType.S_BALL_KICK_UP
            )

        if (
            field.is_ball_moves_to_goal()
            and self.ball_start_point is not None
            and (self.ball_start_point - ball).mag() > const.INTERCEPT_SPEED
        ):
            tmp_pos = aux.get_line_intersection(
                self.ball_start_point,
                ball,
                field.ally_goal.down,
                field.ally_goal.up,
                "RS",
            )
            if tmp_pos is not None:
                gk_pos = aux.closest_point_on_line(
                    ball, tmp_pos, field.allies[field.gk_id].get_pos()
                )
                return wp.Waypoint(gk_pos, gk_angle, wp.WType.S_IGNOREOBSTACLES)
            else:
                tmp_pos = aux.get_line_intersection(
                    self.ball_start_point,
                    ball,
                    field.ally_goal.center_down,
                    field.ally_goal.center_up,
                    "RS",
                )
                if tmp_pos is not None:
                    poses = [
                        goal_up
                        + (field.ally_goal.eye_forw - field.ally_goal.eye_up)
                        * const.ROBOT_R
                        * 0.8,
                        goal_down
                        + (field.ally_goal.eye_forw + field.ally_goal.eye_up)
                        * const.ROBOT_R
                        * 0.8,
                    ]
                    gk_pos = aux.find_nearest_point(tmp_pos, poses)
                    return wp.Waypoint(gk_pos, gk_angle, wp.WType.S_IGNOREOBSTACLES)

        if len(wallliners) != 0:
            segment = self.choose_segment_in_goal(
                field, -1, field.ally_goal, ball, wallliners
            )
            goal_down, goal_up = segment

        if robot_with_ball is not None:
            predict = aux.get_line_intersection(
                robot_with_ball.get_pos(),
                robot_with_ball.get_pos()
                + aux.rotate(aux.RIGHT, robot_with_ball.get_angle()),
                goal_down,
                goal_up,
                "RS",
            )
            if predict is not None:
                p_ball = (ball - predict).unity()
                gk_pos = aux.lerp(
                    aux.point_on_line(field.ally_goal.center, ball, const.GK_FORW),
                    p_ball * const.GK_FORW
                    + aux.get_line_intersection(
                        robot_with_ball.get_pos(),
                        robot_with_ball.get_pos()
                        + aux.rotate(aux.RIGHT, robot_with_ball.get_angle()),
                        goal_down,
                        goal_up,
                        "RS",
                    ),
                    0.5,
                )
                if gk_pos is not None:
                    return wp.Waypoint(gk_pos, gk_angle, wp.WType.S_IGNOREOBSTACLES)

        tmp1 = (goal_up - ball).mag()
        tmp2 = (goal_down - ball).mag()
        seg = goal_down - goal_up
        if tmp2 != 0:
            kick_point = goal_up + seg * 0.5 * (tmp1 / tmp2)
        else:
            kick_point = goal_down

        radius = (const.GK_FORW**2 + (const.GOAL_DY / 2) ** 2) / 2 / const.GK_FORW
        circle_center = field.ally_goal.center - field.ally_goal.eye_forw * (
            radius - const.GK_FORW - const.ROBOT_R
        )
        intersects = aux.line_circle_intersect(kick_point, ball, circle_center, radius)

        if intersects is None:
            gk_pos = field.ally_goal.center + field.ally_goal.eye_forw * const.ROBOT_R
        else:
            gk_pos = intersects[0]

        gk_pos.x = aux.minmax(gk_pos.x, const.GOAL_DX - const.ROBOT_R)

        return wp.Waypoint(gk_pos, gk_angle, wp.WType.S_IGNOREOBSTACLES)

    def prepare_kickoff(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Настройка перед состоянием kickoff по команде судей"""
        if self.we_active:
            self.we_kick = True
        else:
            self.we_kick = False
        self.put_kickoff_waypoints(field, waypoints)

    def put_kickoff_waypoints(
        self, field: fld.Field, waypoints: list[wp.Waypoint]
    ) -> None:
        """Подготовка перед состоянием kickoff"""
        if self.we_kick:
            if const.DIV == "B":
                poses = [
                    aux.Point(650 * field.polarity, 0),
                    aux.Point(2000 * field.polarity, 0),
                    aux.Point(3500 * field.polarity, 0),
                    aux.Point(300 * field.polarity, 2000),
                    aux.Point(300 * field.polarity, -2000),
                ]
            elif const.DIV == "C":
                poses = [
                    aux.Point(650 * field.polarity, 0),
                    aux.Point(300 * field.polarity, 1000),
                ]
        else:
            if const.DIV == "B":
                poses = [
                    aux.Point(650 * field.polarity, -150),
                    aux.Point(650 * field.polarity, 150),
                    aux.Point(3500 * field.polarity, -150),
                    aux.Point(3500 * field.polarity, 150),
                    aux.Point(300 * field.polarity, 2000),
                ]
            elif const.DIV == "C":
                poses = [
                    aux.Point(650 * field.polarity, 0),
                    aux.Point(1500 * field.polarity, 0),
                ]

        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used() and field.allies[i].r_id != field.gk_id:
                robot_pos = field.allies[i].get_pos()
                best_pos = None
                min_dist: float
                for pos in enumerate(poses):
                    dist = aux.dist(pos[1], robot_pos)
                    if best_pos is None or dist < min_dist:
                        best_pos = pos
                        min_dist = dist
                if best_pos is None:
                    continue

                poses.pop(best_pos[0])
                waypoints[i] = wp.Waypoint(
                    best_pos[1],
                    aux.angle_to_point(robot_pos, field.ball.get_pos()),
                    wp.WType.S_ENDPOINT,
                )

        waypoints[field.gk_id] = wp.Waypoint(
            field.ally_goal.center,
            aux.angle_to_point(field.ally_goal.center, field.ball.get_pos()),
            wp.WType.S_ENDPOINT,
        )

    def kickoff(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Удар мяча из аута"""
        self.put_kickoff_waypoints(field, waypoints)
        # self.we_kick = 1
        go_kick = fld.find_nearest_robot(field.ball.get_pos(), field.allies)
        if self.we_kick:
            # self.attacker(field, waypoints, go_kick.r_id)
            waypoints[go_kick.r_id] = wp.Waypoint(
                field.ball.get_pos(),
                aux.angle_to_point(field.ball.get_pos(), field.enemy_goal.center),
                wp.WType.S_BALL_KICK_UP,
            )
        else:
            target = aux.point_on_line(
                field.ball.get_pos(), field.ally_goal.center, 250
            )
            waypoints[go_kick.r_id] = wp.Waypoint(
                target,
                aux.angle_to_point(
                    field.allies[go_kick.r_id].get_pos(), field.ball.get_pos()
                ),
                wp.WType.S_IGNOREOBSTACLES,
            )


def delete_role(roles: list[Role], role_to_delete: Role) -> None:
    """Удаляет роль из массива (если роли в массиве нет, удаляет роль самого низкого приоритета)"""
    for i, role in enumerate(roles):
        if role == role_to_delete:
            roles.pop(i)
            return
    roles.pop()


def find_role(
    field: fld.Field, roles: list[Role], role_to_find: Role
) -> list[rbt.Robot]:
    """Возвращает массив со всеми роботами роли role_to_find"""
    robots: list[rbt.Robot] = []
    for i, role in enumerate(roles):
        if role == role_to_find:
            robots.append(field.allies[i])

    return robots
