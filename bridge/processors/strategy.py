"""Верхнеуровневый код стратегии"""
# pylint: disable=redefined-outer-name

# @package Strategy
# Расчет требуемых положений роботов исходя из ситуации на поле

import math

# !v DEBUG ONLY
import typing
from enum import Enum
from time import time
from typing import Optional

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.drawing as draw
import bridge.processors.field as fld
import bridge.processors.ref_states as refs
import bridge.processors.robot as robot
import bridge.processors.signal as signal
import bridge.processors.twisted_kick as twist
import bridge.processors.waypoint as wp


class States(Enum):
    """Класс с глобальными состояниями игры"""

    DEBUG = 0
    DEFENSE = 1
    ATTACK = 2


class GameStates(Enum):
    """Класс с командами от судей"""

    HALT = 0
    STOP = 1
    RUN = 2
    TIMEOUT = 3
    PREPARE_KICKOFF = 5
    KICKOFF = 6
    PREPARE_PENALTY = 7
    PENALTY = 8
    FREE_KICK = 9
    BALL_PLACEMENT = 11


class ActiveTeam(Enum):
    """Класс с командами"""

    ALL = 0
    YELLOW = 1
    BLUE = 2


class Role(Enum):
    """Класс с ролями"""

    GOALKEEPER = 0
    ATTACKER = 1

    WALLLINER = 3
    FORWARD = 11

    UNAVAILABLE = 100


class Strategy:
    """Основной класс с кодом стратегии"""

    def __init__(self, dbg_game_status: GameStates = GameStates.RUN, dbg_state: States = States.ATTACK) -> None:
        self.refs = refs.RefStates()

        self.game_status = dbg_game_status
        self.active_team: ActiveTeam = ActiveTeam.ALL
        self.state = dbg_state
        self.timer = time()
        self.timer1 = None
        self.is_ball_moved = 0

        # DEFENSE
        self.old_def_helper = -1
        self.old_def = -1
        self.steal_flag = 0

        # ATTACK
        self.forwards: list[robot.Robot] |  None = []
        self.prev_roles: list[Role] = [Role.UNAVAILABLE for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]
        self.robot_with_ball: typing.Optional[int] = None
        self.connector: list[int] = []
        # self.popusk: list[int] = []
        self.attack_state = "TO_BALL"
        self.attack_pos = aux.Point(0, 0)
        self.calc = False
        self.point_res = aux.Point(0, 0)
        self.used_pop_pos = [False, False, False, False, False]

        self.image = draw.Image()
        self.image.draw_field()
        self.ball_start_point: Optional[aux.Point] = None
        self.twist_w: float = 0.5

        self.wait_kick_timer: Optional[float] = None

        self.desision_flag = 0
        self.current_dessision: tuple[Optional[aux.Point], float] = [None, 0.0]
        self.desision_border = 0.2

        self.ball_history: list[Optional[aux.Point]] = [None] * round(0.3 / const.Ts)
        self.ball_pass_update_timer = 0
        self.ball_history_idx = 0
        self.old_ball_pos = None

    def change_game_state(self, new_state: GameStates, upd_active_team: int) -> None:
        """Изменение состояния игры и цвета команды"""
        self.game_status = new_state
        if upd_active_team == 0:
            self.active_team = ActiveTeam.ALL
        elif upd_active_team == 2:
            self.active_team = ActiveTeam.YELLOW
        elif upd_active_team == 1:
            self.active_team = ActiveTeam.BLUE

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

        free_allies = -atk_min - def_min - 2
        for ally in field.allies:
            if ally.is_used():
                free_allies += 1

        free_allies = max(0, free_allies)
        ball_pos = aux.minmax(field.ball.get_pos().x, -const.GOAL_DX, const.GOAL_DX)
        atks = round(free_allies / (2 * const.GOAL_DX) * (ball_pos * field.polarity + const.GOAL_DX)) + atk_min
        defs = free_allies - (atks - atk_min) + def_min
        print("attackers + defenses:", atks, "+", defs)

        # print(ATTACK_ROLES[:atks], DEFENSE_ROLES[:defs])
        roles = ATTACK_ROLES[:atks] + DEFENSE_ROLES[:defs]
        return [Role.GOALKEEPER, Role.ATTACKER] + sorted(roles, key = lambda x: x.value)

    def choose_robots_for_roles(self, field: fld.Field, roles: list[Role], wall_pos: aux.Point) -> list[Role]:
        robot_roles: list[Role] = [Role.UNAVAILABLE for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]
        used_ids: list[int] = []

        for robot_id, role in enumerate(self.prev_roles):
            if role in [Role.FORWARD, Role.WALLLINER] and field.is_ball_moves_to_point(field.allies[robot_id].get_pos()):
                used_ids.append(robot_id)
                robot_roles[robot_id] = role
                delete_role(roles, role)

        for role in roles:
            robot_id = -1
            if role == Role.GOALKEEPER:
                robot_id = const.GK
            elif role == Role.ATTACKER:
                robot_id = fld.find_nearest_robot(field.ball.get_pos(), field.allies, used_ids).r_id
            elif role == Role.WALLLINER:
                robot_id = fld.find_nearest_robot(wall_pos, field.allies, used_ids).r_id
            elif role == Role.FORWARD:
                robot_id = fld.find_nearest_robot(field.enemy_goal.center, field.allies, used_ids).r_id
            robot_roles[robot_id] = role
            used_ids.append(robot_id)

        self.prev_roles = robot_roles
        return robot_roles

    def process(self, field: fld.Field) -> list[wp.Waypoint]:
        """
        Рассчитать конечные точки для каждого робота
        """
        if      self.active_team == ActiveTeam.ALL or \
                field.ally_color == "b" and self.active_team == ActiveTeam.BLUE or \
                field.ally_color == "y" and self.active_team == ActiveTeam.YELLOW:
            self.refs.we_active = 1
        else:
            self.refs.we_active = 0

        if self.ball_history[self.ball_history_idx] is None:
            self.ball_start_point = self.ball_history[0]
        else:
            self.ball_start_point = self.ball_history[self.ball_history_idx]

        self.ball_history[self.ball_history_idx] = field.ball.get_pos()
        self.ball_history_idx += 1
        self.ball_history_idx %= len(self.ball_history)

        waypoints: list[wp.Waypoint] = []
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoints.append(wp.Waypoint(field.allies[i].get_pos(), field.allies[i].get_angle(), wp.WType.S_STOP))

        if self.game_status != GameStates.PENALTY:
            self.refs.is_started = 0

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
                    robot_with_ball = fld.find_nearest_robot(field.ball.get_pos(), field.enemies)
                    waypoints[const.GK] = self.goalk(field, 0, robot_with_ball)[0]
            elif self.game_status == GameStates.BALL_PLACEMENT:
                self.refs.keep_distance(field, waypoints)
            elif self.game_status == GameStates.PREPARE_KICKOFF:
                self.refs.prepare_kickoff(field, waypoints)
            elif self.game_status == GameStates.KICKOFF:
                self.refs.kickoff(field, waypoints)
                robot_with_ball = fld.find_nearest_robot(field.ball.get_pos(), field.enemies)
                waypoints[const.GK] = self.goalk(field, 0, robot_with_ball)[0]
            # elif self.game_status == GameStates.FREE_KICK:
            #     self.free_kick(field, waypoints)
            elif self.game_status == GameStates.STOP:
                self.refs.keep_distance(field, waypoints)
        # print(self.game_status, self.state)

        for rbt in field.allies:
            if rbt.is_used():
                self.image.draw_robot(rbt.get_pos(), rbt.get_angle())
        self.image.draw_dot(field.ball.get_pos(), 5)
        self.image.draw_poly(field.ally_goal.hull)
        self.image.draw_poly(field.enemy_goal.hull)
        self.image.update_window()
        self.image.draw_field()

        return waypoints

    def run(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """
        Определение глобального состояния игры
        roles - роли роботов, отсортированные по приоритету
        robot_roles - список соответствия id робота и его роли
        """

        "Определение набора ролей для роботов"
        roles = self.choose_roles(field)
        print(roles)

        "Вычисление конечных точек, которые не зависят от выбранного робота"
        wall_enemy = field.ball.get_pos()
        wall_pos = self.calc_wall_pos(field, wall_enemy)


        "Выбор роботов для всех ролей, с учетом вычесленных выше точек"
        robot_roles = self.choose_robots_for_roles(field, roles, wall_pos)
        # print(robot_roles)

        "Вычисление конечных точек, которые зависят от положения робота и создание путевых точек"
        self.forwards = find_role(field, robot_roles, Role.FORWARD)
        if self.forwards is not None:
            self.set_forwards_wps(field, waypoints)

        if Role.ATTACKER in robot_roles:
            attacker_id = find_role(field, robot_roles, Role.ATTACKER)[0].r_id
            self.attacker(field, waypoints, attacker_id)

        if Role.WALLLINER in robot_roles:
            wallliners = find_role(field, robot_roles, Role.WALLLINER)
            self.set_wallliners_wps(field, waypoints, wallliners, wall_enemy)

        if Role.GOALKEEPER in robot_roles:
            waypoints[const.GK] = self.goalk(field, field.ally_with_ball)


    square = signal.Signal(15, "SQUARE", lohi=(-2000, -1000))
    square_ang = signal.Signal(4, "SQUARE", lohi=(0, 4))

    def debug(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> list[wp.Waypoint]:
        """Отладка"""

        fld.find_nearest_robot(field.ball.get_pos(), field.allies)
        # self.goalk(field, waypoints, [const.GK], robot_with_ball)

        waypoints[const.DEBUG_ID].pos = field.ball.get_pos()
        waypoints[const.DEBUG_ID].angle = (field.ally_goal.center - field.ball.get_pos() + aux.UP * self.square.get()).arg()
        waypoints[const.DEBUG_ID].type = wp.WType.S_BALL_KICK

        # print(field.ball.get_pos(), waypoints[const.DEBUG_ID])
        return waypoints

    def attacker(self, field: fld.Field, waypoints: list[wp.Waypoint], attacker_id: int) -> None:
        """Логика действий для робота с мячом"""
        tmp = self.choose_kick_point(field, attacker_id)
        est = self.estimate_pass_point(field, field.ball.get_pos(), tmp[0])
        # print(max(est, self.estimate_pass_point(field, field.ball.get_pos(), self.current_dessision[0])))
        if est >= self.estimate_pass_point(field, field.ball.get_pos(), self.current_dessision[0]):
            self.current_dessision[0] = tmp[0]
            self.current_dessision[1] = est

        if self.current_dessision[1] < self.desision_border and len(self.forwards) > 0:
            receiver_id = self.choose_receiver(field)
            self.pass_kicker(field, waypoints, attacker_id, receiver_id)
        else:
            waypoints[attacker_id] = wp.Waypoint(
                field.ball.get_pos(),
                aux.angle_to_point(field.allies[attacker_id].get_pos(), self.current_dessision[0]),
                wp.WType.S_BALL_KICK
            )

    def calc_wall_pos(self, field: fld.Field, ball: aux.Point) -> aux.Point:
        for i in range(len(field.ally_goal.hull) * 1): #NOTE: исправить пересечение прямой с углом зоны
            point = aux.get_line_intersection(field.ally_goal.hull[i-1], field.ally_goal.hull[i], field.ally_goal.center, ball, "SR")
            if point is not None:
                return point
        return field.ally_goal.frw

    def set_wall_targets(self, field: fld.Field, num: int, ball: aux.Point) -> list[aux.Point]:
        if aux.is_point_inside_poly(ball, field.ally_goal.hull):
            ball = fld.find_nearest_robot(field.ally_goal.center, field.enemies).get_pos()
        poses = []

        lines: list[tuple[aux.Point, aux.Point]] =[]
        intersections = []
        for i in range(len(field.ally_goal.hull) * 1): #NOTE: исправить пересечение прямой с углом зоны
            point = aux.get_line_intersection(field.ally_goal.hull[i-1], field.ally_goal.hull[i], ball, field.ally_goal.down, "SS")
            if point is not None:
                lines.append([field.ally_goal.hull[i-1], field.ally_goal.hull[i]])
                intersections.append(point)

            point = aux.get_line_intersection(field.ally_goal.hull[i-1], field.ally_goal.hull[i], ball, field.ally_goal.up, "SS")
            if point is not None:
                lines.append([field.ally_goal.hull[i-1], field.ally_goal.hull[i]])
                intersections.append(point)

        wall_vec = intersections[1] - intersections[0]
        if wall_vec.mag() > num * 2 * const.ROBOT_R:
            for i in range(num):
                delta = const.ROBOT_R * (2 * i + 1)
                poses.append(intersections[0] + wall_vec.unity() * delta)
                self.image.draw_dot(intersections[0] + wall_vec.unity() * delta, const.ROBOT_R * self.image.scale, (128, 128, 128))
        else:
            wall_middle = intersections[0] + wall_vec / 2
            for i in range(num):
                delta = const.ROBOT_R * (-(num - 1) + 2 * i)
                poses.append(wall_middle + wall_vec.unity() * delta)
                self.image.draw_dot(wall_middle + wall_vec.unity() * delta, const.ROBOT_R * self.image.scale, (200, 200, 200))

        return poses

    def set_wallliners_wps(self, field: fld.Field, waypoints: list[wp.Waypoint], wallliners: list[robot.Robot], enemy_pos: aux.Point) -> None:
        poses = self.set_wall_targets(field, len(wallliners), enemy_pos)

        for pos in poses:
            robot = fld.find_nearest_robot(pos, wallliners)
            waypoints[robot.r_id] = wp.Waypoint(pos, field.ally_goal.eye_forw.arg(), wp.WType.S_ENDPOINT)


    def choose_receiver(self, field: fld.Field) -> int:
        """Выбирает робота для получения паса"""
        receiver_id = None
        receiver_score = 0.0
        for forward in self.forwards:
            pass_score = self.estimate_pass_point(field, field.ball.get_pos(), self.current_dessision[0])
            kick_score = self.choose_kick_point(field, forward.r_id)[1]
            score = pass_score * kick_score
            if score > receiver_score or receiver_id is None:
                receiver_id = forward.r_id
                receiver_score = score
        return receiver_id

    def set_forwards_wps(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Расставляет роботов по точкам для получения паса"""
        pos_num = len(self.forwards)
        poses = [
            aux.Point(3500 * field.polarity, 1500),
            aux.Point(3500 * field.polarity, -1500),
            aux.Point(2500 * field.polarity, 0),
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
            print("forwards:",pop.r_id, pos)

    def pass_kicker(self, field: fld.Field, waypoints: list[wp.Waypoint], kicker_id: int, receiver_id: int) -> None:
        """
        Отдает пас от робота kicker_id роботу receiver_id
        Должна вызываться в конечном автомате постоянно, пока первый робот не даст пас
        """
        receiver = field.allies[receiver_id]
        if not field.is_ball_moves_to_point(receiver.get_pos()):
            waypoints[kicker_id] = wp.Waypoint(
                field.ball.get_pos(),
                aux.angle_to_point(field.ball.get_pos(), receiver.get_pos()),
                wp.WType.S_BALL_PASS,
            )
            self.image.draw_dot(
                field.ball.get_pos() + aux.rotate(aux.RIGHT, aux.angle_to_point(field.ball.get_pos(), receiver.get_pos())),
                5,
                (255, 0, 255),
            )
        else:
            waypoints[kicker_id].type = wp.WType.S_STOP

    def pass_receiver(
        self, field: fld.Field, waypoints: list[wp.Waypoint], receiver_id: int, receive_point: aux.Point
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
            and (self.ball_start_point - field.ball.get_pos()).mag() > const.INTERCEPT_SPEED
            and field.ally_with_ball is None
        ):
            target = aux.closest_point_on_line(self.ball_start_point, field.ball.get_pos(), receiver.get_pos(), "R")
            self.image.draw_dot(target, 5, (255, 255, 0))

            waypoints[receiver_id] = wp.Waypoint(
                target, aux.angle_to_point(field.ball.get_pos(), self.ball_start_point), wp.WType.S_ENDPOINT
            )
        else:
            waypoints[receiver_id] = wp.Waypoint(
                receive_point, aux.angle_to_point(receiver.get_pos(), field.ball.get_pos()), wp.WType.S_ENDPOINT
            )
            self.image.draw_dot(receive_point, 5, (255, 255, 0))

    def kick_with_rotation(
        self, field: fld.Field, waypoints: list[wp.Waypoint], kicker_id: int, kick_point: aux.Point
    ) -> None:
        """
        Прицеливание и удар в точку при условии, что мяч находится в захвате у робота
        """
        kicker = field.allies[kicker_id]

        # signed_A = aux.wind_down_angle(aux.angle_to_point(kicker.get_pos(), kick_point) - self.start_rotating_ang)
        # A = abs(signed_A)
        signed_x = aux.wind_down_angle(aux.angle_to_point(kicker.get_pos(), kick_point) - kicker.get_angle())
        x = abs(signed_x)

        beta = 3
        a = beta / x - self.twist_w / (x**2)
        b = 2 * x * a - beta
        w = self.twist_w + b * const.Ts
        self.twist_w = w
        if signed_x < 0:
            w *= -1
        # print(abs(w))
        waypoints[kicker_id] = twist.spin_with_ball(w)
        if x > const.KICK_ALIGN_ANGLE:
            field.allies[kicker_id].set_dribbler_speed(max(5, 15 - abs(w) * 5))
            self.wait_kick_timer = None
        else:
            # if self.wait_kick_timer is None:
            #     self.wait_kick_timer = time()
            # else:
            #     wt = 0.1
            # if time() - self.wait_kick_timer > wt:
            self.wait_kick_timer = None
            field.allies[kicker_id].kick_forward()
            # else:
            #     print(field.allies[kicker_id].dribbler_speed_)
            #     field.allies[kicker_id].set_dribbler_speed(max(6, 15 - (15 / wt) * (time() - self.wait_kick_timer)))

    def estimate_pass_point(self, field: fld.Field, frm: Optional[aux.Point], to: Optional[aux.Point]) -> float:
        """
        Оценивает пас из точки "frm" в точку "to, возвращая положительное значение до 0.8
        """
        if frm is None or to is None:
            return 0
        positions: list[tuple[int, aux.Point]] = []
        for rbt in field.allies:
            if rbt.is_used():
                positions.append([rbt.r_id, rbt.get_pos()])
        positions = sorted(positions, key=lambda x: x[1].y)

        tangents: list[tuple[int, tuple[aux.Point, aux.Point]]] = []
        for p in positions:
            tgs = aux.get_tangent_points(p[1], frm, const.ROBOT_R)
            if tgs is None or isinstance(tgs, aux.Point):
                # print("Err while estimate pass point", p, frm, tgs)
                continue
            # print(tgs[0], tgs[1])
            tangents.append([p[0], tgs])

        min_ = 10e3

        shadows_bots = []
        for tangent in tangents:
            ang1 = aux.get_angle_between_points(to, frm, tangent[1][0])
            ang2 = aux.get_angle_between_points(to, frm, tangent[1][1])

            if not ((ang1 >= 180 and ang2 >= 180) or (ang1 <= 180 and ang2 <= 180)):
                shadows_bots.append(tangent[0])

            if ang1 > 180:
                ang1 = 360 - ang1
            if ang2 > 180:
                ang2 = 360 - ang2

            if ang1 < min_:
                min_ = ang1
            if ang2 < min_:
                min_ = ang2
        # if minId == -1:
        #     return 0
        # print(min_, minId)
        if min_ == 10e3:
            return 0
        dist = (frm - to).mag() / 1000
        max_ang = 57.3 * aux.wind_down_angle(2 * math.atan2(const.ROBOT_SPEED, -0.25 * dist + 4.5)) / 2
        return min(abs(min_ / max_ang), 1)

    def choose_kick_point(
        self, field: fld.Field, kicker_id: int, ball_pos: Optional[aux.Point] = None
    ) -> tuple[aux.Point, float]:
        """
        Выбирает оптимальную точку в воротах для удара
        """
        if ball_pos is None:
            ball_pos = field.ball.get_pos()

        positions = []
        for rbt in field.allies:
            if rbt.r_id != kicker_id:
                if (
                    aux.dist(rbt.get_pos(), field.enemy_goal.center) < aux.dist(field.enemy_goal.center, ball_pos)
                    and rbt.is_used()
                ):
                    positions.append(rbt.get_pos())
        for rbt in field.enemies:
            if (
                aux.dist(rbt.get_pos(), field.enemy_goal.center) < aux.dist(field.enemy_goal.center, ball_pos)
                and rbt.is_used()
            ):
                positions.append(rbt.get_pos())

        positions = sorted(positions, key=lambda x: x.y)

        segments = [field.enemy_goal.up]
        for p in positions:
            tangents = aux.get_tangent_points(p, ball_pos, const.ROBOT_R)
            if tangents is None or isinstance(tangents, aux.Point):
                # print(p, ball_pos, tangents)
                continue

            int1 = aux.get_line_intersection(
                ball_pos,
                tangents[0],
                field.enemy_goal.down,
                field.enemy_goal.up,
                "RS",
            )
            int2 = aux.get_line_intersection(
                ball_pos,
                tangents[1],
                field.enemy_goal.down,
                field.enemy_goal.up,
                "RS",
            )
            if int1 is None and int2 is not None:
                segments.append(field.enemy_goal.up)
                segments.append(int2)
            elif int1 is not None and int2 is None:
                segments.append(int1)
                segments.append(field.enemy_goal.down)
            elif int1 is not None and int2 is not None:
                segments.append(int1)
                segments.append(int2)

        segments.append(field.enemy_goal.down)
        max_ = 0.0
        maxId = -1
        for i in range(0, len(segments), 2):
            c = segments[i]
            a = segments[i + 1]
            b = ball_pos
            if c.y > a.y:
                continue  # Shadow intersection
            ang = aux.get_angle_between_points(a, b, c)
            # print(ang, c.y, a.y)
            if ang > max_:
                max_ = ang
                maxId = i

        if maxId == -1:
            return field.enemy_goal.center, 0

        A = segments[maxId + 1]
        B = ball_pos
        C = segments[maxId]
        tmp1 = (C - B).mag()
        tmp2 = (A - B).mag()
        CA = A - C
        pnt = C + CA * 0.5 * (tmp1 / tmp2)
        self.image.draw_dot(pnt, 10, (255, 0, 0))

        if max_ > 180:
            max_ = 360 - max_

        dist = (ball_pos - pnt).mag() / 1000
        max_ang = 57.3 * aux.wind_down_angle(2 * math.atan2(const.ROBOT_SPEED, -0.25 * dist + 4.5)) / 2

        # print(max_, max_ang, max_ / max_ang)
        return pnt, min(max_ / max_ang, 1)

    def goalk(
        self,
        field: fld.Field,
        robot_with_ball: Optional[robot.Robot],
    ) -> wp.Waypoint:
        """
        Управление вратарём и стенкой
        Возвращает массив с элементами класса Waypoint, где 0й элемент - вратарь, а остальные - стенка
        """
        gk_pos: aux.Point | None = None
        if robot_with_ball is not None:
            predict = aux.get_line_intersection(
                robot_with_ball.get_pos(),
                robot_with_ball.get_pos() + aux.rotate(aux.RIGHT, robot_with_ball.get_angle()),
                field.ally_goal.down,
                field.ally_goal.up,
                "RS",
            )
            if predict is not None:
                p_ball = (field.ball.get_pos() - predict).unity()
                gk_pos = aux.lerp(
                    aux.point_on_line(field.ally_goal.center, field.ball.get_pos(), const.GK_FORW),
                    p_ball * const.GK_FORW
                    + aux.get_line_intersection(
                        robot_with_ball.get_pos(),
                        robot_with_ball.get_pos() + aux.rotate(aux.RIGHT, robot_with_ball.get_angle()),
                        field.ally_goal.down,
                        field.ally_goal.up,
                        "RS",
                    ),
                    0.5,
                )

        if (
            field.is_ball_moves_to_goal()
            and self.ball_start_point is not None
            and (self.ball_start_point - field.ball.get_pos()).mag() > const.INTERCEPT_SPEED
        ):
            tmp_pos = aux.get_line_intersection(
                self.ball_start_point, field.ball.get_pos(), field.ally_goal.down, field.ally_goal.up, "RS"
            )
            if tmp_pos is not None:
                gk_pos = aux.closest_point_on_line(field.ball.get_pos(), tmp_pos, field.allies[const.GK].get_pos())

        if gk_pos is None:
            gk_pos = aux.point_on_line(
                field.ally_goal.center - field.ally_goal.eye_forw * 1000, field.ball.get_pos(), const.GK_FORW + 1000
            )
            gk_pos.x = min(field.ally_goal.center.x + field.ally_goal.eye_forw.x * 300, gk_pos.x, key=lambda x: abs(x))
            if abs(gk_pos.y) > abs(field.ally_goal.up.y):
                gk_pos.y = abs(field.ally_goal.up.y) * abs(gk_pos.y) / gk_pos.y
            self.image.draw_dot(gk_pos, 10, (255, 255, 255))
        else:
            self.image.draw_dot(gk_pos, 10, (0, 0, 0))

        gk_angle = math.pi / 2
        gk_wp = wp.Waypoint(gk_pos, gk_angle, wp.WType.S_IGNOREOBSTACLES)

        if field.is_ball_stop_near_goal() or field.ally_with_ball == field.allies[const.GK]:
            gk_wp = wp.Waypoint(field.ball.get_pos(), field.ally_goal.eye_forw.arg(), wp.WType.S_BALL_KICK_UP)

        return gk_wp

def delete_role(roles: list[Role], role_to_delete: Role) -> None:
    """Удаляет роль из массива (если роли в массиве нет, удаляет роль самого низкого приоритета)"""
    for i, role in enumerate(roles):
        if role == role_to_delete:
            roles.pop(i)
            return
    roles.pop()

def find_role(field: fld.Field, roles: list[Role], role_to_find: Role) -> list[robot.Robot]:
    """Возвращает массив со всеми роботами роли role_to_find"""
    robots: list[robot.Robot] = []
    for i, role in enumerate(roles):
        if role == role_to_find:
            robots.append(field.allies[i])

    return robots
