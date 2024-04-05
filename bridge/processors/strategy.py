"""Верхнеуровневый код стратегии"""
# pylint: disable=redefined-outer-name

# @package Stratery
# Расчет требуемых положений роботов исходя из ситуации на поле

import math

# !v DEBUG ONLY
import typing
from enum import Enum
from typing import Optional
from time import time

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.drawing as draw
import bridge.processors.field as field  # pylint: disable = unused-import
import bridge.processors.robot as robot
import bridge.processors.signal as signal
import bridge.processors.waypoint as wp
import bridge.processors.twisted_kick as twist


class States(Enum):
    """Класс с глобальными состояниями игры"""

    DEBUG = 0
    DEFENCE = 1
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
    BALL_PLACMENT = 11


class ActiveTeam(Enum):
    """Класс с командами"""

    ALL = 0
    YELLOW = 1
    BLUE = 2


class Strategy:
    """Основной класс с кодом стратегии"""

    def __init__(self, dbg_game_status: GameStates = GameStates.RUN, dbg_state: States = States.ATTACK) -> None:
        self.game_status = dbg_game_status
        self.active_team: ActiveTeam = ActiveTeam.ALL
        self.state = dbg_state
        self.we_active = 1
        self.timer = 0.0
        self.is_ball_moved = 0

        # DEFENCE
        self.old_def_helper = -1
        self.old_def = -1
        self.steal_flag = 0

        # ATTACK
        self.robot_with_ball: typing.Optional[int] = None
        self.connector: list[int] = []
        self.popusk: list[int] = []
        self.attack_state = "TO_BALL"
        self.attack_pos = aux.Point(0, 0)
        self.calc = False
        self.point_res = aux.Point(0, 0)
        self.used_pop_pos = [False, False, False, False, False]

        # PENALTY
        self.we_kick = 0
        self.is_started = 0
        self.penalty_timer = 0

        self.image = draw.Image()
        self.ball_start_point: Optional[aux.Point] = None
        self.start_rotating: float = 0.0

    def change_game_state(self, new_state: GameStates, upd_active_team: int) -> None:
        """Изменение состояния игры и цвета команды"""
        self.game_status = new_state
        if upd_active_team == 0:
            self.active_team = ActiveTeam.ALL
        elif upd_active_team == 2:
            self.active_team = ActiveTeam.YELLOW
        elif upd_active_team == 1:
            self.active_team = ActiveTeam.BLUE

    def process(self, field: field.Field) -> list[wp.Waypoint]:
        """
        Рассчитать конечные точки для каждого робота
        """
        if field.ally_color == "b":
            if self.active_team == ActiveTeam.BLUE:
                self.we_active = 1
            else:
                self.we_active = 0
        else:
            if self.active_team == ActiveTeam.YELLOW:
                self.we_active = 1
            else:
                self.we_active = 0
        if self.active_team == ActiveTeam.ALL:
            self.we_active = 1

        if field.is_ball_moves():
            if self.ball_start_point is None:
                self.ball_start_point = field.ball.get_pos()
        else:
            self.ball_start_point = None

        waypoints: list[wp.Waypoint] = []
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoints.append(wp.Waypoint(field.allies[i].get_pos(), field.allies[i].get_angle(), wp.WType.S_ENDPOINT))

        if self.game_status != GameStates.PENALTY:
            self.is_started = 0

        if self.game_status == GameStates.RUN:
            self.run(field, waypoints)
        else:
            if self.game_status == GameStates.TIMEOUT:
                self.timeout(field, waypoints)
            elif self.game_status == GameStates.HALT:
                pass
                # self.halt(field, waypoints)
            elif self.game_status == GameStates.PREPARE_PENALTY:
                self.prepare_penalty(field, waypoints)
            elif self.game_status == GameStates.PENALTY:
                self.penalty(field, waypoints)
            elif self.game_status == GameStates.BALL_PLACMENT:
                self.keep_distance(field, waypoints)
            elif self.game_status == GameStates.PREPARE_KICKOFF:
                self.prepare_kickof(field, waypoints)
            elif self.game_status == GameStates.KICKOFF:
                self.kickoff(field, waypoints)
            elif self.game_status == GameStates.FREE_KICK:
                self.free_kick(field, waypoints)
            elif self.game_status == GameStates.STOP:
                self.keep_distance(field, waypoints)
        # print(self.game_status, self.state)

        for id in [9, 10, 8]:
            self.image.draw_robot(field.allies[id].get_pos(), field.allies[id].get_angle())
        self.image.update_window()

        return waypoints

    def run(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        """Определение глобального состояния игры"""
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoints[i] = wp.Waypoint(field.allies[i].get_pos(), field.allies[i].get_angle(), wp.WType.S_STOP)

        """
        if field.ball.get_vel().mag() < 200:
            if self.is_ball_moved or time.time() - self.timer > 0.1:
                rivals_robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.enemies, [const.ENEMY_GK])
                allies_robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.allies, [const.GK])
                self.timer = time.time()
                if aux.dist(rivals_robot_with_ball.get_pos(), field.ball.get_pos()) < 0.7 * aux.dist(
                    allies_robot_with_ball.get_pos(), field.ball.get_pos()
                ):  # TODO: улучшить определение защиты/атаки
                    self.state = States.DEFENCE
                else:
                    self.state = States.ATTACK
                    self.reset_all_attack_var()
                    self.robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.allies, [const.GK]).r_id
            self.is_ball_moved = 0
            if (
                abs(const.GOAL_DX - abs(field.ball.get_pos().x)) < const.GOAL_DY
                and abs(field.ball.get_pos().y) < const.GOAL_DY
            ):
                if abs(field.ball.get_pos().x - field.enemy_goal.center.x) < abs(
                    field.ball.get_pos().x - field.ally_goal.center.x
                ):
                    self.state = States.DEFENCE
                else:
                    if self.state == States.DEFENCE:
                        self.reset_all_attack_var()
                    self.state = States.ATTACK
        else:
            self.is_ball_moved = 1
        wall = []

        # self.state = States.ATTACK
        # self.state = States.DEFENCE

        if self.state == States.DEBUG:
            self.debug(field, waypoints)
        elif self.state == States.DEFENCE:
            wall = self.defence(field, waypoints)
        elif self.state == States.ATTACK:
            self.decide_popusk_position(field)
            self.pre_attack(field)
            self.attack(field, waypoints)

        if self.state != States.DEFENCE:
            self.old_def_helper = -1
            self.old_def = -1
            self.steal_flag = 0
        """

        # pnt = self.choose_kick_point(field, a_id)
        # if pnt is None:
        #     pnt = field.enemy_goal.center

        # waypoints[a_id] = wp.Waypoint(field.ball.get_pos(), aux.angle_to_point(field.ball.get_pos(), pnt), wp.WType.S_BALL_KICK)
        # self.goalk(field, waypoints, [const.GK], None)

        # # self.passs(field, waypoints, 14, 8)

        # if field.is_ball_in(field.allies[14]):
        #     w = 2 * 3.14 / 4
        #     delta_r = aux.rotate(aux.Point(200, 0), math.pi * 1.2)
        #     waypoints[14] = wp.Waypoint(delta_r, w, wp.WType.S_VELOCITY)
        # else:
        #     waypoints[14] = wp.Waypoint(field.ball.get_pos(), 0, wp.WType.S_BALL_GRAB)

        # waypoints[14]  = wp.Waypoint(aux.Point(self.square.get(), -2000), 1.507, wp.WType.S_ENDPOINT)
        id = 14
        w = 2 * 3.14 / 10
        waypoints[id] = twist.spin_with_ball(w)
        if field.allies[id].get_pos().x == 0:
            return
        if not "x_min" in globals():
            global x_min, x_max
            self.timer = time()
            x_min = field.allies[id].get_pos().x
            x_max = field.allies[id].get_pos().x
        else:
            x = field.allies[id].get_pos().x
            if x < x_min:
                x_min = x

            elif x > x_max:
                x_max = x
            print("real radius: ", (x_max - x_min) / 2)
            # print("vel: ", field.allies[id].get_vel(), "\t; ang_vel: ", field.allies[id].get_anglevel())
            # print("angle speed: ", w, "\t -> ", (field.allies[id].get_angle() - ang00) / (time() - self.timer))


    square = signal.Signal(15, "SQUARE", ampoffset=(-1200, -2500))
    square_ang = signal.Signal(4, "SQUARE", lohi=(0, 1))

    def debug(self, field: field.Field, waypoints: list[wp.Waypoint]) -> list[wp.Waypoint]:
        """Отладка"""

        robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.allies)
        self.goalk(field, waypoints, [const.GK], robot_with_ball)

        waypoints[const.DEBUG_ID].pos = field.ball.get_pos()
        waypoints[const.DEBUG_ID].angle = (field.ally_goal.center - field.ball.get_pos() + aux.UP * self.square.get()).arg()
        waypoints[const.DEBUG_ID].type = wp.WType.S_BALL_KICK

        # print(field.ball.get_pos(), waypoints[const.DEBUG_ID])
        return waypoints

    def passs(self, field: field.Field, waypoints: list[wp.Waypoint], kicker_id: int, receiver_id: int) -> None:
        """
        Отдает пас от робота kicker_id роботу receiver_id
        Должна вызываться в конечном автомате постоянно, пока второй робот не поймает мяч
        TODO: прописать действия отдающего пас робота после удара и принимающего пас робота до удара
        """
        robot_near_ball = robot.find_nearest_robot(field.ball.get_pos(), field.all_bots)
        field.allies[receiver_id].set_dribbler_speed(15)
        if not field.is_ball_moves() and robot_near_ball == field.allies[kicker_id]:
            waypoints[kicker_id] = wp.Waypoint(
                field.ball.get_pos(),
                aux.angle_to_point(field.ball.get_pos(), aux.Point(-2500, -2000)),
                wp.WType.S_BALL_KICK,
            )
            waypoints[receiver_id] = wp.Waypoint(
                aux.Point(-2500, -2000),
                aux.angle_to_point(field.allies[receiver_id].get_pos(), field.ball.get_pos()),
                wp.WType.S_ENDPOINT,
            )

        elif (
            not field.is_ball_moves()
            and robot_near_ball == field.allies[receiver_id]
            or field.is_ball_in(field.allies[receiver_id])
        ):
            waypoints[kicker_id].type = wp.WType.S_STOP
            if (
                abs(
                    aux.angle_to_point(field.allies[receiver_id].get_pos(), field.enemy_goal.center)
                    - field.allies[receiver_id].get_angle()
                )
                > const.KICK_ALIGN_ANGLE
            ):
                w = 2 * 3.14 / 4
                delta_r = aux.rotate(aux.Point(200, 0), math.pi * 1.2)
                waypoints[receiver_id] = wp.Waypoint(delta_r, w, wp.WType.S_VELOCITY)
            else:
                w = 2 * 3.14 / 4
                delta_r = aux.rotate(aux.Point(200, 0), math.pi * 1.2)
                field.allies[receiver_id].kick_forward()
                waypoints[receiver_id] = wp.Waypoint(delta_r * 1000, w / 1000, wp.WType.S_VELOCITY)

        elif (
            field.is_ball_moves_to_point(field.allies[receiver_id].get_pos())
            and self.ball_start_point is not None
            and (self.ball_start_point - field.ball.get_pos()).mag() > const.INTERCEPT_SPEED
        ):
            print("caching a ball")
            target = aux.closest_point_on_line(
                self.ball_start_point, field.ball.get_pos(), field.allies[receiver_id].get_pos(), "R"
            )

            waypoints[receiver_id] = wp.Waypoint(
                target, aux.angle_to_point(field.ball.get_pos(), self.ball_start_point), wp.WType.S_ENDPOINT
            )
            waypoints[kicker_id].type = wp.WType.S_STOP
        else:
            print("pass failed", 0)
            waypoints[receiver_id].type = wp.WType.S_STOP
            waypoints[kicker_id].type = wp.WType.S_STOP
        print(field.ball.get_vel())

    def defence(
        self, field: field.Field, waypoints: list[wp.Waypoint], ENDPOINT_TYPE: wp.WType = wp.WType.S_ENDPOINT
    ) -> list[int]:
        """Защита"""
        dist_between = 200

        works_robots = []
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used():
                field.allies[i].dribbler_enable_ = 0
                field.allies[i].auto_kick_ = 0
                works_robots.append(field.allies[i])
        total_robots = len(works_robots)

        used_robots_id = []
        if field.allies[const.GK].is_used():
            used_robots_id = [const.GK]
        robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.enemies)

        def1 = robot.find_nearest_robot(field.ball.get_pos(), field.allies, used_robots_id)

        # FOR BALL STEALING
        target_point = aux.point_on_line(field.ball.get_pos(), field.ally_goal.center, dist_between)
        if ENDPOINT_TYPE == wp.WType.S_KEEP_BALL_DISTANCE:
            if aux.dist(target_point, field.ball.get_pos()) < const.KEEP_BALL_DIST:
                target_point = aux.point_on_line(
                    aux.point_on_line(target_point, field.ball.get_pos(), -const.KEEP_BALL_DIST),
                    aux.Point(field.polarity * const.GOAL_DX, 0),
                    dist_between,
                )

        waypoint = wp.Waypoint(target_point, aux.angle_to_point(target_point, field.ball.get_pos()), ENDPOINT_TYPE)
        waypoints[def1.r_id] = waypoint
        self.old_def = def1.r_id

        used_robots_id.append(def1.r_id)

        wall_bots = []
        for r in works_robots:
            i = r.r_id
            if field.allies[i].r_id not in used_robots_id:
                wall_bots.append(field.allies[i].r_id)
                used_robots_id.append(field.allies[i].r_id)
        return sorted(wall_bots)

        rbs = sorted(field.enemies, reverse=True, key=lambda x: x.get_pos().x)
        rbs_r_ids = []
        x_attack = 3500

        for r in rbs:
            if (
                field.polarity * r.get_pos().x > 0
                and r.r_id != robot_with_ball.r_id
                and r.is_used()
                and r.r_id != const.ENEMY_GK
            ):
                rbs_r_ids.append(r.r_id)

        # FOR BALL STEALING
        wall_bots = []
        for _ in range(3):  # TODO: change number of wall_bots
            if total_robots - len(used_robots_id) == 0:
                break
            wall_bot = robot.find_nearest_robot(field.ally_goal.center, field.allies, used_robots_id)
            wall_bots.append(wall_bot.r_id)
            used_robots_id.append(wall_bot.r_id)

        for i, rbs_r_id in enumerate(rbs_r_ids):
            if len(used_robots_id) > total_robots:
                defender = robot.find_nearest_robot(field.enemies[rbs_r_id].get_pos(), field.allies, used_robots_id)
                used_robots_id.append(defender.r_id)

                target_point = aux.point_on_line(field.enemies[rbs_r_id].get_pos(), field.ball.get_pos(), dist_between)
                waypoint = wp.Waypoint(target_point, aux.angle_to_point(target_point, field.ball.get_pos()), ENDPOINT_TYPE)
                waypoints[defender.r_id] = waypoint

        if total_robots - len(used_robots_id) == 1:
            for r in works_robots:
                i = r.r_id
                if i not in used_robots_id:
                    target_point = aux.Point(field.polarity * -x_attack, 1500)
                    waypoint = wp.Waypoint(
                        target_point, aux.angle_to_point(target_point, field.ball.get_pos()), ENDPOINT_TYPE
                    )
                    waypoints[i] = waypoint
                    used_robots_id.append(i)
        elif total_robots - len(used_robots_id) >= 2:
            def2 = robot.find_nearest_robot(aux.Point(field.polarity * -x_attack, 1500), field.allies, used_robots_id)
            used_robots_id.append(def2.r_id)
            target_point = aux.Point(field.polarity * -x_attack, 1500)
            waypoint = wp.Waypoint(target_point, aux.angle_to_point(target_point, field.ball.get_pos()), ENDPOINT_TYPE)
            waypoints[def2.r_id] = waypoint

            for r in works_robots:
                i = r.r_id
                if field.allies[i].r_id not in used_robots_id:
                    used_robots_id.append(field.allies[i].r_id)
                    target_point = aux.Point(field.polarity * -x_attack, -1500)
                    waypoint = wp.Waypoint(
                        target_point, aux.angle_to_point(target_point, field.ball.get_pos()), ENDPOINT_TYPE
                    )
                    waypoints[field.allies[i].r_id] = waypoint
                    break

        for r in works_robots:
            i = r.r_id
            if field.allies[i].r_id not in used_robots_id:
                wall_bots.append(field.allies[i].r_id)
                used_robots_id.append(field.allies[i].r_id)
        return sorted(wall_bots)

    def choose_kick_point(self, field: field.Field, robot_inx: int) -> Optional[aux.Point]:
        ball_pos = field.ball.get_pos()

        positions = []
        for robot in field.allies:
            if robot.r_id != field.allies[robot_inx].r_id:
                if aux.dist(robot.get_pos(), field.enemy_goal.center) < aux.dist(field.enemy_goal.center, ball_pos):
                    positions.append(robot.get_pos())
        for robot in field.enemies:
            if aux.dist(robot.get_pos(), field.enemy_goal.center) < aux.dist(field.enemy_goal.center, ball_pos):
                positions.append(robot.get_pos())

        positions = sorted(positions, key=lambda x: x.y)

        segments = [field.enemy_goal.up]
        for p in positions:
            tangents = aux.get_tangent_points(p, ball_pos, const.ROBOT_R)
            if tangents is None or isinstance(tangents, aux.Point):
                print(p, ball_pos, tangents)
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
            return None

        A = segments[maxId + 1]
        B = ball_pos
        C = segments[maxId]
        tmp1 = (C - B).mag()
        tmp2 = (A - B).mag()
        CA = A - C
        pnt = C + CA * 0.5 * (tmp1 / tmp2)
        self.image.draw_dot(pnt, 10, (255, 0, 0))
        return pnt

    def reset_all_attack_var(self) -> None:
        """Обнуление глобальных параметров атаки, используется при смене состояния на атаку"""
        # return
        self.used_pop_pos = [False, False, False, False, False]
        self.robot_with_ball = None
        self.connector = []
        self.popusk = []
        self.attack_state = "TO_BALL"
        self.attack_pos = aux.Point(0, 0)
        self.calc = False
        self.point_res = aux.Point(0, 0)

    def decide_popusk_position(self, field: field.Field) -> None:
        """
        Выбор ролей для нападающих
        """
        for point_ind, popusk_position in enumerate(field.enemy_goal.popusk_positions):
            save_robot = -1
            if not self.used_pop_pos[point_ind]:
                mn = 1e10
                for robo in field.allies:
                    if (
                        robo.is_used()
                        and robo.r_id != const.GK
                        and robo.r_id != self.robot_with_ball
                        and not (robo.r_id in self.popusk)
                    ):
                        pop_pos_dist = aux.dist(robo.get_pos(), popusk_position)
                        if mn > pop_pos_dist:
                            mn = pop_pos_dist
                            save_robot = robo.r_id
                    elif not robo.is_used() and field.allies[robo.r_id].role != 0:
                        self.used_pop_pos[robo.role] = False
                        field.allies[robo.r_id].role = 0
                        if robo.r_id in self.popusk:
                            self.popusk.pop(self.popusk.index(robo.r_id))
            if save_robot != -1:
                field.allies[save_robot].role = point_ind
                self.popusk.append(save_robot)
                self.used_pop_pos[point_ind] = True

        # print(used_pop_pos)
        # print(self.popusk)

    def pre_attack(self, field: field.Field) -> None:
        """
        Выбор атакующего робота
        """
        if self.robot_with_ball is not None and field.ball.get_vel().mag() < 800:
            mn = 1e10
            for robo in field.allies:
                if robo.is_used() and robo.r_id != const.GK:
                    ball_dist = aux.dist(field.ball.get_pos(), robo.get_pos())
                    if mn > ball_dist:
                        mn = ball_dist
                        self.robot_with_ball = robo.r_id
            if self.robot_with_ball in self.popusk:
                self.popusk.pop(self.popusk.index(self.robot_with_ball))
        elif self.robot_with_ball is not None and not field.allies[self.robot_with_ball].is_used():
            field.allies[self.robot_with_ball].role = 0
            self.robot_with_ball = None

    def attack(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        """Атака"""
        for robo in self.popusk:
            pop_pos = field.allies[robo].role
            waypoints[robo] = wp.Waypoint(
                field.enemy_goal.popusk_positions[pop_pos],
                aux.angle_to_point(field.allies[robo].get_pos(), field.enemy_goal.center),
                wp.WType.S_ENDPOINT,
            )
        # print(self.point_res)
        if self.robot_with_ball is not None:
            self.attack_pos = field.ball.get_pos()
            if self.attack_state == "TO_BALL":
                self.point_res = field.enemy_goal.center
                if aux.in_place(self.attack_pos, field.allies[self.robot_with_ball].get_pos(), 1000):
                    self.attack_state = "CALCULATING"

            elif self.attack_state == "CALCULATING":
                self.point_res = self.choose_kick_point(field, self.robot_with_ball)
                if self.point_res is None:
                    print("self.point_res is None")
                    self.point_res = field.enemy_goal.center
                self.attack_state = "GO_TO_SHOOTING_POSITION"

            elif self.attack_state == "GO_TO_SHOOTING_POSITION":
                if aux.in_place(self.attack_pos, field.allies[self.robot_with_ball].get_pos(), 50):
                    self.attack_state = "SHOOT"

            elif self.attack_state == "SHOOT":
                self.attack_pos = field.ball.get_pos()
                if not aux.in_place(self.attack_pos, field.allies[self.robot_with_ball].get_pos(), 3000):
                    self.popusk.append(self.robot_with_ball)
                    self.attack_state = "TO_BALL"
                    self.robot_with_ball = None
            # ехать к нужной точке
            if self.robot_with_ball is not None:
                waypoints[self.robot_with_ball] = wp.Waypoint(
                    field.ball.get_pos(), aux.angle_to_point(field.ball.get_pos(), self.point_res), wp.WType.S_BALL_KICK
                )

    def prepare_penalty(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        """Подготовка пенальти по команде от судей"""
        if self.we_active:
            self.we_kick = 1
        else:
            self.we_kick = 0
        if self.we_kick:
            rC = 0
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if (
                    field.allies[i].is_used()
                    and field.allies[i].r_id != const.PENALTY_KICKER
                    and field.allies[i].r_id != const.GK
                ):
                    waypoint = wp.Waypoint(
                        aux.Point(3000 * field.polarity, 2500 - 1000 * rC), field.allies[i].get_angle(), wp.WType.S_ENDPOINT
                    )
                    waypoints[i] = waypoint
                    rC += 1
            waypoint = wp.Waypoint(
                aux.Point(1700 * field.polarity, 0),
                aux.angle_to_point(aux.Point(1700 * field.polarity, 0), field.ball.get_pos()),
                wp.WType.S_ENDPOINT,
            )
            waypoints[field.allies[const.PENALTY_KICKER].r_id] = waypoint
            waypoint = wp.Waypoint(
                field.ally_goal.center, aux.angle_to_point(field.ally_goal.center, field.ball.get_pos()), wp.WType.S_ENDPOINT
            )
            waypoints[field.allies[const.GK].r_id] = waypoint
        else:
            rC = 0
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if field.allies[i].is_used() and field.allies[i].r_id != const.GK:
                    waypoint = wp.Waypoint(aux.Point(3000 * -field.polarity, 2500 - 1000 * rC), 0, wp.WType.S_ENDPOINT)
                    waypoints[i] = waypoint
                    rC += 1
            waypoint = wp.Waypoint(
                field.ally_goal.center, aux.angle_to_point(field.ally_goal.center, field.ball.get_pos()), wp.WType.S_ENDPOINT
            )
            waypoints[field.allies[const.GK].r_id] = waypoint

    def halt(
        self, field: field.Field, waypoints: list[wp.Waypoint]
    ) -> None:  # TODO: проверить что роботы останавливаются на самом деле
        """Пауза по команде от судей"""
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used():
                waypoint = wp.Waypoint(field.allies[i].get_pos(), field.allies[i].get_angle(), wp.WType.S_ENDPOINT)
                waypoints[i] = waypoint

    def timeout(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        """Таймаут по команде от судей"""
        rC = 0
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used():
                waypoint = wp.Waypoint(aux.Point(2000 * field.polarity, 1250 - 500 * rC), 0, wp.WType.S_ENDPOINT)
                waypoints[i] = waypoint
                rC += 1

    def keep_distance(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        # TODO: вшить в waypoint или в стратегию удержание расстояния до мяча
        """Удержание расстояния до мяча по команде судей"""
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used():
                if aux.dist(field.allies[i].get_pos(), field.ball.get_pos()) < const.KEEP_BALL_DIST:
                    waypoint = wp.Waypoint(
                        aux.point_on_line(field.ball.get_pos(), aux.Point(0, 0), const.KEEP_BALL_DIST),
                        field.allies[i].get_angle(),
                        wp.WType.S_ENDPOINT,
                    )
                    waypoints[i] = waypoint

    def penalty(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        """Настройка пенальти по команде судей"""
        if self.we_kick or 1:
            self.penalty_kick(field, waypoints)
        else:
            robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.enemies)
            self.goalk(field, waypoints, [const.GK], robot_with_ball)

    def penalty_kick(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        """Пенальти (атака)"""
        self.is_started += 1
        if self.is_started < 5:
            field.allies[const.PENALTY_KICKER].kick_up_ = 1

        field.allies[const.PENALTY_KICKER].dribbler_enable_ = 1
        field.allies[const.PENALTY_KICKER].dribbler_speed_ = 10
        field.allies[const.PENALTY_KICKER].kicker_charge_enable_ = 2
        field.allies[const.PENALTY_KICKER].auto_kick_ = 2
        kick_delta = 400

        angle_to_keeper = math.atan2(
            field.enemies[const.ENEMY_GK].get_pos().y - field.allies[const.PENALTY_KICKER].get_pos().y,
            field.enemies[const.ENEMY_GK].get_pos().x - field.allies[const.PENALTY_KICKER].get_pos().x,
        )
        angle_to_right_corner = math.atan2(
            kick_delta - field.allies[const.PENALTY_KICKER].get_pos().y,
            field.enemy_goal.center.x - field.allies[const.PENALTY_KICKER].get_pos().x,
        )
        angle_to_left_corner = math.atan2(
            -kick_delta - field.allies[const.PENALTY_KICKER].get_pos().y,
            field.enemy_goal.center.x - field.allies[const.PENALTY_KICKER].get_pos().x,
        )

        if abs(angle_to_keeper - angle_to_right_corner) > abs(angle_to_keeper - angle_to_left_corner):
            target = aux.Point(field.enemy_goal.center.x, kick_delta)
        else:
            target = aux.Point(field.enemy_goal.center.x, -kick_delta)

        # print(field.ball.get_pos().x)
        if abs(field.enemy_goal.center.x - field.ball.get_pos().x) > 1000:
            field.allies[const.PENALTY_KICKER].kicker_voltage_ = 5
            waypoint = wp.Waypoint(
                field.ball.get_pos(), (target - field.allies[const.PENALTY_KICKER].get_pos()).arg(), wp.WType.S_BALL_KICK
            )
        else:
            field.allies[const.PENALTY_KICKER].kicker_voltage_ = 15
            waypoint = wp.Waypoint(
                field.ball.get_pos(), (target - field.allies[const.PENALTY_KICKER].get_pos()).arg(), wp.WType.S_BALL_KICK
            )

        waypoints[const.PENALTY_KICKER] = waypoint

    def prepare_kickof(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        """Настройка перед состоянием kickof по команде судей"""
        if self.we_active:
            self.we_kick = 1
        else:
            self.we_kick = 0
        self.put_kickoff_waypoints(field, waypoints)

    def put_kickoff_waypoints(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        """Подготовка перед состоянием kickof"""
        rC = 0
        if self.we_kick:
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if field.allies[i].is_used() and field.allies[i].r_id != const.GK:
                    if rC < 3:
                        if rC == 1:
                            waypoint = wp.Waypoint(
                                aux.Point(700 * field.polarity, 0),
                                aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                                wp.WType.S_ENDPOINT,
                            )
                        else:
                            waypoint = wp.Waypoint(
                                aux.Point(700 * field.polarity, 2000 - 2000 * rC),
                                aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                                wp.WType.S_ENDPOINT,
                            )
                        waypoints[i] = waypoint
                    else:
                        waypoint = wp.Waypoint(
                            aux.Point(200 * field.polarity, 1500 - 3000 * (rC - 3)),
                            aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                            wp.WType.S_ENDPOINT,
                        )
                        waypoints[i] = waypoint
                    rC += 1
        else:
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if field.allies[i].is_used() and field.allies[i].r_id != const.GK:
                    if rC == 0:
                        waypoint = wp.Waypoint(
                            aux.Point(700 * field.polarity, 0),
                            aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                            wp.WType.S_ENDPOINT,
                        )
                    elif rC < 3:
                        waypoint = wp.Waypoint(
                            aux.Point(200 * field.polarity, 1000 - 2000 * (rC - 1)),
                            aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                            wp.WType.S_ENDPOINT,
                        )
                    else:
                        waypoint = wp.Waypoint(
                            aux.Point(200 * field.polarity, 2000 + 4000 * (rC - 4)),
                            aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                            wp.WType.S_ENDPOINT,
                        )
                    waypoints[i] = waypoint
                    rC += 1
        waypoint = wp.Waypoint(
            field.ally_goal.center, aux.angle_to_point(field.ally_goal.center, field.ball.get_pos()), wp.WType.S_ENDPOINT
        )
        waypoints[field.allies[const.GK].r_id] = waypoint

    def kickoff(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        """Удар мяча из аута"""
        self.put_kickoff_waypoints(field, waypoints)
        # self.we_kick = 0
        if self.we_kick:
            go_kick = robot.find_nearest_robot(field.ball.get_pos(), field.allies)
            target = field.enemy_goal.center
            target.y = 300
            waypoint = wp.Waypoint(
                field.ball.get_pos(), (target - field.allies[go_kick.r_id].get_pos()).arg(), wp.WType.S_BALL_KICK
            )
            waypoints[go_kick.r_id] = waypoint
        else:
            go_kick = robot.find_nearest_robot(field.ball.get_pos(), field.allies)
            target = aux.point_on_line(field.ball.get_pos(), aux.Point(field.polarity * const.GOAL_DX, 0), 200)
            waypoint = wp.Waypoint(
                target, (field.ball.get_pos() - field.allies[go_kick.r_id].get_pos()).arg(), wp.WType.S_IGNOREOBSTACLES
            )
            waypoints[go_kick.r_id] = waypoint

        robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.enemies)
        self.goalk(field, waypoints, [const.GK], robot_with_ball)

    def free_kick(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        """Свободный удар (после любого нарушения/остановки игры) по команде судей"""
        wall = []
        if not self.we_active:
            wall = self.defence(field, waypoints, wp.WType.S_KEEP_BALL_DISTANCE)
            self.keep_distance(field, waypoints)
        else:
            self.state = States.ATTACK
            self.decide_popusk_position(field)
            self.pre_attack(field)
            self.attack(field, waypoints)
        robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.enemies)
        self.goalk(field, waypoints, [const.GK] + wall, robot_with_ball)

    def goalk(
        self, field: field.Field, waypoints: list[wp.Waypoint], gk_wall_idx_list: list[int], robot_with_ball: robot.Robot
    ) -> None:
        gk_pos = None
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
            and (self.ball_start_point - field.ball.get_pos()).mag() > const.INTERCEPT_SPEED * 0.5
        ):
            tmpPos = aux.get_line_intersection(
                self.ball_start_point, field.ball.get_pos(), field.ally_goal.down, field.ally_goal.up, "RS"
            )
            if tmpPos is not None:
                gk_pos = aux.closest_point_on_line(field.ball.get_pos(), tmpPos, field.allies[gk_wall_idx_list[0]].get_pos())

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
        waypoints[gk_wall_idx_list[0]] = wp.Waypoint(gk_pos, gk_angle, wp.WType.S_IGNOREOBSTACLES)

        self.image.draw_dot(field.ball.get_pos(), 5)

        if field.is_ball_stop_near_goal():
            waypoints[gk_wall_idx_list[0]] = wp.Waypoint(
                field.ball.get_pos(), field.ally_goal.eye_forw.arg(), wp.WType.S_BALL_KICK_UP
            )

        wallline = [field.ally_goal.frw + field.ally_goal.eye_forw * const.GOAL_WALLLINE_OFFSET]
        wallline.append(wallline[0] + field.ally_goal.eye_up)

        walline = aux.point_on_line(field.ally_goal.center, field.ball.get_pos(), const.GOAL_WALLLINE_OFFSET)
        walldir = aux.rotate((field.ally_goal.center - field.ball.get_pos()).unity(), math.pi / 2)
        dirsign = -aux.sign(aux.vec_mult(field.ally_goal.center, field.ball.get_pos()))

        wall = []
        for i in range(len(gk_wall_idx_list) - 1):
            wall.append(walline - walldir * (i + 1) * dirsign * (1 + (i % 2) * -2) * const.GOAL_WALL_ROBOT_SEPARATION)
            waypoints[gk_wall_idx_list[i + 1]] = wp.Waypoint(wall[i], walldir.arg(), wp.WType.S_IGNOREOBSTACLES)
