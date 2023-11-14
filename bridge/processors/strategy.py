"""Верхнеуровневый код стратегии"""
# pylint: disable=redefined-outer-name

# @package Stratery
# Расчет требуемых положений роботов исходя из ситуации на поле

import math

# !v DEBUG ONLY
import time
import typing
from enum import Enum

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.field as field  # pylint: disable = unused-import
import bridge.processors.robot as robot
import bridge.processors.signal as signal
import bridge.processors.waypoint as wp


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
        if const.OUR_COLOR == "b":
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
        return waypoints

    def run(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        """Определение глобального состояния игры"""
        if field.ball.get_vel().mag() < 300:
            if self.is_ball_moved or time.time() - self.timer > 0.1:
                rivals_robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.enemies, [const.ENEMY_GK])
                allies_robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.allies, [const.GK])
                self.timer = time.time()
                if aux.dist(rivals_robot_with_ball.get_pos(), field.ball.get_pos()) < aux.dist(
                    allies_robot_with_ball.get_pos(), field.ball.get_pos()
                ):  # TODO: улучшить определение защиты/атаки
                    self.state = States.DEFENCE
                    print("defence")
                else:
                    self.state = States.ATTACK
                    self.reset_all_attack_var()
                    self.robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.allies, [const.GK]).r_id
                    print("attack")
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

        robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.enemies)
        self.gk_go(field, waypoints, [const.GK] + wall, robot_with_ball)

    square = signal.Signal(8, "SQUARE", ampoffset=(300, 0))
    square_ang = signal.Signal(8, "SQUARE", lohi=(0, 3.14))

    def debug(self, field: field.Field, waypoints: list[wp.Waypoint]) -> list[wp.Waypoint]:
        """Отладка"""

        robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.allies)
        self.gk_go(field, waypoints, [const.GK], robot_with_ball)

        waypoints[const.DEBUG_ID].pos = field.ball.get_pos()
        waypoints[const.DEBUG_ID].angle = (field.ally_goal.center - field.ball.get_pos() + aux.UP * self.square.get()).arg()
        waypoints[const.DEBUG_ID].type = wp.WType.S_BALL_KICK

        # print(field.ball.get_pos(), waypoints[const.DEBUG_ID])
        return waypoints

    def gk_go(
        self, field: field.Field, waypoints: list[wp.Waypoint], gk_wall_idx_list: list[int], robot_with_ball: robot.Robot
    ) -> None:
        """
        Расчет требуемых положений вратаря и стенки

        [in] field - объект Field(), ситуация на поле
        [out] waypoints - ссылка на массив путевых точек, который будет изменен!
        [in] gk_wall_idx_list - массив индексов вратаря и стенки. Нулевой элемент всегда индекс вратаря
        Дальше индексы роботов в стенке, кол-во от 0 до 3
        [in] robot_with_ball - текущий робот с мячом
        """
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
                # print("PREDICTION: ", robot_with_ball.getAngle())
            else:
                # print("NO PREDICTION1")
                gk_pos = aux.point_on_line(field.ally_goal.center, field.ball.get_pos(), const.GK_FORW)
        else:
            # print("NO PREDICTION2")
            gk_pos = aux.point_on_line(field.ally_goal.center, field.ball.get_pos(), const.GK_FORW)

        # print(field.ball.vel.mag())
        if (
            field.is_ball_moves_to_goal()
            and aux.get_line_intersection(
                field.ally_goal.down,
                field.ally_goal.up,
                field.ball.get_pos(),
                field.ball.get_pos() + field.ball.get_vel(),
                "SR",
            )
            is not None
        ):
            gk_pos = aux.closest_point_on_line(
                field.ball.get_pos(), field.ball.get_vel() * 1000000, field.allies[gk_wall_idx_list[0]].get_pos()
            )
            # print("GK INTERCEPT", time.time())

        gk_angle = math.pi / 2
        waypoints[gk_wall_idx_list[0]] = wp.Waypoint(gk_pos, gk_angle, wp.WType.S_ENDPOINT)

        # print(field.isBallInGoalSq(), field.ball.get_pos())
        if field.is_ball_stop_near_goal():
            waypoints[gk_wall_idx_list[0]] = wp.Waypoint(
                field.ball.get_pos(), field.ally_goal.eye_forw.arg(), wp.WType.S_BALL_KICK
            )

        # wallline = [field.ally_goal.forw + field.ally_goal.eye_forw * const.GOAL_WALLLINE_OFFSET]
        # wallline.append(wallline[0] + field.ally_goal.eye_up)

        walline = aux.point_on_line(field.ally_goal.center, field.ball.get_pos(), const.GOAL_WALLLINE_OFFSET)
        walldir = aux.rotate((field.ally_goal.center - field.ball.get_pos()).unity(), math.pi / 2)
        dirsign = -aux.sign(aux.vect_mult(field.ally_goal.center, field.ball.get_pos()))

        wall = []
        for i in range(len(gk_wall_idx_list) - 1):
            wall.append(walline - walldir * (i + 1) * dirsign * (1 + (i % 2) * -2) * const.GOAL_WALL_ROBOT_SEPARATION)
            waypoints[gk_wall_idx_list[i + 1]] = wp.Waypoint(wall[i], walldir.arg(), wp.WType.S_ENDPOINT)

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
        target_point = aux.point_on_line(field.ball.get_pos(), aux.Point(const.POLARITY * const.GOAL_DX, 0), dist_between)
        if ENDPOINT_TYPE == wp.WType.S_KEEP_BALL_DISTANCE:
            if aux.dist(target_point, field.ball.get_pos()) < const.KEEP_BALL_DIST:
                target_point = aux.point_on_line(
                    aux.point_on_line(target_point, field.ball.get_pos(), -const.KEEP_BALL_DIST),
                    aux.Point(const.POLARITY * const.GOAL_DX, 0),
                    dist_between,
                )
        waypoint = wp.Waypoint(target_point, aux.angle_to_point(target_point, field.ball.get_pos()), ENDPOINT_TYPE)
        waypoints[def1.r_id] = waypoint
        self.old_def = def1.r_id

        used_robots_id.append(def1.r_id)
        rbs = sorted(field.enemies, reverse=True, key=lambda x: x.get_pos().x)
        rbs_r_ids = []
        x_attack = 3500

        for r in rbs:
            if (
                const.POLARITY * r.get_pos().x > 0
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
                    target_point = aux.Point(const.POLARITY * -x_attack, 1500)
                    waypoint = wp.Waypoint(
                        target_point, aux.angle_to_point(target_point, field.ball.get_pos()), ENDPOINT_TYPE
                    )
                    waypoints[i] = waypoint
                    used_robots_id.append(i)
        elif total_robots - len(used_robots_id) >= 2:
            def2 = robot.find_nearest_robot(aux.Point(const.POLARITY * -x_attack, 1500), field.allies, used_robots_id)
            used_robots_id.append(def2.r_id)
            target_point = aux.Point(const.POLARITY * -x_attack, 1500)
            waypoint = wp.Waypoint(target_point, aux.angle_to_point(target_point, field.ball.get_pos()), ENDPOINT_TYPE)
            waypoints[def2.r_id] = waypoint

            for r in works_robots:
                i = r.r_id
                if field.allies[i].r_id not in used_robots_id:
                    used_robots_id.append(field.allies[i].r_id)
                    target_point = aux.Point(const.POLARITY * -x_attack, -1500)
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
        print("robot_with_ball: ", self.robot_with_ball)

    def attack(self, field: field.Field, waypoints: list[wp.Waypoint]) -> None:
        """Атака"""
        goal_points = [aux.Point(field.enemy_goal.center.x, i) for i in range(-200, 201, 40)]
        for robo in self.popusk:
            pop_pos = field.allies[robo].role
            waypoints[robo] = wp.Waypoint(field.enemy_goal.popusk_positions[pop_pos], 0, wp.WType.S_ENDPOINT)
        # print(self.point_res)
        if self.robot_with_ball is not None:
            if self.attack_state == "TO_BALL":
                self.attack_pos = field.ball.get_pos()
                self.point_res = field.enemy_goal.center
                if aux.in_place(self.attack_pos, field.allies[self.robot_with_ball].get_pos(), 1000):
                    self.attack_state = "CALCULATING"

            elif self.attack_state == "CALCULATING":
                self.point_res = robot.shot_decision(field.ball.get_pos(), goal_points, field.enemies)
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
                        aux.Point(3000 * const.POLARITY, 2500 - 1000 * rC), field.allies[i].get_angle(), wp.WType.S_ENDPOINT
                    )
                    waypoints[i] = waypoint
                    rC += 1
            waypoint = wp.Waypoint(
                aux.Point(1700 * const.POLARITY, 0),
                aux.angle_to_point(aux.Point(1700 * const.POLARITY, 0), field.ball.get_pos()),
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
                    waypoint = wp.Waypoint(aux.Point(3000 * -const.POLARITY, 2500 - 1000 * rC), 0, wp.WType.S_ENDPOINT)
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
                waypoint = wp.Waypoint(aux.Point(2000 * const.POLARITY, 1250 - 500 * rC), 0, wp.WType.S_ENDPOINT)
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
            self.gk_go(field, waypoints, [const.GK], robot_with_ball)

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
                                aux.Point(700 * const.POLARITY, 0),
                                aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                                wp.WType.S_ENDPOINT,
                            )
                        else:
                            waypoint = wp.Waypoint(
                                aux.Point(700 * const.POLARITY, 2000 - 2000 * rC),
                                aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                                wp.WType.S_ENDPOINT,
                            )
                        waypoints[i] = waypoint
                    else:
                        waypoint = wp.Waypoint(
                            aux.Point(200 * const.POLARITY, 1500 - 3000 * (rC - 3)),
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
                            aux.Point(700 * const.POLARITY, 0),
                            aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                            wp.WType.S_ENDPOINT,
                        )
                    elif rC < 3:
                        waypoint = wp.Waypoint(
                            aux.Point(200 * const.POLARITY, 1000 - 2000 * (rC - 1)),
                            aux.angle_to_point(field.allies[i].get_pos(), aux.Point(0, 0)),
                            wp.WType.S_ENDPOINT,
                        )
                    else:
                        waypoint = wp.Waypoint(
                            aux.Point(200 * const.POLARITY, 2000 + 4000 * (rC - 4)),
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
            target = aux.point_on_line(field.ball.get_pos(), aux.Point(const.POLARITY * const.GOAL_DX, 0), 200)
            waypoint = wp.Waypoint(
                target, (field.ball.get_pos() - field.allies[go_kick.r_id].get_pos()).arg(), wp.WType.S_IGNOREOBSTACLES
            )
            waypoints[go_kick.r_id] = waypoint

        robot_with_ball = robot.find_nearest_robot(field.ball.get_pos(), field.enemies)
        self.gk_go(field, waypoints, [const.GK], robot_with_ball)

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
        self.gk_go(field, waypoints, [const.GK] + wall, robot_with_ball)
