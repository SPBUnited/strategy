"""Верхнеуровневый код стратегии"""
# pylint: disable=redefined-outer-name

# @package Strategy
# Расчет требуемых положений роботов исходя из ситуации на поле

import math

# !v DEBUG ONLY
from time import time
from typing import Optional

import bridge.router.waypoint as wp
from bridge import const
from bridge.auxiliary import aux, fld, rbt
from bridge.processors.referee_state_processor import Color as ActiveTeam
from bridge.processors.referee_state_processor import State as GameStates
from bridge.strategy import ref_states as refs


class DoingAction:
    """
    Класс действий некоторого робота, когда нужно зафиксировать положение
    """

    def __init__(self) -> None:
        self.id: Optional[int] = None
        self.angle: Optional[float] = None
        self.point: Optional[aux.Point] = None


class Strategy:
    """Основной класс с кодом стратегии"""

    def __init__(self, dbg_game_status: GameStates = GameStates.RUN) -> None:
        self.game_status = dbg_game_status
        self.active_team: ActiveTeam = ActiveTeam.ALL
        self.we_kick = False
        self.we_active = False
        self.ball_point = aux.Point(0, 0)
        self.doing_pass = DoingAction()
        self.doing_kick = DoingAction()
        self.global_st = 1
        self.new_st = 1
        self.time_st = time()

    def change_game_state(self, new_state: GameStates, upd_active_team: ActiveTeam) -> None:
        """Изменение состояния игры и цвета команды"""
        self.game_status = new_state
        self.active_team = upd_active_team

    def process(self, field: fld.Field) -> list[wp.Waypoint]:
        """
        Рассчитать конечные точки для каждого робота
        """
        if self.active_team == ActiveTeam.ALL or field.ally_color == self.active_team:
            self.we_active = True
        else:
            self.we_active = False

        waypoints: list[wp.Waypoint] = []
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            waypoints.append(wp.Waypoint(field.allies[i].get_pos(), field.allies[i].get_angle(), wp.WType.S_STOP))
        if self.game_status == GameStates.RUN or 1:
            self.run(field, waypoints)
        else:
            if self.game_status == GameStates.TIMEOUT:
                refs.timeout(field, waypoints)
            elif self.game_status == GameStates.HALT:
                pass
                # self.halt(field, waypoints)
            elif self.game_status == GameStates.PREPARE_PENALTY:
                refs.prepare_penalty(field, waypoints, self.we_active)
            # elif self.game_status == GameStates.PENALTY:
            #     if self.we_active:
            #         refs.penalty_kick(waypoints, field, self.we_active)
            #     else:
            #         self.goalkeeper(waypoints, field, True)
            elif self.game_status == GameStates.PREPARE_KICKOFF:
                self.prepare_kickoff(field, waypoints)
            elif self.game_status == GameStates.KICKOFF:
                self.kickoff(field, waypoints)
            elif self.game_status == GameStates.FREE_KICK:
                self.run(field, waypoints)
            elif self.game_status == GameStates.STOP:
                self.run(field, waypoints)

        return waypoints

    def run(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """
        Определение глобального состояния игры
        """
        self.return_ball_point(field)
        self.do_state(field)
        if self.global_st == 0:
            self.defense(waypoints, field)
        elif self.global_st == 1:
            self.attack(waypoints, field)
        print("STATE", self.global_st)
        # self.attack(waypoints, field)

    def debug(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """
        дебаг
        """

    def go_to_ball(self, main_p: aux.Point, field: fld.Field) -> list:
        """
        функция, определяющая оптимальный угол для удара по воротам соперников из данной точки
        """
        angle_point = aux.closest_point_on_line(field.enemy_goal.up, field.enemy_goal.down, main_p, is_inf="L")
        gates = [
            [
                aux.get_angle_between_points(angle_point, main_p, field.enemy_goal.down),
                aux.get_angle_between_points(angle_point, main_p, field.enemy_goal.up),
            ]
        ]
        # print("gates =", gates[0][0], gates[0][1])
        gates.sort()
        minuses = []
        for enemy in field.enemies:
            if enemy.is_used():
                tangents = aux.get_tangent_points(enemy.get_pos(), main_p, enemy.get_radius() + field.ball.get_radius())
                if tangents is not None:
                    if len(tangents) > 1:
                        minus = [
                            aux.get_angle_between_points(angle_point, main_p, tangents[0]),
                            aux.get_angle_between_points(angle_point, main_p, tangents[1]),
                        ]
                        minus.sort()
                        if abs(minus[0]) < math.pi / 2 or abs(minus[1]) < math.pi / 2:
                            minus = [
                                min(abs(minus[0]), math.pi / 2) * aux.sign(minus[0]),
                                min(abs(minus[1]), math.pi / 2) * aux.sign(minus[1]),
                            ]
                            minuses.append(minus)
        gates = aux.range_minus(gates, minuses)
        max_angle: list[float] = [1, -1]
        for ang in gates:
            if max_angle[1] - max_angle[0] < ang[1] - ang[0]:
                max_angle = ang
        if max_angle[1] - max_angle[0] < 0:
            cick_angle = aux.get_angle_between_points(angle_point, main_p, field.enemy_goal.center)
        else:
            cick_angle = (max_angle[1] + max_angle[0]) / 2
        cick_angle = aux.wind_down_angle(cick_angle + aux.angle_to_point(aux.Point(0, 0), field.enemy_goal.center))
        # if len(minuses) > 0:
        return [cick_angle, max(0, max_angle[1] - max_angle[0])]

    def do_state(self, field: fld.Field) -> None:
        """
        определяет состояние игры
        """
        nearest_enemy = fld.find_nearest_robots(field.ball.get_pos(), field.enemies, 1)
        if not nearest_enemy:
            self.global_st = 2
            return
        closest_dist = aux.dist(nearest_enemy[0].get_pos(), field.ball.get_pos())
        # print(closest_dist, 'ewwceviwebrlghwbeoi', self.new_st)
        if closest_dist < const.ROBOT_R + const.BALL_R * 2 and self.new_st != 0:
            self.new_st = 0
            self.time_st = time()
        elif closest_dist >= const.ROBOT_R + const.BALL_R * 2 and self.new_st != 1:
            self.new_st = 1
        if self.new_st == 0 and self.global_st != 0 and time() - self.time_st > 0.5:
            self.global_st = 0
        if self.new_st == 1 and self.global_st != 1 and not field.is_ball_moves():
            self.global_st = 1

    def defense(self, waypoints: list[wp.Waypoint], field: fld.Field) -> None:
        """
        функция режима защиты.
        Роботы подъезжают близко к мячу и вражеским роботам.
        """
        self.doing_kick.id = None
        self.doing_kick.angle = None
        self.doing_pass.id = None
        self.doing_pass.point = None
        now_enemies = fld.find_nearest_robots(field.ball.get_pos(), field.enemies)
        now_allies = fld.find_nearest_robots(
            field.ball.get_pos(), field.allies, None, [(const.GK, field.allies[const.GK].color)]
        )
        now_enemies.pop(0)
        ally_with_ball = now_allies[0]
        print("ГЛАВНЫЙ ЗАЩИТНИЧЕК", ally_with_ball.r_id, ally_with_ball.color)
        now_allies.pop(0)
        angle_point = aux.closest_point_on_line(field.ally_goal.up, field.ally_goal.down, field.ball.get_pos(), is_inf="L")
        gates = [
            [
                aux.get_angle_between_points(angle_point, field.ball.get_pos(), field.ally_goal.down),
                aux.get_angle_between_points(angle_point, field.ball.get_pos(), field.ally_goal.up),
            ]
        ]
        gates.sort()
        def_angle = (gates[0][0] + gates[0][1]) / 2
        def_angle = aux.wind_down_angle(def_angle + aux.angle_to_point(aux.Point(0, 0), field.ally_goal.center))
        help_point = aux.Point(
            field.ally_goal.center.x,
            field.ball.get_pos().y + (field.ally_goal.center.x - field.ball.get_pos().x) * math.tan(def_angle),
        )
        p_to_go = aux.average_point(
            [
                aux.closest_point_on_line(field.ball.get_pos(), help_point, ally_with_ball.get_pos(), is_inf="R"),
                aux.point_on_line(field.ball.get_pos(), help_point, field.ball.get_radius() + ally_with_ball.get_radius()),
            ]
        )
        waypoints[ally_with_ball.r_id] = wp.Waypoint(p_to_go, aux.wind_down_angle(def_angle - math.pi), wp.WType.S_BALL_KICK)
        if len(now_enemies) != 0:
            now_enemies = fld.find_nearest_robots(field.ball.get_pos(), now_enemies, len(now_enemies))
            for enemy in now_enemies:
                if len(now_allies) == 0:
                    break
                now_ally = fld.find_nearest_robot(enemy.get_pos(), now_allies)
                now_allies2 = []
                for ally in now_allies:
                    if ally.r_id != now_ally.r_id:
                        now_allies2.append(ally)
                now_allies = now_allies2
                p_to_go = aux.average_point(
                    [
                        aux.closest_point_on_line(enemy.get_pos(), field.ball.get_pos(), now_ally.get_pos(), is_inf="R"),
                        aux.point_on_line(enemy.get_pos(), field.ball.get_pos(), enemy.get_radius() + now_ally.get_radius()),
                    ]
                )
                waypoints[now_ally.r_id] = wp.Waypoint(
                    p_to_go, aux.angle_to_point(field.ball.get_pos(), enemy.get_pos()), wp.WType.S_ENDPOINT
                )
                # print(enemy.get_pos().x, enemy.get_pos().y, now_ally.get_pos().x, now_ally.get_pos().y)
        self.goalkeeper(waypoints, field)

    def return_ball_point(self, field: fld.Field) -> None:
        """
        функция, обновляющая положение мяча для определения его трактории. Вызывать с большой частотой
        """
        if not field.is_ball_moves():
            self.ball_point = field.ball.get_pos()

    def goalkeeper(self, waypoints: list[wp.Waypoint], field: fld.Field, penalty: bool = False) -> None:
        """
        функция вратаря с несколькими режимами поведения
        """
        p1, p2 = None, None
        if field.is_ball_moves_to_goal():
            p1 = self.ball_point
            p2 = field.ball.get_pos()
        elif (self.new_st == 0) or penalty:
            enemy_with_ball = fld.find_nearest_robots(field.ball.get_pos(), field.enemies, 1)[0]
            p1 = enemy_with_ball.get_pos()
            def_angle = enemy_with_ball.get_angle()
            help_point = aux.Point(
                field.ally_goal.center.x,
                field.ball.get_pos().y + (field.ally_goal.center.x - field.ball.get_pos().x) * math.tan(def_angle),
            )
            if (
                aux.dist(
                    aux.average_point(aux.closest_point_on_poly(p1, help_point, field.ally_goal.small_hull)),
                    aux.average_point(aux.closest_point_on_poly(p1, field.ball.get_pos(), field.ally_goal.small_hull)),
                )
                < const.ROBOT_R
            ):
                p2 = help_point
            # print(def_angle)
        if p1 is None or p2 is None:
            angle_point = aux.closest_point_on_line(
                field.ally_goal.up, field.ally_goal.down, field.ball.get_pos(), is_inf="L"
            )
            gates = [
                [
                    aux.get_angle_between_points(angle_point, field.ball.get_pos(), field.ally_goal.down),
                    aux.get_angle_between_points(angle_point, field.ball.get_pos(), field.ally_goal.up),
                ]
            ]
            gates.sort()
            def_angle = (gates[0][0] + gates[0][1]) / 2
            def_angle = aux.wind_down_angle(def_angle + aux.angle_to_point(aux.Point(0, 0), field.ally_goal.center))
            help_point = aux.Point(
                field.ally_goal.center.x,
                field.ball.get_pos().y + (field.ally_goal.center.x - field.ball.get_pos().x) * math.tan(def_angle),
            )
            p1 = field.ball.get_pos()
            p2 = help_point
        p_to_go = aux.average_point(aux.closest_point_on_poly(p1, p2, field.ally_goal.small_hull))
        waypoints[const.GK] = wp.Waypoint(p_to_go, aux.angle_to_point(p_to_go, field.ball.get_pos()), wp.WType.S_ENDPOINT)

    def attack(self, waypoints: list[wp.Waypoint], field: fld.Field) -> None:
        """
        фунцкия атаки.
        """
        if field.is_ball_moves():
            self.doing_kick.id = None
            self.doing_kick.angle = None
        elif self.doing_kick.id is None:
            self.doing_pass.id = None
            self.doing_pass.point = None
        if self.doing_kick.id is not None:
            if aux.dist(field.ball.get_pos(), field.allies[self.doing_kick.id].get_pos()) > const.ROBOT_R + const.BALL_R * 3:
                self.doing_kick.id = None
                self.doing_kick.angle = None
        if self.doing_pass.id is not None:
            print("пасики", self.doing_pass.id, self.doing_pass.point)
        else:
            print("нет пасиков")
        if self.doing_kick.id is not None:
            print("кики", self.doing_kick.id, self.doing_kick.angle)
        else:
            print("нет киков")
        print("moving ball", field.is_ball_moves())
        if (
            aux.is_point_inside_poly(field.ball.get_pos(), field.ally_goal.hull)
            or aux.dist(aux.nearest_point_on_poly(field.ball.get_pos(), field.ally_goal.hull), field.ball.get_pos())
            < const.ROBOT_R * 2 + const.BALL_R
        ):
            active_gk = 1
        else:
            active_gk = 0
        now_allies: list[rbt.Robot] = []
        if active_gk:
            now_allies.append(field.allies[const.GK])
        now_allies2 = []
        for ally in field.allies:
            if ally.r_id != const.GK and ally.is_used():
                now_allies2.append(ally)
        if len(now_allies2) != 0:
            now_allies2 = fld.find_nearest_robots(field.ball.get_pos(), now_allies2, len(now_allies2))
        now_allies.extend(now_allies2)
        if len(now_allies) == 0:
            self.goalkeeper(waypoints, field)
            return None
        ally_with_ball = now_allies[0]
        now_allies.pop(0)
        point_allies: list[aux.Point] = []
        enemy_angles = []
        gate_angles = []
        angle_point = aux.closest_point_on_line(field.enemy_goal.up, field.enemy_goal.down, field.ball.get_pos(), is_inf="L")
        for enemy in field.enemies:
            if enemy.is_used() and (
                enemy.r_id != const.ENEMY_GK or not aux.is_point_inside_poly(enemy.get_pos(), field.enemy_goal.hull)
            ):
                tangents = aux.get_tangent_points(enemy.get_pos(), field.ball.get_pos(), const.ROBOT_R * 2)
                if tangents is not None:
                    if len(tangents) > 1:
                        enemy_angle = [
                            aux.get_angle_between_points(angle_point, field.ball.get_pos(), tangents[0]),
                            aux.get_angle_between_points(angle_point, field.ball.get_pos(), tangents[1]),
                        ]
                        enemy_angle.sort()
                        if abs(enemy_angle[0]) < math.pi / 2 or abs(enemy_angle[1]) < math.pi / 2:
                            enemy_angle = [
                                min(abs(enemy_angle[0]), math.pi / 2) * aux.sign(enemy_angle[0]),
                                min(abs(enemy_angle[1]), math.pi / 2) * aux.sign(enemy_angle[1]),
                            ]
                            enemy_angles.append(enemy_angle)
        for now_ally in now_allies:
            best_point = aux.Point(
                (field.enemy_goal.center.x - field.ball.get_pos().x) * (len(point_allies) + 1) / (len(now_allies) + 1)
                + field.ball.get_pos().x,
                const.HALF_HEIGHT * 2 / 3 * aux.sign(field.ball.get_pos().y) * (((len(point_allies) % 2) * 2) - 1),
            )
            best_angle = aux.get_angle_between_points(angle_point, field.ball.get_pos(), best_point)
            # print(best_point)
            enemy_angles2 = []
            min_tan = aux.get_tangent_points(best_point, field.ball.get_pos(), const.ROBOT_R + const.BALL_R)
            if min_tan is not None:
                if len(min_tan) > 1:
                    min_en = abs(aux.get_angle_between_points(min_tan[0], field.ball.get_pos(), min_tan[1]))
            for ang in enemy_angles:
                if ang[1] - ang[0] > min_en:
                    enemy_angles2.append([ang[0] - 0.3, ang[1] + 0.3])
            enemy_angles2 = aux.range_plus(enemy_angles2)
            flag_best = True
            for ang in enemy_angles2:
                if aux.is_in_range(best_angle, ang):
                    flag_best = False
                    if (abs(best_angle - ang[0]) < abs(best_angle - ang[1]) and ang[0] > -math.pi / 2) or ang[
                        1
                    ] > math.pi / 2:
                        best_angle = ang[0]
                    else:
                        best_angle = ang[1]
            best_angle = aux.wind_down_angle(best_angle + aux.angle_to_point(aux.Point(0, 0), field.enemy_goal.center))
            if not flag_best:
                best_point = aux.Point(
                    field.ball.get_pos().x + math.cos(best_angle) * aux.dist(best_point, field.ball.get_pos()),
                    field.ball.get_pos().y + math.sin(best_angle) * aux.dist(best_point, field.ball.get_pos()),
                )
            point_allies.append(best_point)
        # print(len(point_allies))
        for point_ally in point_allies:
            print(point_ally, "best")
            now_ally = fld.find_nearest_robots(point_ally, now_allies, 1)[0]
            if (
                aux.dist(
                    now_ally.get_pos(),
                    aux.closest_point_on_line(field.ball.get_pos(), point_ally, now_ally.get_pos(), is_inf="L"),
                )
                < const.ROBOT_R + const.BALL_R
            ):
                gate_angles.append(self.go_to_ball(point_ally, field))
                gate_angles[-1].append(now_ally.r_id)
                gate_angles[-1].append(point_ally)
            now_allies2 = []
            for ally in now_allies:
                if ally.r_id != now_ally.r_id:
                    now_allies2.append(ally)
            now_allies = now_allies2
            p_to_go = None
            if self.doing_pass.id == now_ally.r_id and self.doing_pass.point is not None:
                if field.is_ball_moves():
                    p_to_go = aux.closest_point_on_line(
                        self.ball_point, field.ball.get_pos(), self.doing_pass.point, is_inf="L"
                    )
                    p_to_go = aux.point_on_line(
                        self.ball_point, p_to_go, aux.dist(self.ball_point, p_to_go) + const.ROBOT_R * 3
                    )
                else:
                    p_to_go = self.doing_pass.point
                print("doing pass", now_ally.r_id)
            if p_to_go is None:
                p_to_go = point_ally
            waypoints[now_ally.r_id] = wp.Waypoint(
                p_to_go, aux.angle_to_point(p_to_go, field.ball.get_pos()), wp.WType.S_ENDPOINT
            )
            print(waypoints[now_ally.r_id].pos, now_ally.r_id)
        if not active_gk:
            self.goalkeeper(waypoints, field)
        p_to_go = None
        angle_to_go = None
        if self.doing_pass.id == ally_with_ball.r_id and self.doing_pass.point is not None:
            if field.is_ball_moves():
                p_to_go = aux.closest_point_on_line(self.ball_point, field.ball.get_pos(), self.doing_pass.point, is_inf="L")
                p_to_go = aux.point_on_line(self.ball_point, p_to_go, aux.dist(self.ball_point, p_to_go) + const.ROBOT_R * 3)
            else:
                p_to_go = self.doing_pass.point
            angle_to_go = aux.angle_to_point(p_to_go, field.ball.get_pos())
            waypoints[ally_with_ball.r_id] = wp.Waypoint(p_to_go, angle_to_go, wp.WType.S_ENDPOINT)
        if angle_to_go is None:
            p_to_go = field.ball.get_pos()
            if self.doing_kick.id == ally_with_ball.r_id and self.doing_kick.angle is not None:
                angle_to_go = self.doing_kick.angle
                waypoints[ally_with_ball.r_id] = wp.Waypoint(p_to_go, angle_to_go, wp.WType.S_BALL_KICK)
                # print(waypoints[ally_with_ball.r_id].pos,waypoints[ally_with_ball.r_id].angle, ally_with_ball.r_id, "ball")
            if angle_to_go is None:
                # print(ally_with_ball.r_id)
                gate_angles.insert(0, self.go_to_ball(field.ball.get_pos(), field))
                gate_angles[0].append(ally_with_ball.r_id)
                gate_angles[0].append(field.ball.get_pos())
                best_gate_idx = None
                for idx, gate in enumerate(gate_angles):
                    if best_gate_idx is not None:
                        if gate[1] > gate_angles[best_gate_idx][1]:
                            best_gate_idx = idx
                    else:
                        best_gate_idx = idx
                if best_gate_idx is not None and ally_with_ball is not None:
                    print("best gate", gate_angles[best_gate_idx][1])
                    if (
                        gate_angles[best_gate_idx][2] == ally_with_ball.r_id
                        or abs(field.enemy_goal.center.x - field.ball.get_pos().x) < const.GOAL_DX / 3
                        or gate_angles[0][1] > 0.25
                    ):
                        best_gate_idx = 0
                        print(gate_angles[best_gate_idx][1])
                        angle_to_go = gate_angles[best_gate_idx][0]
                    else:
                        angle_to_go = aux.angle_to_point(field.ball.get_pos(), gate_angles[best_gate_idx][3])
                        if (
                            aux.dist(p_to_go, ally_with_ball.get_pos()) < const.ROBOT_R + const.BALL_R * 2
                            and self.doing_pass.id is None
                            and not field.is_ball_moves()
                        ):
                            self.doing_pass.id = gate_angles[best_gate_idx][2]
                            self.doing_pass.point = gate_angles[best_gate_idx][3]
                            print("Я СДЕЛАЛ ПАС")
                    if (
                        aux.dist(p_to_go, ally_with_ball.get_pos()) < const.ROBOT_R + const.BALL_R * 2
                        and self.doing_kick.id is None
                        and not field.is_ball_moves()
                    ):
                        self.doing_kick.id = ally_with_ball.r_id
                        self.doing_kick.angle = angle_to_go
                        print("Я СДЕЛАЛ КИК")
                    waypoints[ally_with_ball.r_id] = wp.Waypoint(
                        p_to_go, aux.angle_to_point(p_to_go, field.enemies[const.ENEMY_GK].get_pos()), wp.WType.S_BALL_KICK
                    )
                    print(gate_angles[best_gate_idx][2], "лучший")
                    print(
                        "МЯЧ ДВИГАЕТСЯ??????",
                        aux.dist(p_to_go, ally_with_ball.get_pos()) < const.ROBOT_R + const.BALL_R * 3
                        and self.doing_kick.id is None
                        and not field.is_ball_moves(),
                    )
                    print(
                        waypoints[ally_with_ball.r_id].pos, waypoints[ally_with_ball.r_id].angle, ally_with_ball.r_id, "ball"
                    )

    def prepare_kickoff(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Настройка перед состоянием kickoff по команде судей"""
        if self.we_active:
            self.we_kick = True
        else:
            self.we_kick = False
        self.put_kickoff_waypoints(field, waypoints)

    def put_kickoff_waypoints(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Подготовка перед состоянием kickoff"""
        rC = 0
        if self.we_kick:
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if field.allies[i].is_used() and field.allies[i].r_id != field.gk_id:
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
                if field.allies[i].is_used() and field.allies[i].r_id != field.gk_id:
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
        waypoints[field.allies[field.gk_id].r_id] = waypoint

    def kickoff(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Удар мяча из аута"""
        self.put_kickoff_waypoints(field, waypoints)
        # self.we_kick = 0
        if self.we_kick:
            go_kick = fld.find_nearest_robot(field.ball.get_pos(), field.allies)
            target = field.enemy_goal.center
            target.y = 300
            waypoint = wp.Waypoint(
                field.ball.get_pos(), (target - field.allies[go_kick.r_id].get_pos()).arg(), wp.WType.S_BALL_KICK
            )
            waypoints[go_kick.r_id] = waypoint
        else:
            go_kick = fld.find_nearest_robot(field.ball.get_pos(), field.allies)
            target = aux.point_on_line(field.ball.get_pos(), aux.Point(field.polarity * const.GOAL_DX, 0), 200)
            waypoint = wp.Waypoint(
                target, (field.ball.get_pos() - field.allies[go_kick.r_id].get_pos()).arg(), wp.WType.S_IGNOREOBSTACLES
            )
            waypoints[go_kick.r_id] = waypoint
