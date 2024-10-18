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
        self.pos_count = 0
        self.pos_ball: list[aux.Point] = []
        self.pos_n = 5
        for _ in range(self.pos_n):
            self.pos_ball.append(aux.Point(0, 0))

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
        if self.game_status == GameStates.RUN:
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
        # self.goalkeeper(waypoints, field)

    def debug(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """
        дебаг
        """

    def go_to_ball(self, main_p: aux.Point, field: fld.Field) -> list:
        """
        функция, определяющая оптимальный угол для удара по воротам соперников из данной точки
        """
        angle_point = aux.closest_point_on_line(field.enemy_goal.up, field.enemy_goal.down, main_p, is_inf="L")
        gates: list[list[float]] = sorted(
            [
                [
                    aux.get_angle_between_points(angle_point, main_p, field.enemy_goal.down),
                    aux.get_angle_between_points(angle_point, main_p, field.enemy_goal.up),
                ]
            ]
        )
        minuses = []
        active_enemies = fld.get_active_bots(field.enemies)
        for enemy in active_enemies:
            tangents = aux.get_tangent_points(enemy.get_pos(), main_p, enemy.get_radius() + field.ball.get_radius())
            if len(tangents) > 1:
                minus: list[float] = sorted(
                    [
                        aux.get_angle_between_points(angle_point, main_p, tangents[0]),
                        aux.get_angle_between_points(angle_point, main_p, tangents[1]),
                    ]
                )
                if minus[1] - minus[0] < math.pi:
                    minuses.append(minus)

        gates = sorted(aux.range_minus(gates, minuses), key=lambda a: a[1] - a[0], reverse=True)
        if not gates:
            cick_angle = aux.get_angle_between_points(angle_point, main_p, field.enemy_goal.center)
            gates = [[0, 0]]
        else:
            cick_angle = (gates[0][1] + gates[0][0]) / 2
        cick_angle = aux.wind_down_angle(cick_angle + aux.angle_to_point(aux.Point(0, 0), field.enemy_goal.center))
        return [cick_angle, gates[0][1] - gates[0][0]]

    def do_state(self, field: fld.Field) -> None:
        """
        определяет состояние игры
        """
        nearest_enemy = fld.find_nearest_robots(field.ball.get_pos(), field.enemies, 1)
        nearest_ally = fld.find_nearest_robots(field.ball.get_pos(), field.allies, 1, [field.allies[const.GK]])
        if nearest_enemy:
            closest_dist = aux.dist(nearest_enemy[0].get_pos(), field.ball.get_pos())
        else:
            self.new_st = 1
            self.global_st = 1
            return
        if nearest_ally:
            closest_dist_ally = aux.dist(nearest_ally[0].get_pos(), field.ball.get_pos())
        else:
            self.new_st = 0
            self.global_st = 0
            return
        if (closest_dist < const.ROBOT_R + const.BALL_R * 2 or closest_dist < closest_dist_ally) and self.new_st != 0:
            self.new_st = 0
            self.time_st = time()
        elif closest_dist >= const.ROBOT_R + const.BALL_R * 2 and closest_dist > closest_dist_ally and self.new_st != 1:
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
        self.goalkeeper(waypoints, field)
        now_enemies = fld.find_nearest_robots(field.ball.get_pos(), field.enemies)
        now_allies = fld.find_nearest_robots(field.ball.get_pos(), field.allies, None, [field.allies[const.GK]])
        if not now_enemies:
            return
        now_enemies.pop(0)
        if not now_allies:
            return
        ally_with_ball = now_allies[0]
        now_allies.pop(0)
        help_p = (
            aux.rotate(
                (field.ally_goal.down - field.ball.get_pos()),
                aux.get_angle_between_points(field.ally_goal.down, field.ball.get_pos(), field.ally_goal.up) / 2,
            )
            + field.ball.get_pos()
        )
        p_to_go = aux.average_point(
            [
                aux.closest_point_on_line(field.ball.get_pos(), help_p, ally_with_ball.get_pos(), is_inf="R"),
                field.ball.get_pos(),
            ]
        )
        def_ang = (
            aux.average_angle(
                [(field.ally_goal.up - field.ball.get_pos()).arg(), (field.ally_goal.down - field.ball.get_pos()).arg()]
            )
            - math.pi
        )
        waypoints[ally_with_ball.r_id] = wp.Waypoint(p_to_go, def_ang, wp.WType.S_BALL_KICK)
        now_enemies = fld.find_nearest_robots(field.ally_goal.center, now_enemies)
        for enemy in now_enemies:
            if len(now_allies) == 0:
                break
            now_ally = fld.find_nearest_robot(enemy.get_pos(), now_allies)
            now_allies = fld.exclude(now_allies, [now_ally])
            p_to_go = aux.average_point(
                [
                    aux.closest_point_on_line(enemy.get_pos(), field.ball.get_pos(), now_ally.get_pos(), is_inf="R"),
                    aux.point_on_line(enemy.get_pos(), field.ball.get_pos(), enemy.get_radius() + now_ally.get_radius()),
                ]
            )
            waypoints[now_ally.r_id] = wp.Waypoint(
                p_to_go, aux.angle_to_point(field.ball.get_pos(), enemy.get_pos()), wp.WType.S_ENDPOINT
            )

    def return_ball_point(self, field: fld.Field) -> None:
        """
        функция, обновляющая положение мяча для определения его трактории. Вызывать с большой частотой
        """
        self.pos_ball[self.pos_count] = field.ball.get_pos()
        self.pos_count = (self.pos_count + 1) % self.pos_n
        self.ball_point = self.pos_ball[self.pos_count]
        field.image.draw_dot(self.ball_point, (0, 255, 0), 30)

    def goalkeeper(self, waypoints: list[wp.Waypoint], field: fld.Field) -> None:
        """
        функция вратаря с несколькими режимами поведения
        """
        p_ball = None
        if field.is_ball_moves_to_goal():
            p_ball = self.ball_point
        elif self.new_st == 0:
            sorted_enemies = fld.find_nearest_robots(field.ball.get_pos(), field.enemies)
            if sorted_enemies:
                enemy_with_ball = sorted_enemies[0]
                if aux.dist(enemy_with_ball.get_pos(), field.ball.get_pos()) < const.ROBOT_R * 2 + const.BALL_R:
                    help_p = aux.rotate(aux.Point(1, 0), enemy_with_ball.get_angle()) + field.ball.get_pos()
                    if (
                        abs(aux.get_angle_between_points(help_p, enemy_with_ball.get_pos(), field.ball.get_pos()))
                        < const.TRUE_ANGLE
                    ):
                        p_ball = help_p
                    else:
                        p_ball = enemy_with_ball.get_pos()
                    if not aux.get_line_intersection(
                        field.ball.get_pos(), p_ball, field.ally_goal.down, field.ally_goal.up, "LS"
                    ):
                        p_ball = None
        if p_ball is None:
            p_ball = (
                aux.rotate(
                    (field.ally_goal.down - field.ball.get_pos()),
                    aux.get_angle_between_points(field.ally_goal.down, field.ball.get_pos(), field.ally_goal.up) / 2,
                )
                + field.ball.get_pos()
            )
        p_to_go = aux.average_point(aux.closest_point_on_poly(field.ball.get_pos(), p_ball, field.ally_goal.small_hull))
        field.image.draw_line(field.ball.get_pos(), p_ball)
        waypoints[const.GK] = wp.Waypoint(
            p_to_go, aux.angle_to_point(field.ally_goal.center, aux.Point(0, 0)), wp.WType.S_IGNOREOBSTACLES
        )

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
        if (
            aux.is_point_inside_poly(field.ball.get_pos(), field.ally_goal.hull)
            or aux.dist(aux.nearest_point_on_poly(field.ball.get_pos(), field.ally_goal.hull), field.ball.get_pos())
            < const.ROBOT_R * 2 + const.BALL_R
        ):
            active_gk = 1
        else:
            active_gk = 0
        now_allies: list[rbt.Robot] = fld.find_nearest_robots(
            field.ball.get_pos(), field.allies, None, [field.allies[const.GK]]
        )
        if active_gk:
            now_allies.insert(0, field.allies[const.GK])
        if len(now_allies) == 0:
            self.goalkeeper(waypoints, field)
            return None
        ally_with_ball = now_allies[0]
        now_allies.pop(0)
        point_allies: list[aux.Point] = []
        enemy_poses: list[tuple[aux.Point, list[float]]] = []
        gate_angles = []
        angle_point = aux.closest_point_on_line(field.enemy_goal.up, field.enemy_goal.down, field.ball.get_pos(), is_inf="L")
        active_enemies = fld.get_active_bots(field.enemies)
        ally_poses: list[tuple[int, aux.Point, float]] = []
        for enemy in active_enemies:
            tangents = aux.get_tangent_points(enemy.get_pos(), field.ball.get_pos(), const.ROBOT_R * 2)
            if len(tangents) > 1:
                enemy_angle: list[float] = sorted(
                    [
                        aux.get_angle_between_points(angle_point, field.ball.get_pos(), tangents[0]),
                        aux.get_angle_between_points(angle_point, field.ball.get_pos(), tangents[1]),
                    ]
                )
                if enemy_angle[1] - enemy_angle[0] < math.pi:
                    enemy_poses.append(
                        (enemy.get_pos(), [enemy_angle[0] - const.DELTA_ANGLE / 2, enemy_angle[1] + const.DELTA_ANGLE / 2])
                    )
        for now_ally in now_allies:
            best_point = aux.Point(
                (field.enemy_goal.center.x - field.ball.get_pos().x) * (len(point_allies) + 1) / (len(now_allies) + 1)
                + field.ball.get_pos().x,
                const.HALF_HEIGHT * 7 / 12 * aux.sign(field.ball.get_pos().y) * (((len(point_allies) % 2) * 2) - 1),
            )
            best_angle = aux.get_angle_between_points(angle_point, field.ball.get_pos(), best_point)
            min_tan = aux.get_tangent_points(best_point, field.ball.get_pos(), const.ROBOT_R * 2)
            enemy_angles_local = []
            if len(min_tan) > 1:
                min_en = abs(aux.get_angle_between_points(min_tan[0], field.ball.get_pos(), min_tan[1]))
            for pos in enemy_poses:
                if pos[1][1] - pos[1][0] - const.DELTA_ANGLE > min_en and aux.dist(pos[0], best_point) > const.ROBOT_R * 2:
                    enemy_angles_local.append(pos[1])
            enemy_angles_local = aux.range_plus(enemy_angles_local)
            for ang in enemy_angles_local:
                if aux.is_in_range(best_angle, ang):
                    if abs(best_angle - ang[0]) < abs(best_angle - ang[1]) and ang[0] > -math.pi / 2:
                        best_angle = ang[0]
                    else:
                        best_angle = ang[1]
            best_angle = aux.wind_down_angle(best_angle + aux.angle_to_point(aux.Point(0, 0), field.enemy_goal.center))
            best_point = (
                aux.rotate(best_point - field.ball.get_pos(), best_angle - (best_point - field.ball.get_pos()).arg())
                + field.ball.get_pos()
            )
            point_allies.append(best_point)
        for point_ally in point_allies:
            now_ally = fld.find_nearest_robots(point_ally, now_allies, 1)[0]
            if (
                aux.dist(
                    now_ally.get_pos(),
                    aux.closest_point_on_line(field.ball.get_pos(), point_ally, now_ally.get_pos(), is_inf="R"),
                )
                < const.ROBOT_R + const.BALL_R
            ):
                gate_angles.append(self.go_to_ball(point_ally, field))
                gate_angles[-1].extend([now_ally.r_id, point_ally])
            now_allies = fld.exclude(now_allies, [now_ally])
            ally_poses.append((now_ally.r_id, point_ally, aux.angle_to_point(point_ally, field.ball.get_pos())))
        if not active_gk:
            self.goalkeeper(waypoints, field)
        gate_angles = sorted(gate_angles, key=lambda x: x[1])
        gate_angles.insert(0, self.go_to_ball(field.ball.get_pos(), field))
        gate_angles[0].extend([ally_with_ball.r_id, field.ball.get_pos()])
        if (
            gate_angles[0][1] >= gate_angles[min(1, len(gate_angles) - 1)][1]
            or abs(field.enemy_goal.center.x - field.ball.get_pos().x) < const.GOAL_DX / 3
            or gate_angles[0][1] >= const.DELTA_ANGLE
        ):
            angle_to_go = float(gate_angles[0][0])
        else:
            angle_to_go = aux.angle_to_point(field.ball.get_pos(), gate_angles[1][3])
            if (
                aux.dist(field.ball.get_pos(), ally_with_ball.get_pos()) < const.ACTION_DIST
                and self.doing_pass.id is None
                and not field.is_ball_moves()
            ):
                self.doing_pass.id = gate_angles[1][2]
                self.doing_pass.point = gate_angles[1][3]
        if (
            aux.dist(field.ball.get_pos(), ally_with_ball.get_pos()) < const.ACTION_DIST
            and self.doing_kick.id is None
            and not field.is_ball_moves()
        ):
            self.doing_kick.id = ally_with_ball.r_id
            self.doing_kick.angle = angle_to_go
        p_to_go = aux.Point(
            field.ball.get_pos().x - math.cos(angle_to_go) * (const.ROBOT_R + const.BALL_R),
            field.ball.get_pos().y - math.sin(angle_to_go) * (const.ROBOT_R + const.BALL_R),
        )
        # if aux.dist(field.ball.get_pos(), ally_with_ball.get_pos()) < const.LIE_DIST:
        #     angle_to_go += math.pi / 6
        ally_poses.append((ally_with_ball.r_id, p_to_go, angle_to_go))
        for ally_pos in ally_poses:
            if ally_pos[0] == self.doing_pass.id and self.doing_pass.point is not None:
                if field.is_ball_moves():
                    p_to_go = aux.closest_point_on_line(
                        self.ball_point, field.ball.get_pos(), field.allies[ally_pos[0]].get_pos(), is_inf="L"
                    )
                    p_to_go = aux.point_on_line(
                        field.ball.get_pos(),
                        p_to_go,
                        max(
                            field.ball.get_vel().mag() * const.SUMM_DELAY / 1000 + const.ACTION_DIST,
                            aux.dist(field.ball.get_pos(), field.allies[ally_pos[0]].get_pos()),
                        ),
                    )
                    waypoints[ally_pos[0]] = wp.Waypoint(
                        p_to_go, aux.angle_to_point(p_to_go, self.ball_point), wp.WType.S_IGNOREOBSTACLES
                    )
                else:
                    waypoints[ally_pos[0]] = wp.Waypoint(
                        self.doing_pass.point, aux.angle_to_point(self.doing_pass.point, self.ball_point), wp.WType.S_ENDPOINT
                    )
            elif ally_pos[0] == self.doing_kick.id and self.doing_kick.angle is not None:
                waypoints[ally_pos[0]] = wp.Waypoint(field.ball.get_pos(), self.doing_kick.angle, wp.WType.S_BALL_KICK)
            else:
                waypoints[ally_pos[0]] = wp.Waypoint(ally_pos[1], ally_pos[2], wp.WType.S_ENDPOINT)

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

    def attack_points(self, waypoints: list[wp.Waypoint], field: fld.Field) -> None:
        """
        высчитывает точки, на которые выстраивает защитников
        """
