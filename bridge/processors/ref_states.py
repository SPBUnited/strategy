import math

import bridge.processors.auxiliary as aux
import bridge.processors.const as const
import bridge.processors.field as fld
import bridge.processors.waypoint as wp


class RefStates:
    def __init__(self) -> None:
        self.we_active = 1

        # PENALTY
        self.we_kick = 0
        self.is_started = 0
        self.penalty_timer = 0
        self.doing_penalty = 0

    def prepare_penalty(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
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
                    and field.allies[i].r_id != field.gk_id
                ):
                    waypoint = wp.Waypoint(
                        aux.Point(1500 * field.polarity, 250 - 500 * rC), field.allies[i].get_angle(), wp.WType.S_ENDPOINT
                    )
                    waypoints[i] = waypoint
                    rC += 1
            waypoint = wp.Waypoint(
                aux.Point(550 * field.polarity, 0),
                aux.angle_to_point(aux.Point(550 * field.polarity, 0), field.ball.get_pos()),
                wp.WType.S_ENDPOINT,
            )
            waypoints[field.allies[const.PENALTY_KICKER].r_id] = waypoint

            waypoint = wp.Waypoint(field.ally_goal.center, 0, wp.WType.S_ENDPOINT)
            waypoints[field.allies[field.gk_id].r_id] = waypoint
        else:
            rC = 0
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if field.allies[i].is_used() and field.allies[i].r_id != field.gk_id:
                    waypoint = wp.Waypoint(aux.Point(1500 * -field.polarity, 250 - 500 * rC), 0, wp.WType.S_ENDPOINT)
                    waypoints[i] = waypoint
                    rC += 1
            waypoint = wp.Waypoint(field.ally_goal.center, 0, wp.WType.S_ENDPOINT)
            waypoints[field.allies[field.gk_id].r_id] = waypoint

    def halt(
        self, field: fld.Field, waypoints: list[wp.Waypoint]
    ) -> None:  # TODO: проверить что роботы останавливаются на самом деле
        """Пауза по команде от судей"""
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used():
                waypoint = wp.Waypoint(field.allies[i].get_pos(), field.allies[i].get_angle(), wp.WType.S_ENDPOINT)
                waypoints[i] = waypoint

    def timeout(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Таймаут по команде от судей"""
        rC = 0
        for i in range(const.TEAM_ROBOTS_MAX_COUNT):
            if field.allies[i].is_used():
                waypoint = wp.Waypoint(aux.Point(1000 * field.polarity, 1250 - 500 * rC), 0, wp.WType.S_ENDPOINT)
                waypoints[i] = waypoint
                rC += 1

    def penalty_kick(self, waypoints: list[wp.Waypoint], field: fld.Field) -> None:
        """Пенальти (атака)"""
        small_ball = const.BALL_R / 2
        if (
            field.is_ball_moves()
            or aux.dist(field.allies[const.PENALTY_KICKER].get_pos(), field.ball.get_pos()) > const.ROBOT_R * 3
        ):
            self.doing_penalty = 0
        angle_point = aux.closest_point_on_line(field.enemy_goal.up, field.enemy_goal.down, field.ball.get_pos(), is_inf="L")
        gates = [
            [
                aux.get_angle_between_points(angle_point, field.ball.get_pos(), field.enemy_goal.down),
                aux.get_angle_between_points(angle_point, field.ball.get_pos(), field.enemy_goal.up),
            ]
        ]
        # print("gates =", gates[0][0], gates[0][1])
        gates.sort()
        minuses = []
        for enemy in field.enemies:
            if enemy.is_used():
                try:
                    tangents = aux.get_tangent_points(
                        enemy.get_pos(), field.ball.get_pos(), enemy.get_radius() + field.ball.get_radius()
                    )
                    minus = [
                        aux.get_angle_between_points(angle_point, field.ball.get_pos(), tangents[0]),
                        aux.get_angle_between_points(angle_point, field.ball.get_pos(), tangents[1]),
                    ]
                    minus.sort()
                    if abs(minus[0]) < math.pi / 2 or abs(minus[1]) < math.pi / 2:
                        minus = [
                            min(abs(minus[0]), math.pi / 2) * aux.sign(minus[0]),
                            min(abs(minus[1]), math.pi / 2) * aux.sign(minus[1]),
                        ]
                        minuses.append(minus)
                except:
                    pass
        gates = aux.range_minus(gates, minuses)
        max_angle = [1, -1]
        for ang in gates:
            if max_angle[1] - max_angle[0] < ang[1] - ang[0]:
                max_angle = ang
        if max_angle[1] - max_angle[0] < 0:
            cick_angle = aux.get_angle_between_points(angle_point, field.ball.get_pos(), field.enemy_goal.center)
        else:
            cick_angle = (max_angle[1] + max_angle[0]) / 2
        way_angle = (math.pi / 3 - abs(cick_angle)) * aux.sign(cick_angle)
        cick_angle = aux.wind_down_angle(cick_angle + aux.angle_to_point(aux.Point(0, 0), field.enemy_goal.center))
        way_angle = aux.wind_down_angle(way_angle + aux.angle_to_point(aux.Point(0, 0), field.enemy_goal.center))
        if not self.doing_penalty:
            minus_angle = aux.wind_down_angle(cick_angle + math.pi)
            help_point = aux.Point(
                field.ball.get_pos().x - aux.sign(field.enemy_goal.center.x),
                field.ball.get_pos().y - aux.sign(field.enemy_goal.center.x) * math.tan(minus_angle),
            )
            print(
                field.ball.get_pos(),
                help_point,
                aux.point_on_line(field.ball.get_pos(), help_point, const.ROBOT_R + small_ball),
            )
            if aux.dist(field.allies[const.PENALTY_KICKER].get_pos(), field.ball.get_pos()) < const.ROBOT_R + small_ball * 2:
                self.doing_penalty = 1
            waypoints[const.PENALTY_KICKER] = wp.Waypoint(
                aux.point_on_line(field.ball.get_pos(), help_point, const.ROBOT_R + small_ball),
                way_angle,
                wp.WType.S_ENDPOINT,
            )
        else:
            waypoints[const.PENALTY_KICKER] = wp.Waypoint(field.ball.get_pos(), cick_angle, wp.WType.S_BALL_KICK)
