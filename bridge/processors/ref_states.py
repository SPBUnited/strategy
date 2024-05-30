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
            waypoints[field.allies[field.gk_id].r_id] = waypoint
        else:
            rC = 0
            for i in range(const.TEAM_ROBOTS_MAX_COUNT):
                if field.allies[i].is_used() and field.allies[i].r_id != field.gk_id:
                    waypoint = wp.Waypoint(aux.Point(3000 * -field.polarity, 2500 - 1000 * rC), 0, wp.WType.S_ENDPOINT)
                    waypoints[i] = waypoint
                    rC += 1
            waypoint = wp.Waypoint(
                field.ally_goal.center, aux.angle_to_point(field.ally_goal.center, field.ball.get_pos()), wp.WType.S_ENDPOINT
            )
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
                waypoint = wp.Waypoint(aux.Point(2000 * field.polarity, 1250 - 500 * rC), 0, wp.WType.S_ENDPOINT)
                waypoints[i] = waypoint
                rC += 1

    def penalty_kick(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
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

    def prepare_kickoff(self, field: fld.Field, waypoints: list[wp.Waypoint]) -> None:
        """Настройка перед состоянием kickoff по команде судей"""
        if self.we_active:
            self.we_kick = 1
        else:
            self.we_kick = 0
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