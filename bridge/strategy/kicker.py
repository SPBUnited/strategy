"""
Генерация объектов типа wp.Waypoint для роботов, движущихся с мячом
"""

import math
from time import time
from typing import Optional

import bridge.router.waypoint as wp
from bridge import const
from bridge.auxiliary import aux, fld, rbt


class KickerAux:
    def __init__(self) -> None:
        self.twist_w: float = 0.5
        self.kick_await_timer: Optional[float] = None

        self.robot_voltage: list[int] = [15 for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]

    def reset_kick_consts(self) -> None:
        "Обнулить константы для ударов"
        self.twist_w = 0.5
        self.kick_await_timer = None

    def pass_to_point(self, field: fld.Field, kicker: rbt.Robot, receive_pos: aux.Point) -> wp.Waypoint:
        "Пасс"
        ball = field.ball.get_pos()
        if aux.dist(receive_pos, kicker.get_pos()) < const.GOAL_DX:
            self.set_voltage(field, kicker.r_id, receive_pos, "PASS")
        else:
            self.set_voltage(field, kicker.r_id, receive_pos, "SHOOT")

        fld.find_nearest_robot(ball, field.enemies)
        if const.IS_SIMULATOR_USED:
            angle = aux.angle_to_point(ball, receive_pos)
            return wp.Waypoint(ball, angle, wp.WType.S_BALL_KICK)
        elif field.is_ball_in(kicker):
            return self.twisted(field, kicker, receive_pos)
        # elif aux.dist(ball, nearest_enemy.get_pos()) > 500:
        #     angle = aux.angle_to_point(kicker.get_pos(), ball)
        # angle = aux.angle_to_point(ball, receive_pos)
        #     return wp.Waypoint(ball, angle, wp.WType.S_BALL_KICK)
        else:
            self.reset_kick_consts()
            angle = aux.angle_to_point(kicker.get_pos(), ball)
            return wp.Waypoint(ball, angle, wp.WType.S_BALL_GRAB)

    def shoot_to_goal(self, field: fld.Field, kicker: rbt.Robot, shoot_point: aux.Point) -> wp.Waypoint:
        "Удар по воротам"

        print(shoot_point)
        ball = field.ball.get_pos()
        self.set_voltage(field, kicker.r_id, shoot_point, "SHOOT")

        fld.find_nearest_robot(ball, field.enemies)
        if const.IS_SIMULATOR_USED:
            angle = aux.angle_to_point(ball, shoot_point)
            return wp.Waypoint(ball, angle, wp.WType.S_BALL_KICK)
        elif field.is_ball_in(kicker):
            if aux.dist(ball, field.enemy_goal.center) < 1000:
                return self.angry_turn(kicker, shoot_point)
            return self.twisted(field, kicker, shoot_point)
        # elif aux.dist(ball, nearest_enemy.get_pos()) < 500:
        else:
            self.reset_kick_consts()
            angle = aux.angle_to_point(kicker.get_pos(), ball)
            return wp.Waypoint(ball, angle, wp.WType.S_BALL_GRAB)
        # else:
        #     target = aux.Point(shoot_point.x, 0)
        #     angle = aux.angle_to_point(ball, target)
        #     return wp.Waypoint(ball, angle, wp.WType.S_BALL_GRAB)

    def set_voltage(self, field: fld.Field, kicker_id: int, kick_point: aux.Point, style: str) -> None:
        "Set voltage for robot's kicker"
        angle_to_target = aux.angle_to_point(field.allies[kicker_id].get_pos(), kick_point)
        field.allies[kicker_id].kicker_charge_enable_ = 1
        if (
            aux.dist(field.allies[kicker_id].get_pos(), field.ball.get_pos()) > 200
            or abs(aux.wind_down_angle(field.allies[kicker_id].get_angle() - angle_to_target)) > 1
        ):
            # print(aux.dist(field.allies[kicker_id].get_pos(), field.ball.get_pos()), aux.wind_down_angle(field.allies[kicker_id].get_angle() - angle_to_target))
            field.allies[kicker_id].kicker_voltage_ = const.VOLTAGE_ZERO
        elif style == "PASS":
            field.allies[kicker_id].kicker_voltage_ = const.VOLTAGE_PASS
        elif style == "SHOOT":
            field.allies[kicker_id].kicker_voltage_ = const.VOLTAGE_SHOOT
        elif style == "UP":
            field.allies[kicker_id].kicker_voltage_ = const.VOLTAGE_UP

        # print("voltage", self.robot_voltage[kicker_id], "->", field.allies[kicker_id].kicker_voltage_)
        # if field.allies[kicker_id].kicker_voltage_ < self.robot_voltage[
        #     kicker_id
        # ] and not field.is_ball_in(field.allies[kicker_id]):
        #     field.allies[kicker_id].kick_forward()
        self.robot_voltage[kicker_id] = field.allies[kicker_id].kicker_voltage_

    def twisted(
        self,
        field: fld.Field,
        kicker: rbt.Robot,
        kick_point: aux.Point,
    ) -> wp.Waypoint:
        """
        Прицеливание и удар в точку
        """
        x = aux.angle_to_point(kicker.get_pos(), kick_point) - kicker.get_angle()

        if not const.IS_DRIBBLER_USED:
            waypoint = spin_around_ball(1 * aux.sign(x))
        else:
            nearest_enemy = fld.find_nearest_robot(kicker.get_pos(), field.enemies)
            angle_to_enemy = aux.get_angle_between_points(nearest_enemy.get_pos(), kicker.get_pos(), kick_point)
            angle_to_kick = aux.wind_down_angle(aux.angle_to_point(kicker.get_pos(), kick_point) - kicker.get_angle())
            if aux.dist(nearest_enemy.get_pos(), kicker.get_pos()) < 500 and abs(angle_to_enemy) < abs(angle_to_kick):
                if angle_to_enemy < 0 and x < 0:
                    x += 2 * math.pi
                elif angle_to_enemy > 0 and x > 0:
                    x -= 2 * math.pi
            else:
                x = aux.wind_down_angle(x)

            beta = 7
            a = beta / abs(x) - abs(self.twist_w) / (x**2)
            b = 2 * abs(x) * a - beta
            if abs(self.twist_w) > b * const.Ts:
                w = abs(self.twist_w) + b * const.Ts
            else:
                w = b * const.Ts - abs(self.twist_w)

            if x < 0:
                w *= -1

            if abs(x) < 0.1:
                w = x / 2

            self.twist_w = w
            waypoint = spin_with_ball(w)

        field.strategy_image.draw_line(
            kicker.get_pos(),
            kicker.get_pos() + aux.rotate(aux.RIGHT, kicker.get_angle()) * 5000,
        )

        if self.kick_await_timer is not None and time() - self.kick_await_timer > 0.5:
            kicker.kick_forward()
        elif abs(x) > const.KICK_ALIGN_ANGLE * 2:
            kicker.set_dribbler_speed(max(5, 15 - abs(w) * 5))
            self.kick_await_timer = None
        else:
            # kicker.kick_forward()
            # self.reset_kick_consts()
            if self.kick_await_timer is None:
                self.kick_await_timer = time()
            kicker.set_dribbler_speed(10)
            waypoint.pos.x += 100
        return waypoint

    def angry_turn(self, kicker: rbt.Robot, kick_point: aux.Point) -> wp.Waypoint:
        x = aux.wind_down_angle(aux.angle_to_point(kicker.get_pos(), kick_point) - kicker.get_angle())
        waypoint = spin_with_ball(1 * aux.sign(x))
        if abs(x) > const.KICK_ALIGN_ANGLE:
            kicker.set_dribbler_speed(12)
        else:
            kicker.kick_forward()
        return waypoint


def spin_with_ball(w: float) -> wp.Waypoint:
    """
    Расчёт скорости робота для поворота с мячом с угловой скоростью w (рад/сек)
    """
    if 0.01 < abs(w) < 0.4:
        w = math.copysign(0.4, w)

    if w > 0:
        delta_r = aux.Point(50, 25)
    else:
        delta_r = aux.Point(-50, 25)
    vel = delta_r * w

    return wp.Waypoint(vel, -w, wp.WType.S_VELOCITY)


def spin_around_ball(w: float) -> wp.Waypoint:
    """
    Расчёт скорости робота для поворота мячом с угловой скоростью w (рад/сек)
    """
    if 0.01 < abs(w) < 0.5:
        w = 0.5 * aux.sign(w)

    delta_r = aux.Point(0, 140)

    vel = delta_r * w

    k_w = 1.65  # костыль для приближения значений угловой скорости к рад/с
    k_vel = 0.4 * k_w
    return wp.Waypoint(vel * k_vel, w * k_w, wp.WType.S_VELOCITY)
