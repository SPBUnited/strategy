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
    """Class for smart turns with ball"""

    def __init__(self) -> None:
        self.twist_w: float = 0.5
        self.kick_await_timer: Optional[float] = None

        self.robot_voltage: list[int] = [15 for _ in range(const.TEAM_ROBOTS_MAX_COUNT)]

    def reset_kick_consts(self) -> None:
        "Обнулить константы для ударов"
        self.twist_w = 0.5
        self.kick_await_timer = None

    def twisted(
        self,
        field: fld.Field,
        kicker: rbt.Robot,
        kick_point: aux.Point,
    ) -> tuple[aux.Point, float]:
        """
        Прицеливание и удар в точку
        """
        x = aux.angle_to_point(kicker.get_pos(), kick_point) - kicker.get_angle()

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
        b = 2 * abs(x) * a - beta  # surrender, even me can't understand it

        if abs(x) < 0.15:
            w = x / 3
        else:
            w = b * const.Ts + abs(self.twist_w)
            if x < 0:
                w *= -1

        self.twist_w = w
        vel, wvel = spin_with_ball(w)

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
            vel.x += 100
        return vel, wvel


def pass_to_point(field: fld.Field, kicker: rbt.Robot, receive_pos: aux.Point) -> wp.Waypoint:
    "Пасс"
    ball = field.ball.get_pos()

    nearest_enemy = fld.find_nearest_robot(ball, field.enemies, [field.enemy_gk_id])
    is_enemy_near = aux.dist(ball, nearest_enemy.get_pos()) + 1000 < aux.dist(ball, kicker.get_pos())
    if 1 and (const.IS_SIMULATOR_USED or not is_enemy_near):  # NOTE
        angle = aux.angle_to_point(ball, receive_pos)
        return wp.Waypoint(ball, angle, wp.WType.S_BALL_KICK)

    angle = aux.angle_to_point(kicker.get_pos(), ball)
    return wp.Waypoint(ball, angle, wp.WType.S_BALL_TWIST_PASS)


def shoot_to_goal(field: fld.Field, kicker: rbt.Robot, shoot_point: aux.Point) -> wp.Waypoint:
    "Удар по воротам"
    ball = field.ball.get_pos()

    nearest_enemy = fld.find_nearest_robot(ball, field.enemies, [field.enemy_gk_id])
    is_enemy_near = aux.dist(ball, nearest_enemy.get_pos()) + 1000 < aux.dist(ball, kicker.get_pos())
    if 1 and (const.IS_SIMULATOR_USED or not is_enemy_near):  # NOTE
        angle = aux.angle_to_point(ball, shoot_point)
        return wp.Waypoint(ball, angle, wp.WType.S_BALL_KICK)

    # if aux.dist(ball, field.enemy_goal.center) < 1000:
    #     return angry_turn(kicker, shoot_point)

    angle = aux.angle_to_point(kicker.get_pos(), ball)
    return wp.Waypoint(ball, angle, wp.WType.S_BALL_TWIST)


# def angry_turn(kicker: rbt.Robot, kick_point: aux.Point) -> wp.Waypoint:
#     """Turn without stopping"""
#     x = aux.wind_down_angle(aux.angle_to_point(kicker.get_pos(), kick_point) - kicker.get_angle())
#     waypoint = spin_with_ball(1 * aux.sign(x))
#     if abs(x) > const.KICK_ALIGN_ANGLE:
#         kicker.set_dribbler_speed(12)
#     else:
#         kicker.kick_forward()
#     return waypoint


def spin_with_ball(w: float) -> tuple[aux.Point, float]:
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

    return (vel, -w)
